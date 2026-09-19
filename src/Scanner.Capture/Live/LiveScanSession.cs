using System.Numerics;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Live;

public enum LiveScanState { Idle, WaitingForTarget, Recording, Paused, Completed }

public sealed record LiveScanOptions(
    float VoxelSize = 0.005f,
    float RegionRadius = 0.30f,
    double IntegrationIntervalSeconds = 0.2,
    DepthFilter? Filter = null,
    int MinIsolatedPoints = 200);

public sealed record LiveScanResult(Vector3[] AllPoints, Vector3[] PiecePoints, bool Isolated);

/// <summary>
/// Platform-independent state of one live scan: throttles depth integration, accumulates points inside the
/// region around the target, records frames, and isolates the piece on completion.
/// <see cref="Integrate"/> runs on one worker thread at a time; <see cref="SnapshotPoints"/> may run concurrently
/// on the render thread; state methods run on the UI/render thread.
/// </summary>
public sealed class LiveScanSession
{
    private readonly ScanSessionWriter _writer;
    private readonly VoxelPointAccumulator _accumulator;
    private readonly object _gate = new();
    private LiveScanState _state = LiveScanState.Idle;
    private double _lastIntegration = double.NegativeInfinity;

    public LiveScanSession(ScanSessionWriter writer, LiveScanOptions? options = null)
    {
        _writer = writer;
        Options = options ?? new LiveScanOptions();
        _accumulator = new VoxelPointAccumulator(Options.VoxelSize);
    }

    public LiveScanOptions Options { get; }

    public LiveScanState State
    {
        get { lock (_gate) return _state; }
    }

    public Vector3? Target { get; private set; }
    public float? SupportPlaneHeight { get; private set; }
    public int PointCount => _accumulator.CellCount;

    public int FrameCount
    {
        get { lock (_gate) return _writer.FrameCount; }
    }

    /// <summary>Idle → WaitingForTarget (the renderer then picks the target); Paused → Recording.</summary>
    public void RequestStart()
    {
        lock (_gate)
        {
            _state = _state switch
            {
                LiveScanState.Idle => LiveScanState.WaitingForTarget,
                LiveScanState.Paused => LiveScanState.Recording,
                LiveScanState.WaitingForTarget or LiveScanState.Recording => _state,
                _ => throw new InvalidOperationException("The scan is already completed."),
            };
        }
    }

    /// <summary>Sets the aimed-at world point and the support plane height (world +Y up), and starts recording.</summary>
    public void SetTarget(Vector3 target, float? supportPlaneHeight)
    {
        lock (_gate)
        {
            if (_state != LiveScanState.WaitingForTarget)
                throw new InvalidOperationException($"Cannot set the target while {_state}.");
            Target = target;
            SupportPlaneHeight = supportPlaneHeight;
            _state = LiveScanState.Recording;
        }
    }

    public void Pause()
    {
        lock (_gate)
        {
            _state = _state switch
            {
                LiveScanState.Recording => LiveScanState.Paused,
                LiveScanState.WaitingForTarget => LiveScanState.Idle,
                _ => _state,
            };
        }
    }

    /// <summary>True when recording and at least <see cref="LiveScanOptions.IntegrationIntervalSeconds"/> passed since the last accepted frame.</summary>
    public bool ShouldIntegrate(double timestampSeconds)
    {
        lock (_gate)
        {
            if (_state != LiveScanState.Recording) return false;
            if (timestampSeconds - _lastIntegration < Options.IntegrationIntervalSeconds) return false;
            _lastIntegration = timestampSeconds;
            return true;
        }
    }

    /// <summary>Back-projects the frame, accumulates the points inside the region and records the frame.
    /// Returns the number of points added (0 once the scan is completed).</summary>
    public int Integrate(DepthFrame frame, byte[]? confidence)
    {
        Vector3? target;
        lock (_gate)
        {
            if (_state == LiveScanState.Completed) return 0;
            target = Target;
        }

        var filter = (Options.Filter ?? new DepthFilter()) with
        {
            Region = target is { } t ? new ScanRegion(t, Options.RegionRadius) : null,
        };
        var points = new List<Vector3>();
        DepthBackProjector.Project(frame, confidence, filter, points);

        lock (_gate)
        {
            if (_state == LiveScanState.Completed) return 0;
            _accumulator.AddRange(points);
            _writer.AppendFrame(frame, confidence);
        }
        return points.Count;
    }

    public Vector3[] SnapshotPoints() => _accumulator.Snapshot();

    /// <summary>Stops the scan, isolates the piece around the target, writes points and manifest.</summary>
    public LiveScanResult Complete()
    {
        lock (_gate)
        {
            if (_state == LiveScanState.Completed) throw new InvalidOperationException("The scan is already completed.");
            _state = LiveScanState.Completed;

            var all = _accumulator.Snapshot();
            var piece = Target is { } t
                ? ObjectIsolator.Isolate(all, t, SupportPlaneHeight, 2 * Options.VoxelSize)
                : all;
            bool isolated = Target is not null && piece.Length >= Options.MinIsolatedPoints;
            if (!isolated) piece = all;

            _writer.Complete(Target, SupportPlaneHeight, piece);
            return new LiveScanResult(all, piece, isolated);
        }
    }
}
