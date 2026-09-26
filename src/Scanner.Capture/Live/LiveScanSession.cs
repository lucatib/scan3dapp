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
    int MinIsolatedPoints = 200,
    double PhotoIntervalSeconds = 0.5,
    int MaxPhotos = 90);

public sealed record LiveScanResult(Vector3[] AllPoints, Vector3[] PiecePoints, bool Isolated);

/// <summary>
/// Platform-independent state of one live scan: throttles depth integration, accumulates points inside the
/// region around the target, records frames and camera photos, and isolates the piece on completion.
/// <see cref="Integrate"/> runs on one worker thread at a time; <see cref="SnapshotPoints"/> may run concurrently
/// on the render thread; state methods run on the UI/render thread.
/// <see cref="State"/> and <see cref="FrameCount"/> are read several times a second on the UI thread and never
/// take a lock, so the UI cannot be blocked by a frame write.
/// </summary>
public sealed class LiveScanSession
{
    private readonly ScanSessionWriter _writer;
    private readonly VoxelPointAccumulator _accumulator;

    /// <summary>Guards the state transitions and the target. Never held across file I/O.</summary>
    private readonly object _gate = new();

    /// <summary>Guards the recorded data (accumulator + writer). Taken by integration workers and
    /// <see cref="Complete"/> only, never by the UI thread, and never while <see cref="_gate"/> is held.</summary>
    private readonly object _writeGate = new();

    private volatile LiveScanState _state = LiveScanState.Idle;
    private double _lastIntegration = double.NegativeInfinity;
    private double _lastPhoto = double.NegativeInfinity;
    private int _frameCount;

    /// <summary>Photos promised by <see cref="ShouldCapturePhoto"/>, which is what the cap counts: encoding one
    /// takes long enough that several can be in flight, and counting only the written ones would let a burst
    /// sail past <see cref="LiveScanOptions.MaxPhotos"/>. Guarded by <see cref="_gate"/>.</summary>
    private int _photosReserved;

    private int _photoCount;

    /// <summary>Small copies of the photos for the on-phone preview. Guarded by <see cref="_writeGate"/>.</summary>
    private readonly List<PhotoView> _previews = [];

    public LiveScanSession(ScanSessionWriter writer, LiveScanOptions? options = null)
    {
        _writer = writer;
        Options = options ?? new LiveScanOptions();
        _accumulator = new VoxelPointAccumulator(Options.VoxelSize);
    }

    public LiveScanOptions Options { get; }

    public LiveScanState State => _state;

    public Vector3? Target { get; private set; }
    public float? SupportPlaneHeight { get; private set; }
    public int PointCount => _accumulator.CellCount;

    /// <summary>Frames written so far; published only after the frame is on disk.</summary>
    public int FrameCount => Volatile.Read(ref _frameCount);

    /// <summary>Photos written so far; published only after the picture and its metadata are on disk.</summary>
    public int PhotoCount => Volatile.Read(ref _photoCount);

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

    /// <summary>True when recording, the photo interval has elapsed and the cap has room. Reserves the slot, so
    /// a caller that asks must go on to call <see cref="AddPhoto"/> or forfeit it.</summary>
    public bool ShouldCapturePhoto(double timestampSeconds)
    {
        lock (_gate)
        {
            if (_state != LiveScanState.Recording) return false;
            if (_photosReserved >= Options.MaxPhotos) return false;
            if (timestampSeconds - _lastPhoto < Options.PhotoIntervalSeconds) return false;
            _lastPhoto = timestampSeconds;
            _photosReserved++;
            return true;
        }
    }

    /// <summary>Records one camera photo. Returns false once the scan is completed, exactly as <see cref="Integrate"/>
    /// does: the JPEG is encoded off the render thread, so it can arrive after the user has pressed Finish.</summary>
    public bool AddPhoto(byte[] jpeg, ScanPhotoData photo, PhotoView? preview = null)
    {
        // The same write gate as the frames, so a photo cannot land in a session whose manifest is already written.
        lock (_writeGate)
        {
            if (_state == LiveScanState.Completed) return false;
            _writer.AppendPhoto(jpeg, photo);
            if (preview is not null) _previews.Add(preview);
            Volatile.Write(ref _photoCount, _writer.PhotoCount);
            return true;
        }
    }

    /// <summary>The in-memory preview copies of the photos recorded so far, in capture order.</summary>
    public PhotoView[] PreviewPhotos()
    {
        lock (_writeGate) return _previews.ToArray();
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

        // Outside _gate: the write is a File.Create per frame and the UI polls State/FrameCount while it runs.
        // _writeGate keeps accumulation and the frame write atomic and ordered, and re-checks completion so that
        // a Complete that started meanwhile cannot be followed by a frame it did not count.
        lock (_writeGate)
        {
            if (_state == LiveScanState.Completed) return 0;
            _accumulator.AddRange(points);
            _writer.AppendFrame(frame, confidence);
            Volatile.Write(ref _frameCount, _writer.FrameCount);
        }
        return points.Count;
    }

    public Vector3[] SnapshotPoints() => _accumulator.Snapshot();

    /// <summary>Stops the scan, isolates the piece around the target, writes points and manifest.</summary>
    public LiveScanResult Complete()
    {
        Vector3? target;
        float? supportPlaneHeight;
        lock (_gate)
        {
            if (_state == LiveScanState.Completed) throw new InvalidOperationException("The scan is already completed.");
            _state = LiveScanState.Completed; // no integration entering the write gate from here on appends a frame
            target = Target;
            supportPlaneHeight = SupportPlaneHeight;
        }

        // Waits for an integration already inside the write gate, so every frame written is counted here.
        lock (_writeGate)
        {
            var all = _accumulator.Snapshot();
            // ARCore often has not found the table yet when Start is pressed; then it is found in the scan itself,
            // or it is never cut away and links everything into one piece.
            supportPlaneHeight ??= SupportPlaneFinder.Find(all);
            var piece = target is { } t
                ? ObjectIsolator.Isolate(all, t, supportPlaneHeight, 2 * Options.VoxelSize)
                : all;
            bool isolated = target is not null && piece.Length >= Options.MinIsolatedPoints;
            if (!isolated) piece = all;

            _writer.Complete(target, supportPlaneHeight, piece);
            return new LiveScanResult(all, piece, isolated);
        }
    }
}
