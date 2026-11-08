using System.Numerics;
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;
using Scanner.Core.Texturing;
using Scanner.Core.Turntable;

namespace Scanner.App.Pages;

/// <summary>
/// Turntable scan with the rear camera and LiDAR: the phone stands still on its stand, back towards the motorised
/// platter, and the piece turns. Each frame is posed from the platter's dot ring in the photo (ICP while the ring is
/// hidden) and its LiDAR depth fused in the platter's frame (<see cref="LidarMatTurntable"/>); a photo every
/// <see cref="PhotoEveryDegrees"/> of turn textures the model.
/// </summary>
public sealed class LidarTurntablePage : ContentPage
{
    private const float PhotoEveryDegrees = 10;

    private readonly SessionStore _store;
    private readonly TurntableView _camera = new() { Camera = DepthCamera.BackLidar };
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startButton = new() { Text = "Start" };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private readonly object _gate = new();
    private readonly HashSet<int> _photoSteps = new();
    private LidarMatTurntable? _scan;
    private ScanSessionWriter? _writer;
    private string? _sessionId;
    private bool _recording;
    private bool _finishing;
    private int _busy;
    private DateTime _lastStatus;

    public LidarTurntablePage(SessionStore store)
    {
        _store = store;
        Title = "Turntable (LiDAR)";
        _startButton.Clicked += OnStartClicked;
        _finish.Clicked += OnFinishClicked;
        _camera.Captured += OnCaptured;
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startButton, _finish } } },
        };
        Content = new Grid { Children = { _camera, panel } };
        _status.Text = "Phone on the stand, back towards the platter, the whole dot ring in view.";
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        if (await Permissions.RequestAsync<Permissions.Camera>() != PermissionStatus.Granted)
        {
            _status.Text = "Camera permission is required to scan.";
            return;
        }
        _camera.Start();
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        _camera.Stop();
        lock (_gate)
        {
            if (_finishing || _sessionId is null) return;
            _recording = false;
            _store.Delete(_sessionId); // leaving without finishing discards the scan
            _sessionId = null;
            _writer = null;
            _scan = null;
        }
    }

    private void OnStartClicked(object? sender, EventArgs e)
    {
        try
        {
            lock (_gate)
            {
                if (_recording || _finishing) return;
                var (id, directory) = _store.CreateNew();
                _sessionId = id;
                _writer = new ScanSessionWriter(directory, id, DeviceInfo.Current.Model);
                _scan = new LidarMatTurntable();
                _photoSteps.Clear();
                _recording = true;
            }
            _startButton.IsEnabled = false;
            _finish.IsEnabled = true;
            _status.Text = "Start the turntable: one full turn…";
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not start the scan: {ex.Message}";
        }
    }

    /// <summary>Capture thread. Frames that arrive while the previous one is being fused are dropped.</summary>
    private void OnCaptured(object? sender, DepthCapture capture)
    {
        if (_finishing || Interlocked.CompareExchange(ref _busy, 1, 0) != 0)
        {
            capture.Release();
            return;
        }
        Task.Run(() =>
        {
            string? text;
            try
            {
                text = Process(capture);
            }
            catch (Exception ex)
            {
                text = ex.Message;
            }
            finally
            {
                capture.Release();
                Volatile.Write(ref _busy, 0);
            }
            if (text is null || DateTime.UtcNow - _lastStatus < TimeSpan.FromMilliseconds(250)) return;
            _lastStatus = DateTime.UtcNow;
            MainThread.BeginInvokeOnMainThread(() => { if (!_finishing) _status.Text = text; });
        });
    }

    /// <summary>Worker thread: poses and fuses the frame, keeps a photo at each new angle; the status line.</summary>
    private string? Process(DepthCapture capture)
    {
        var preview = capture.Preview();
        LidarMatTurntable? scan;
        lock (_gate) scan = _recording ? _scan : null;
        if (scan is null)
        {
            // Before Start: only shows whether the ring is seen.
            var pose = MatPose.Solve(MatDetector.Detect(preview.Image), preview.Intrinsics);
            return pose is null
                ? "Dot ring not found: tilt the phone until the whole ring is in view."
                : $"Dot ring found ({pose.Dots} dots): press Start.";
        }

        var result = scan.Add(capture.Depth, preview);
        if (result is { Accepted: true, CameraToMat: { } cameraToMat, AngleDegrees: { } angle })
        {
            lock (_gate)
            {
                if (!_recording || _writer is not { } writer) return null;
                writer.AppendFrame(capture.Depth with { CameraToWorld = cameraToMat }, null);
                // Photos only from ring poses: they are exact, and the texture shows any error.
                if (result.Source == TurntablePoseSource.Mat && _photoSteps.Add((int)(angle / PhotoEveryDegrees)))
                    writer.AppendPhoto(capture.EncodeJpeg(),
                        new ScanPhotoData(capture.PhotoIntrinsics, cameraToMat, capture.Depth.TimestampSeconds, 0));
            }
        }

        int photos;
        lock (_gate) photos = _writer?.PhotoCount ?? 0;
        return $"{scan.CoveredDegrees:F0}° of 360° · {scan.MatFrames} ring + {scan.IcpFrames} ICP frames · {photos} photos"
               + (scan.CoveredDegrees >= 360 ? "\nFull turn: press Finish." : "")
               + (result.Reason is { } reason ? $"\n{reason}" : "");
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        LidarMatTurntable? scan;
        ScanSessionWriter? writer;
        string? sessionId;
        lock (_gate)
        {
            if (!_recording || _finishing) return;
            _recording = false;
            _finishing = true;
            (scan, writer, sessionId) = (_scan, _writer, _sessionId);
        }
        _finish.IsEnabled = false;
        _camera.Stop();
        _status.Text = "Building the model…";
        try
        {
            while (Volatile.Read(ref _busy) != 0) await Task.Delay(20); // the last frame being fused
            if (scan is null || writer is null || sessionId is null || scan.AcceptedFrames == 0)
                throw new InvalidOperationException("No frame was recorded: is the dot ring in view?");
            string directory = _store.DirectoryOf(sessionId);
            var triangles = await Task.Run(() =>
            {
                var mesh = scan.Mesh();
                writer.Complete(new Vector3(0, 0.01f, 0), 0, mesh.Positions, new FittedSupportPlane(0, 0, 0),
                    mesh.Positions.Count);
                if (mesh.TriangleCount > 0 && writer.PhotoCount > 0)
                {
                    var cameras = ScanSessionReader.ReadPhotos(directory).Select(p => TextureCamera.FromPhoto(p.Photo)).ToList();
                    TexturedModelFile.Write(directory, MeshTexturer.Texture(mesh, cameras));
                }
                return mesh.TriangleCount;
            });
            _status.Text = $"Model ready: {triangles:N0} triangles.";
            await Shell.Current.GoToAsync($"preview?id={sessionId}");
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not finish the scan: {ex.Message}";
        }
        finally
        {
            lock (_gate)
            {
                _finishing = false;
                _sessionId = null;
                _writer = null;
                _scan = null;
            }
            _startButton.IsEnabled = true;
        }
    }
}
