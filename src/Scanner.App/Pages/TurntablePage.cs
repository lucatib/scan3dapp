using System.Numerics;
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;
using Scanner.Core.Texturing;
using Scanner.Core.Turntable;

namespace Scanner.App.Pages;

/// <summary>
/// Turntable scan with the front depth camera: the phone stands still facing the piece (about 25 cm away), and the
/// piece turns on a turntable. Each depth frame is aligned to the piece fused so far (<see cref="TurntableFusion"/>);
/// a photo is kept every <see cref="PhotoEveryDegrees"/> of turn to texture the model.
/// </summary>
public sealed class TurntablePage : ContentPage
{
    private const float PhotoEveryDegrees = 15;

    private readonly SessionStore _store;
    private readonly TurntableView _camera = new();
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startButton = new() { Text = "Start" };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private readonly object _gate = new();
    private TurntableFusion? _fusion;
    private ScanSessionWriter? _writer;
    private string? _sessionId;
    private bool _recording;
    private bool _finishing;
    private int _busy;
    private float _lastPhotoTurn = float.NegativeInfinity;
    private string? _message;
    private DateTime _lastStatus;

    public TurntablePage(SessionStore store)
    {
        _store = store;
        Title = "Turntable";
        _startButton.Clicked += OnStartClicked;
        _finish.Clicked += OnFinishClicked;
        _camera.Captured += OnCaptured;
        var crosshair = new Label
        {
            Text = "+", FontSize = 40, TextColor = Colors.White, InputTransparent = true,
            HorizontalOptions = LayoutOptions.Center, VerticalOptions = LayoutOptions.Center,
        };
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startButton, _finish } } },
        };
        Content = new Grid { Children = { _camera, crosshair, panel } };
        _status.Text = "Stand the phone about 25 cm from the piece on the turntable, crosshair on the piece, then Start.";
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
            _fusion = null;
        }
    }

    private void OnStartClicked(object? sender, EventArgs e)
    {
        lock (_gate)
        {
            if (_recording || _finishing) return;
            var (id, directory) = _store.CreateNew();
            _sessionId = id;
            _writer = new ScanSessionWriter(directory, id, DeviceInfo.Current.Model);
            _fusion = null; // created from the first frame, whose centre is the target
            _lastPhotoTurn = float.NegativeInfinity;
            _recording = true;
        }
        _startButton.IsEnabled = false;
        _finish.IsEnabled = true;
        _status.Text = "Turn the piece slowly, one full turn…";
    }

    /// <summary>Capture thread. Frames that arrive while the previous one is being fused are dropped.</summary>
    private void OnCaptured(object? sender, DepthCapture capture)
    {
        if (!_recording || Interlocked.CompareExchange(ref _busy, 1, 0) != 0)
        {
            capture.Release();
            return;
        }
        Task.Run(() =>
        {
            try
            {
                Process(capture);
            }
            catch (Exception ex)
            {
                _message = ex.Message;
            }
            finally
            {
                capture.Release();
                Volatile.Write(ref _busy, 0);
            }
        });
    }

    private void Process(DepthCapture capture)
    {
        TurntableFusion fusion;
        ScanSessionWriter writer;
        lock (_gate)
        {
            if (!_recording || _writer is null) return;
            writer = _writer;
            if (_fusion is null)
            {
                if (CentreTarget(capture.Depth) is not { } target)
                {
                    _message = "No depth under the crosshair: move the phone to 20–40 cm";
                    return;
                }
                _fusion = new TurntableFusion(target);
            }
            fusion = _fusion;
        }

        var result = fusion.Add(capture.Depth);
        _message = result.Reason;
        if (result is { Accepted: true, CameraToObject: { } pose })
        {
            var toUpright = fusion.CameraToUpright(pose);
            lock (_gate)
            {
                if (!_recording) return;
                writer.AppendFrame(capture.Depth with { CameraToWorld = toUpright }, null);
                if (result.TurnDegrees - _lastPhotoTurn >= PhotoEveryDegrees)
                {
                    writer.AppendPhoto(capture.EncodeJpeg(),
                        new ScanPhotoData(capture.PhotoIntrinsics, toUpright, capture.Depth.TimestampSeconds, 0));
                    _lastPhotoTurn = result.TurnDegrees;
                }
            }
        }

        if (DateTime.UtcNow - _lastStatus < TimeSpan.FromMilliseconds(250)) return;
        _lastStatus = DateTime.UtcNow;
        string text = $"Turned {fusion.TurnDegrees:F0}° · {fusion.AcceptedFrames} frames · {writer.PhotoCount} photos"
                      + (fusion.TurnDegrees >= 360 ? "\nFull turn: press Finish (or keep turning for more detail)." : "")
                      + (_message is { } message ? $"\n{message}" : "");
        MainThread.BeginInvokeOnMainThread(() => { if (!_finishing) _status.Text = text; });
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        TurntableFusion? fusion;
        ScanSessionWriter? writer;
        string? sessionId;
        lock (_gate)
        {
            if (!_recording || _finishing) return;
            _recording = false;
            _finishing = true;
            fusion = _fusion;
            writer = _writer;
            sessionId = _sessionId;
        }
        _finish.IsEnabled = false;
        _camera.Stop();
        _status.Text = "Building the model…";
        try
        {
            while (Volatile.Read(ref _busy) != 0) await Task.Delay(20); // the last frame being fused
            if (fusion is null || writer is null || sessionId is null || fusion.AcceptedFrames == 0)
                throw new InvalidOperationException("No frame was recorded.");
            string directory = _store.DirectoryOf(sessionId);
            var triangles = await Task.Run(() =>
            {
                var mesh = fusion.Mesh();
                var target = Vector3.Transform(fusion.Target, fusion.Upright());
                writer.Complete(target, 0, mesh.Positions, new FittedSupportPlane(0, 0, 0), mesh.Positions.Count);
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
                _fusion = null;
            }
            _startButton.IsEnabled = true;
        }
    }

    /// <summary>The median depth point in a small window at the image centre (the crosshair), camera coordinates.</summary>
    private static Vector3? CentreTarget(DepthFrame frame)
    {
        var k = frame.Intrinsics;
        int cu = k.Width / 2, cv = k.Height / 2;
        var depths = new List<float>();
        for (int v = cv - 3; v <= cv + 3; v++)
        for (int u = cu - 3; u <= cu + 3; u++)
            if (frame.DepthAt(u, v) is > 0.08f and < 0.6f and var d) depths.Add(d);
        if (depths.Count < 10) return null;
        depths.Sort();
        float depth = depths[depths.Count / 2];
        return new Vector3((cu - k.Cx) / k.Fx * depth, (cv - k.Cy) / k.Fy * depth, depth);
    }
}
