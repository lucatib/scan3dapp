using System.Numerics;
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;
using Scanner.Core.Texturing;
using Scanner.Core.Turntable;

namespace Scanner.App.Pages;

/// <summary>
/// Turntable scan on the printed mat: the phone stands still on its back camera, the piece turns on the mat, and
/// every photo is posed from the mat's dots (<see cref="MatTurntableScan"/>). A photo is kept every
/// <see cref="MatTurntableScan.StepDegrees"/> of turn; the model is built from them at Finish.
/// </summary>
public sealed class MatTurntablePage : ContentPage
{
    private readonly SessionStore _store;
    private readonly PhotoCameraView _camera = new();
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startButton = new() { Text = "Start" };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private readonly object _gate = new();
    private MatTurntableScan? _scan;
    private ScanSessionWriter? _writer;
    private string? _sessionId;
    private bool _recording;
    private bool _finishing;
    private bool _ready;

    public MatTurntablePage(SessionStore store)
    {
        _store = store;
        Title = "Turntable (mat)";
        _startButton.Clicked += OnStartClicked;
        _finish.Clicked += OnFinishClicked;
        _camera.PhotoTaken += OnPhotoTaken;
        _camera.Failed += (_, message) =>
        {
            _status.Text = message;
            _startButton.IsEnabled = false;
        };
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startButton, _finish } } },
        };
        Content = new Grid { Children = { _camera, panel } };
        _status.Text = "Phone on a stand, piece in the middle of the mat, the whole dot ring in view.";
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        try
        {
            if (!_ready)
            {
                if (await ArPlatform.PrepareAsync() is { } error)
                {
                    _status.Text = error;
                    _startButton.IsEnabled = false;
                    return;
                }
                _ready = true;
            }
            _startButton.IsEnabled = !_recording && !_finishing;
            _camera.Start();
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not start the camera: {ex.Message}";
        }
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
                _writer = new ScanSessionWriter(directory, id, DeviceInfo.Current.Model);
                _sessionId = id;
                _scan = new MatTurntableScan();
                _recording = true;
            }
            _startButton.IsEnabled = false;
            _finish.IsEnabled = true;
            _status.Text = "Turn the mat slowly (or in small steps), one full turn…";
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not start the scan: {ex.Message}";
        }
    }

    /// <summary>Worker thread: poses the photo from the mat, keeps it if it adds an angle, shows the progress.</summary>
    private void OnPhotoTaken(object? sender, PhotoCapture photo)
    {
        string text;
        lock (_gate)
        {
            if (_finishing) return;
            var probe = _scan ?? new MatTurntableScan(); // before Start: only shows whether the mat is seen
            var result = probe.Offer(photo.Preview);
            if (result is { Kept: true, Pose: { } pose } && _recording && _writer is not null)
            {
                try
                {
                    // The pose comes from the small copy, but it is the same camera: the full picture's intrinsics stay.
                    _writer.AppendPhoto(photo.EncodeJpeg(), photo.Metadata with { CameraToWorld = pose.CameraToMat });
                }
                catch (Exception ex)
                {
                    probe.RemoveLast();
                    result = result with { Reason = $"Could not save a photo: {ex.Message}" };
                }
            }
            text = (_recording
                       ? $"{probe.CoveredSteps * MatTurntableScan.StepDegrees:F0}° of 360° covered · {probe.Views.Count} photos"
                       : result.Pose is { } seen ? $"Mat found ({seen.Dots} dots): press Start." : "")
                   + (result.Reason is { } reason ? $"\n{reason}" : "")
                   + (_recording && probe.CoveredSteps * MatTurntableScan.StepDegrees >= 360 ? "\nFull turn: press Finish." : "");
        }
        MainThread.BeginInvokeOnMainThread(() => { if (!_finishing && text.Length > 0) _status.Text = text; });
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        MatTurntableScan? scan;
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
        _status.Text = "Reconstructing photos — this may take a minute…";
        try
        {
            if (scan is null || writer is null || sessionId is null || scan.Views.Count < 3)
                throw new InvalidOperationException("Too few photos of the mat.");
            string directory = _store.DirectoryOf(sessionId);
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var triangles = await Task.Run(() =>
            {
                var reconstruction = scan.Reconstruct(log: new LogcatWriter())
                                     ?? throw new InvalidOperationException("The photos gave no surface.");
                var mesh = reconstruction.Mesh;
                writer.Complete(new Vector3(0, 0.01f, 0), 0, mesh.Positions, new FittedSupportPlane(0, 0, 0),
                    mesh.Positions.Count);
                if (mesh.TriangleCount > 0)
                {
                    var cameras = ScanSessionReader.ReadPhotos(directory).Select(p => TextureCamera.FromPhoto(p.Photo)).ToList();
                    TexturedModelFile.Write(directory, MeshTexturer.Texture(mesh, cameras));
                }
                return mesh.TriangleCount;
            });
            _status.Text = $"Model ready in {clock.Elapsed.TotalSeconds:F0} s: {triangles:N0} triangles.";
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
