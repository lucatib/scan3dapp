using System.Numerics;
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Live;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;
using Scanner.Core.Photogrammetry;

namespace Scanner.App.Pages;

public sealed class ScanPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly ArScanView _arView = new();
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startPause = new() { Text = "Start", IsEnabled = false };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private LiveScanSession? _scan;
    private string? _sessionId;
    private Window? _window;
    private bool _visible;
    private bool _preparing;
    private bool _finishing;

    public ScanPage(SessionStore store)
    {
        _store = store;
        Title = "Scan";
        _startPause.Clicked += OnStartPauseClicked;
        _finish.Clicked += OnFinishClicked;
        _arView.StatusChanged += OnStatusChanged;

        var crosshair = new Label
        {
            Text = "+", FontSize = 40, TextColor = Colors.White, InputTransparent = true,
            HorizontalOptions = LayoutOptions.Center, VerticalOptions = LayoutOptions.Center,
        };
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startPause, _finish } } },
        };
        Content = new Grid { Children = { _arView, crosshair, panel } };
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        _visible = true;
        try
        {
            if (Window is { } window && !ReferenceEquals(window, _window))
            {
                // Subscribe before preparing: installing ARCore leaves the app, and Resumed brings us back here.
                DetachWindow();
                _window = window;
                window.Stopped += OnWindowStopped;
                window.Resumed += OnWindowResumed;
            }
            await StartAsync();
        }
        catch (Exception ex)
        {
            // async void: an exception here would be unobserved and would take the process down.
            _status.Text = $"Could not start the scan: {ex.Message}";
        }
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        _visible = false;
        DetachWindow();
        _arView.Pause();

        // Leaving without finishing discards the unfinished session.
        if (_scan is not null && _scan.State != LiveScanState.Completed && _sessionId is not null)
        {
            _arView.Session = null;
            _scan.Pause();
            _store.Delete(_sessionId);
            _scan = null;
            _sessionId = null;
        }
    }

    /// <summary>Prepares the platform (permission, ARCore install) and creates the session once, then starts AR.
    /// Never throws: it is awaited from async void entry points, where an exception would kill the process.</summary>
    private async Task StartAsync()
    {
        try
        {
            if (_scan is null)
            {
                if (_preparing) return;
                _preparing = true;
                string? error;
                try
                {
                    _status.Text = "Preparing the camera…";
                    error = await ArPlatform.PrepareAsync();
                }
                finally
                {
                    _preparing = false;
                }
                if (!_visible) return;
                if (error is not null)
                {
                    _status.Text = error;
                    _startPause.IsEnabled = false;
                    return;
                }
                var (id, directory) = _store.CreateNew();
                _sessionId = id;
                // ScanSessionWriter creates the session folder, so this can fail on a full or read-only volume.
                _scan = new LiveScanSession(new ScanSessionWriter(directory, id, DeviceInfo.Current.Model));
                _arView.Session = _scan;
                _startPause.IsEnabled = true;
                _status.Text = "Aim the crosshair at the piece and press Start.";
            }
            _arView.Resume();
        }
        catch (Exception ex)
        {
            // Expected: IOException and UnauthorizedAccessException from creating the session folder, and whatever
            // the platform preparation throws. Anything else must be shown rather than escape unobserved.
            // A session that was created stays usable (only AR failed to start); without one, Start stays disabled.
            _status.Text = $"Could not start the scan: {ex.Message}";
            _startPause.IsEnabled = _scan is not null;
        }
    }

    private void DetachWindow()
    {
        if (_window is null) return;
        _window.Stopped -= OnWindowStopped;
        _window.Resumed -= OnWindowResumed;
        _window = null;
    }

    private void OnWindowStopped(object? sender, EventArgs e) => _arView.Pause();

    private async void OnWindowResumed(object? sender, EventArgs e)
    {
        try
        {
            if (_visible) await StartAsync(); // also retries preparation after the ARCore install flow
        }
        catch (Exception ex)
        {
            // StartAsync already guards itself; this is the async void backstop that must never let anything escape.
            _status.Text = $"Could not resume the scan: {ex.Message}";
        }
    }

    private void OnStartPauseClicked(object? sender, EventArgs e)
    {
        if (_scan is null || _scan.State == LiveScanState.Completed) return;
        if (_scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget) _scan.Pause();
        else _scan.RequestStart();
        UpdateButtons();
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        if (_scan is null || _sessionId is null || _finishing) return;
        var scan = _scan;
        _finishing = true;
        _finish.IsEnabled = false;
        _startPause.IsEnabled = false;
        _status.Text = "Building the 3D preview from the photos…";
        scan.Pause();
        try
        {
            var result = await Task.Run(() => scan.Complete(depthPoints => WithPhotoPoints(scan, depthPoints)));
            _status.Text = result.Isolated ? "Piece isolated from the table." : "Could not isolate the piece; showing all points.";
            await Shell.Current.GoToAsync($"preview?id={_sessionId}");
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not finish the scan: {ex.Message}";
        }
    }

    /// <summary>
    /// Quick photogrammetry on the small in-memory copies of the photos, with ARCore's poses as they are, merged with
    /// ARCore's depth points (photos first, depth only filling gaps). The full reconstruction happens on the PC; this
    /// is a preview, so a failure falls back to the depth points rather than losing the scan.
    /// </summary>
    private static Vector3[] WithPhotoPoints(LiveScanSession scan, Vector3[] depthPoints)
    {
        if (scan.Target is not { } target) return depthPoints;
        try
        {
            var photoPoints = PhotoReconstruction.DensePoints(scan.PreviewPhotos(), target, PreviewOptions);
            return SourceMerge.Merge(photoPoints, depthPoints);
        }
        catch (Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Photo preview failed, keeping the depth points: {ex}");
            return depthPoints;
        }
    }

    // Measured on a 24-thread PC at 480x270: 12 photos x 96 depths take 6 s, 8 x 64 take 2.5 s; the phone is several
    // times slower, and this is only a preview.
    private static readonly ReconstructionOptions PreviewOptions =
        new(ReferenceViews: 8, Stereo: new StereoOptions(DepthSamples: 64));

    private void OnStatusChanged(object? sender, ArScanStatus status)
    {
        if (_scan is null || _finishing || _scan.State == LiveScanState.Completed) return;
        _status.Text = $"{status.Tracking} · {status.State} · {status.PointCount:N0} points · {status.FrameCount} frames"
                       + $" · {status.PhotoCount} photos"
                       + (status.Message is { } message ? $"\n{message}" : "");
        UpdateButtons();
    }

    private void UpdateButtons()
    {
        if (_scan is null || _scan.State == LiveScanState.Completed) return;
        bool running = _scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget;
        _startPause.Text = running ? "Pause" : _scan.FrameCount > 0 ? "Resume" : "Start";
        _finish.IsEnabled = _scan.FrameCount > 0;
    }
}
