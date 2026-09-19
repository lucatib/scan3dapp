using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Live;
using Scanner.Capture.Sessions;

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

    /// <summary>Prepares the platform (permission, ARCore install) and creates the session once, then starts AR.</summary>
    private async Task StartAsync()
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
            _scan = new LiveScanSession(new ScanSessionWriter(directory, id, DeviceInfo.Current.Model));
            _arView.Session = _scan;
            _startPause.IsEnabled = true;
            _status.Text = "Aim the crosshair at the piece and press Start.";
        }
        _arView.Resume();
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
        if (_visible) await StartAsync(); // also retries preparation after the ARCore install flow
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
        _status.Text = "Isolating the piece…";
        scan.Pause();
        try
        {
            var result = await Task.Run(scan.Complete);
            _status.Text = $"Scan complete · {result.PiecePoints.Length:N0} points";
            await DisplayAlertAsync("Scan complete",
                $"{result.PiecePoints.Length:N0} points{(result.Isolated ? " (piece isolated from the table)" : "")}.", "OK");
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not finish the scan: {ex.Message}";
        }
    }

    private void OnStatusChanged(object? sender, ArScanStatus status)
    {
        if (_scan is null || _finishing || _scan.State == LiveScanState.Completed) return;
        _status.Text = $"{status.Tracking} · {status.State} · {status.PointCount:N0} points · {status.FrameCount} frames"
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
