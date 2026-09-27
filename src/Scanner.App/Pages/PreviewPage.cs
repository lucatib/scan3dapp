using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

/// <summary>Shows the saved ARCore depth scan as an orbitable 3D point cloud.</summary>
[QueryProperty(nameof(SessionId), "id")]
public sealed class PreviewPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly PointCloudView _view = new();
    private readonly Label _info = new() { FontSize = 14 };
    private string? _loadedId;
    private Window? _window;

    public PreviewPage(SessionStore store)
    {
        _store = store;
        Title = "3D scan";
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, Children = { _info },
        };
        Content = new Grid
        {
            RowDefinitions = { new RowDefinition(GridLength.Star), new RowDefinition(GridLength.Auto) },
            Children = { _view, panel },
        };
        Grid.SetRow(panel, 1);
    }

    public string? SessionId { get; set; }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        if (Window is { } window && !ReferenceEquals(window, _window))
        {
            DetachWindow();
            _window = window;
            window.Stopped += OnWindowStopped;
            window.Resumed += OnWindowResumed;
        }
        _view.Resume();
        await LoadAsync();
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        DetachWindow();
        _view.Pause();
    }

    private async Task LoadAsync()
    {
        if (SessionId is null || SessionId == _loadedId) return;
        string id = SessionId;
        string directory = _store.DirectoryOf(id);
        _info.Text = "Loading 3D scan…";
        try
        {
            var (manifest, points) = await Task.Run(() => (
                ScanSessionReader.ReadManifest(directory),
                ScanSessionReader.ReadPoints(directory)));
            _view.Points = points;
            _loadedId = id;
            _info.Text = $"{points.Length:N0} points · {manifest.FrameCount} depth frames"
                         + $" · {manifest.CreatedUtc.LocalDateTime:g}\nDrag to rotate, pinch to zoom.";
        }
        catch (Exception ex)
        {
            _info.Text = $"Could not load the scan: {ex.Message}";
        }
    }

    private void DetachWindow()
    {
        if (_window is null) return;
        _window.Stopped -= OnWindowStopped;
        _window.Resumed -= OnWindowResumed;
        _window = null;
    }

    private void OnWindowStopped(object? sender, EventArgs e) => _view.Pause();

    private void OnWindowResumed(object? sender, EventArgs e) => _view.Resume();
}
