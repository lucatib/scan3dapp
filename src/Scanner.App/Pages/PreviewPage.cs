using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

/// <summary>Shows a completed scan: an orbitable 3D point cloud, and the camera photos taken while it ran.</summary>
[QueryProperty(nameof(SessionId), "id")]
public sealed class PreviewPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly PointCloudView _view = new();
    private readonly Image _photo = new() { Aspect = Aspect.AspectFit, BackgroundColor = Colors.Black, IsVisible = false };
    private readonly Label _info = new() { FontSize = 14 };
    private readonly Button _cloudMode = new() { Text = "3D", IsEnabled = false };
    private readonly Button _photoMode = new() { Text = "Photos", IsEnabled = false };
    private readonly Button _previous = new() { Text = "‹", WidthRequest = 56 };
    private readonly Button _next = new() { Text = "›", WidthRequest = 56 };
    private readonly Label _photoInfo = new() { FontSize = 14, VerticalTextAlignment = TextAlignment.Center };
    private readonly HorizontalStackLayout _photoNav;

    /// <summary>Bottom row, shared with the action buttons added in a later task.</summary>
    private readonly VerticalStackLayout _panel;

    private IReadOnlyList<(ScanPhoto Photo, string ImagePath)> _photos = [];
    private int _index;
    private bool _showingPhotos;
    private string? _loadedId;
    private Window? _window;

    public PreviewPage(SessionStore store)
    {
        _store = store;
        Title = "Preview";
        _cloudMode.Clicked += (_, _) => ShowPhotos(false);
        _photoMode.Clicked += (_, _) => ShowPhotos(true);
        _previous.Clicked += (_, _) => Step(-1);
        _next.Clicked += (_, _) => Step(1);

        _photoNav = new HorizontalStackLayout
        {
            Spacing = 12, IsVisible = false, Children = { _previous, _photoInfo, _next },
        };
        _panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8,
            Children =
            {
                _info,
                new HorizontalStackLayout { Spacing = 12, Children = { _cloudMode, _photoMode } },
                _photoNav,
            },
        };
        Content = new Grid
        {
            RowDefinitions = { new RowDefinition(GridLength.Star), new RowDefinition(GridLength.Auto) },
            // The photo sits over the cloud in the same cell; only one of the two is ever visible.
            Children = { _view, _photo, _panel },
        };
        Grid.SetRow(_panel, 1);
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
        ResumeCloud();
        await LoadAsync();
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        DetachWindow();
        _view.Pause(); // the render thread and its GL context must not outlive the visible page
    }

    /// <summary>Loads the session once. Never throws: it is awaited from an async void entry point.</summary>
    private async Task LoadAsync()
    {
        if (SessionId is null || SessionId == _loadedId) return;
        _loadedId = SessionId;
        string directory = _store.DirectoryOf(SessionId);
        _info.Text = "Loading…";
        try
        {
            var (manifest, points, photos) = await Task.Run(() => (
                ScanSessionReader.ReadManifest(directory),
                ScanSessionReader.ReadPoints(directory),
                ScanSessionReader.ReadPhotos(directory)));
            _view.Points = points;
            _photos = photos;
            _index = 0;
            _photoMode.IsEnabled = photos.Count > 0;
            _info.Text = $"{points.Length:N0} points · {manifest.FrameCount} frames · {photos.Count} photos"
                         + $" · {manifest.CreatedUtc.LocalDateTime:g}\nDrag to rotate, pinch to zoom.";
            UpdatePhoto();
        }
        catch (Exception ex)
        {
            // Expected: IOException, UnauthorizedAccessException, JsonException, InvalidDataException. Anything
            // else would become an unobserved async void exception and take the process down with it.
            _info.Text = $"Could not load the scan: {ex.Message}";
        }
    }

    private void ShowPhotos(bool photos)
    {
        if (photos && _photos.Count == 0) return;
        _showingPhotos = photos;
        _photo.IsVisible = photos;
        _view.IsVisible = !photos;
        _photoNav.IsVisible = photos;
        _cloudMode.IsEnabled = photos;
        _photoMode.IsEnabled = !photos;
        // Hiding the surface alone would leave the render thread spinning on a cloud nobody can see.
        if (photos) _view.Pause();
        else _view.Resume();
        UpdatePhoto();
    }

    private void Step(int delta)
    {
        if (_photos.Count == 0) return;
        _index = ((_index + delta) % _photos.Count + _photos.Count) % _photos.Count;
        UpdatePhoto();
    }

    private void UpdatePhoto()
    {
        _previous.IsEnabled = _next.IsEnabled = _photos.Count > 1;
        if (_photos.Count == 0)
        {
            _photo.Source = null;
            _photoInfo.Text = "No photos in this scan.";
            return;
        }

        var (photo, path) = _photos[_index];
        _photo.Source = ImageSource.FromFile(path);
        // The intrinsics describe the sensor image, so this is the capture size however the phone was held - and
        // it is worth showing: it is set by the camera configuration ARCore picks, not by anything in this app.
        _photoInfo.Text = $"{_index + 1} / {_photos.Count}  ·  {photo.Intrinsics.Width}×{photo.Intrinsics.Height}";
    }

    /// <summary>Restarts the cloud's render thread unless the photos are the thing on screen.</summary>
    private void ResumeCloud()
    {
        if (!_showingPhotos) _view.Resume();
    }

    private void DetachWindow()
    {
        if (_window is null) return;
        _window.Stopped -= OnWindowStopped;
        _window.Resumed -= OnWindowResumed;
        _window = null;
    }

    private void OnWindowStopped(object? sender, EventArgs e) => _view.Pause();

    private void OnWindowResumed(object? sender, EventArgs e) => ResumeCloud();
}
