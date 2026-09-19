using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

/// <summary>Shows a completed scan as an orbitable 3D point cloud.</summary>
[QueryProperty(nameof(SessionId), "id")]
public sealed class PreviewPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly PointCloudView _view = new();
    private readonly Label _info = new() { FontSize = 14 };

    /// <summary>Bottom row, shared with the action buttons added in a later task.</summary>
    private readonly VerticalStackLayout _panel;

    private string? _loadedId;

    public PreviewPage(SessionStore store)
    {
        _store = store;
        Title = "Preview";
        _panel = new VerticalStackLayout { Padding = 12, Spacing = 8, Children = { _info } };
        Content = new Grid
        {
            RowDefinitions = { new RowDefinition(GridLength.Star), new RowDefinition(GridLength.Auto) },
            Children = { _view, _panel },
        };
        Grid.SetRow(_panel, 1);
    }

    public string? SessionId { get; set; }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        if (SessionId is null || SessionId == _loadedId) return;
        _loadedId = SessionId;
        string directory = _store.DirectoryOf(SessionId);
        _info.Text = "Loading…";
        try
        {
            var (manifest, points) = await Task.Run(() =>
                (ScanSessionReader.ReadManifest(directory), ScanSessionReader.ReadPoints(directory)));
            _view.Points = points;
            _info.Text = $"{points.Length:N0} points · {manifest.FrameCount} frames · {manifest.CreatedUtc.LocalDateTime:g}\n"
                         + "Drag to rotate, pinch to zoom.";
        }
        catch (Exception ex) when (ex is IOException or InvalidDataException or System.Text.Json.JsonException)
        {
            _info.Text = $"Could not load the scan: {ex.Message}";
        }
    }
}
