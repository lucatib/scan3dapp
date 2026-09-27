using Scanner.Capture.Live;
using Scanner.Core.Photogrammetry;

namespace Scanner.App.Controls;

/// <summary>Snapshot of the AR scan for the UI, raised a few times per second.</summary>
public sealed record ArScanStatus(
    string Tracking, LiveScanState State, int PointCount, int FrameCount, int PhotoCount, string? Message);

/// <summary>Full-screen AR camera view that feeds depth frames into <see cref="Session"/> and draws its points.</summary>
public sealed class ArScanView : View
{
    public static readonly BindableProperty SessionProperty =
        BindableProperty.Create(nameof(Session), typeof(LiveScanSession), typeof(ArScanView));

    public LiveScanSession? Session
    {
        get => (LiveScanSession?)GetValue(SessionProperty);
        set => SetValue(SessionProperty, value);
    }

    public static readonly BindableProperty LiveProperty =
        BindableProperty.Create(nameof(Live), typeof(LivePhotogrammetry), typeof(ArScanView));

    /// <summary>The photogrammetry that runs during the scan: it receives every photo and supplies the live surface.</summary>
    public LivePhotogrammetry? Live
    {
        get => (LivePhotogrammetry?)GetValue(LiveProperty);
        set => SetValue(LiveProperty, value);
    }

    /// <summary>True between <see cref="Resume"/> and <see cref="Pause"/>; a handler connected later starts AR when set.</summary>
    internal bool IsResumeRequested { get; private set; }

    public event EventHandler<ArScanStatus>? StatusChanged;

    /// <summary>Starts or resumes the camera and AR tracking.</summary>
    public void Resume()
    {
        IsResumeRequested = true;
        Handler?.Invoke(nameof(Resume));
    }

    /// <summary>Pauses the camera and AR tracking (call when the page is hidden or the app is backgrounded).</summary>
    public void Pause()
    {
        IsResumeRequested = false;
        Handler?.Invoke(nameof(Pause));
    }

    internal void ReportStatus(ArScanStatus status) => StatusChanged?.Invoke(this, status);
}
