using System.Numerics;

namespace Scanner.App.Controls;

/// <summary>Interactive 3D view of a point cloud: drag to orbit, pinch to zoom.</summary>
public sealed class PointCloudView : View
{
    public static readonly BindableProperty PointsProperty =
        BindableProperty.Create(nameof(Points), typeof(Vector3[]), typeof(PointCloudView));

    public Vector3[]? Points
    {
        get => (Vector3[]?)GetValue(PointsProperty);
        set => SetValue(PointsProperty, value);
    }

    /// <summary>True between <see cref="Resume"/> and <see cref="Pause"/>; a handler connected later resumes rendering when set.</summary>
    internal bool IsResumeRequested { get; private set; }

    /// <summary>Starts or resumes the render thread.</summary>
    public void Resume()
    {
        IsResumeRequested = true;
        Handler?.Invoke(nameof(Resume));
    }

    /// <summary>Pauses the render thread (call when the page is hidden or the app is backgrounded).</summary>
    public void Pause()
    {
        IsResumeRequested = false;
        Handler?.Invoke(nameof(Pause));
    }
}
