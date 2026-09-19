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
}
