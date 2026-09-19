using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class DepthBackProjectorTests
{
    private static readonly CameraIntrinsics K = new(4, 3, 2f, 2f, 1.5f, 1f);

    private static DepthFrame FlatFrame(float depth, Matrix4x4 pose) =>
        new(K, Enumerable.Repeat(depth, K.Width * K.Height).ToArray(), pose, 0);

    [Fact]
    public void Principal_point_pixel_projects_on_the_optical_axis()
    {
        var frame = FlatFrame(1f, Matrix4x4.CreateTranslation(0, 0, 5));
        frame.Depth[1 * K.Width + 1] = 0; // u=1,v=1 invalid, only check u=… below
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, null, new DepthFilter(), points);

        Assert.Equal(11, points.Count);
        // Pixel (u=3, v=1): x = (3 - 1.5) / 2 * 1 = 0.75, y = 0, z = 1, then translated by +5 on Z.
        Assert.Contains(points, p => Vector3.Distance(p, new Vector3(0.75f, 0f, 6f)) < 1e-6f);
    }

    [Fact]
    public void Depth_range_and_confidence_filter_points()
    {
        var frame = FlatFrame(1f, Matrix4x4.Identity);
        frame.Depth[0] = 0.05f;  // too close
        frame.Depth[1] = 2.0f;   // too far
        var confidence = Enumerable.Repeat((byte)255, 12).ToArray();
        confidence[2] = 10;      // low confidence
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, confidence, new DepthFilter(), points);

        Assert.Equal(9, points.Count);
    }

    [Fact]
    public void Region_keeps_only_points_inside_the_sphere()
    {
        var frame = FlatFrame(1f, Matrix4x4.Identity);
        var region = new ScanRegion(new Vector3(0, 0, 1), 0.3f);
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, null, new DepthFilter(Region: region), points);

        Assert.NotEmpty(points);
        Assert.All(points, p => Assert.True(region.Contains(p)));
        Assert.True(points.Count < 12);
    }
}
