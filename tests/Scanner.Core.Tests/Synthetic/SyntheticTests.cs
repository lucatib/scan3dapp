using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Synthetic;

public class SyntheticTests
{
    [Fact]
    public void Box_sdf_is_negative_inside_and_metric_outside()
    {
        var box = new BoxSdf(Vector3.Zero, new Vector3(0.02f));

        Assert.Equal(-0.02f, box.Distance(Vector3.Zero), 6);
        Assert.Equal(0.01f, box.Distance(new Vector3(0.03f, 0, 0)), 6);
    }

    [Fact]
    public void Tube_sdf_hole_is_outside_and_wall_is_inside()
    {
        var tube = new TubeSdf(Vector3.Zero, 0.02f, 0.01f, 0.03f);

        Assert.Equal(0.01f, tube.Distance(Vector3.Zero), 6);
        Assert.Equal(-0.005f, tube.Distance(new Vector3(0.015f, 0, 0)), 6);
    }

    [Fact]
    public void LookAt_points_camera_z_at_target_with_right_handed_axes()
    {
        var eye = new Vector3(0.1f, 0.05f, -0.02f);
        var m = CameraPoses.LookAt(eye, Vector3.Zero);

        var forward = Vector3.TransformNormal(Vector3.UnitZ, m);
        var right = Vector3.TransformNormal(Vector3.UnitX, m);
        var down = Vector3.TransformNormal(Vector3.UnitY, m);

        Assert.True(Vector3.Distance(forward, Vector3.Normalize(-eye)) < 1e-5f);
        Assert.True(Vector3.Distance(Vector3.Cross(right, down), forward) < 1e-5f);
        Assert.True(Vector3.Distance(m.Translation, eye) < 1e-6f);
    }

    [Fact]
    public void Renderer_measures_box_face_depth_on_optical_axis()
    {
        var k = new CameraIntrinsics(64, 48, 60f, 60f, 32f, 24f);
        var pose = CameraPoses.LookAt(new Vector3(0, 0, -0.1f), Vector3.Zero);

        var frame = SyntheticDepthRenderer.Render(new BoxSdf(Vector3.Zero, new Vector3(0.02f)), k, pose);

        Assert.Equal(0.08f, frame.DepthAt(32, 24), 4);
        Assert.Equal(0f, frame.DepthAt(0, 0));
    }

    [Fact]
    public void Scan_produces_one_frame_per_view()
    {
        var frames = SyntheticScan.Capture(new SphereSdf(Vector3.Zero, 0.02f), 5, 0.12f,
            new CameraIntrinsics(32, 24, 30f, 30f, 16f, 12f), 0f, 1);

        Assert.Equal(5, frames.Count);
        Assert.All(frames, f => Assert.Contains(f.Depth, d => d > 0));
    }
}
