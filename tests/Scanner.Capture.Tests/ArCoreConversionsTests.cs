using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.ArCore;

namespace Scanner.Capture.Tests;

public class ArCoreConversionsTests
{
    private static readonly float[] Identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

    [Fact]
    public void Identity_gl_pose_looks_down_negative_world_z_with_image_down_as_world_down()
    {
        var m = ArCoreConversions.CameraToWorldFromGlPose(Identity);

        Assert.Equal(-Vector3.UnitZ, Vector3.TransformNormal(Vector3.UnitZ, m));
        Assert.Equal(-Vector3.UnitY, Vector3.TransformNormal(Vector3.UnitY, m));
        Assert.Equal(Vector3.UnitX, Vector3.TransformNormal(Vector3.UnitX, m));
    }

    [Fact]
    public void Translation_is_preserved()
    {
        float[] pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1];

        var m = ArCoreConversions.CameraToWorldFromGlPose(pose);

        Assert.Equal(new Vector3(1, 2, 3), Vector3.Transform(Vector3.Zero, m));
    }

    [Fact]
    public void Camera_rotated_90_degrees_about_y_looks_down_negative_world_x()
    {
        // Column-major Ry(+90°): columns (0,0,-1), (0,1,0), (1,0,0).
        float[] pose = [0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1];

        var m = ArCoreConversions.CameraToWorldFromGlPose(pose);

        Assert.True(Vector3.Distance(-Vector3.UnitX, Vector3.TransformNormal(Vector3.UnitZ, m)) < 1e-6f);
    }

    [Fact]
    public void Wrong_length_throws()
    {
        Assert.Throws<ArgumentException>(() => ArCoreConversions.CameraToWorldFromGlPose(new float[12]));
    }

    [Fact]
    public void Intrinsics_scale_to_depth_resolution_with_pixel_center_convention()
    {
        var k = ArCoreConversions.ScaleIntrinsics(500f, 500f, 319.5f, 239.5f, 640, 480, 160, 120);

        Assert.Equal(160, k.Width);
        Assert.Equal(120, k.Height);
        Assert.Equal(125f, k.Fx, 4);
        Assert.Equal(125f, k.Fy, 4);
        Assert.Equal(79.5f, k.Cx, 4);
        Assert.Equal(59.5f, k.Cy, 4);
    }

    [Fact]
    public void Millimeters_become_meters_and_zero_stays_invalid()
    {
        var k = new CameraIntrinsics(2, 1, 1, 1, 0, 0);

        var frame = ArCoreConversions.DepthFrameFromMillimeters([1500, 0], k, Matrix4x4.Identity, 2.5);

        Assert.Equal(1.5f, frame.Depth[0], 6);
        Assert.Equal(0f, frame.Depth[1]);
        Assert.Equal(2.5, frame.TimestampSeconds);
    }
}
