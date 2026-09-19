using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Fusion;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Fusion;

public class TsdfVolumeTests
{
    [Fact]
    public void Set_and_get_work_with_negative_coordinates()
    {
        var volume = new TsdfVolume(0.002f, 0.006f);

        volume.Set(-1, -9, 17, 0.25f, 3f);

        Assert.True(volume.TryGet(-1, -9, 17, out float tsdf, out float weight));
        Assert.Equal(0.25f, tsdf);
        Assert.Equal(3f, weight);
        Assert.False(volume.TryGet(-2, -9, 17, out _, out _));
        Assert.False(volume.TryGet(100, 100, 100, out _, out _));
        Assert.True(Vector3.Distance(new Vector3(-0.002f, -0.018f, 0.034f), volume.VoxelToWorld(-1, -9, 17)) < 1e-7f);
    }

    [Fact]
    public void Integrate_single_view_sets_signed_values_along_optical_axis()
    {
        var k = new CameraIntrinsics(64, 48, 60f, 60f, 32f, 24f);
        var pose = CameraPoses.LookAt(new Vector3(0, 0, -0.1f), Vector3.Zero);
        var frame = SyntheticDepthRenderer.Render(new SphereSdf(Vector3.Zero, 0.02f), k, pose);
        var volume = new TsdfVolume(0.002f, 0.006f);

        volume.Integrate(frame);

        Assert.True(volume.BlockCount > 0);
        Assert.True(volume.TryGet(0, 0, -12, out float outside, out _));  // 4 mm outside the sphere
        Assert.Equal(0.667f, outside, 2);
        Assert.True(volume.TryGet(0, 0, -8, out float inside, out _));    // 4 mm inside the sphere
        Assert.Equal(-0.667f, inside, 2);
        Assert.False(volume.TryGet(0, 0, 0, out _, out _));               // center: beyond truncation
    }
}
