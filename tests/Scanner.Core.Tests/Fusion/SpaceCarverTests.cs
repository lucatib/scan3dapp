using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Fusion;

public class SpaceCarverTests
{
    private const float Voxel = 0.004f;

    /// <summary>
    /// A 6 cm post on a table stereo did not match (like plain or mirroring wood), photographed from 50° above over
    /// 250° of the loop, as in the grinder scan that grew wings. The post stays solid; the space beside it on the side
    /// no photo faced, which only the photos opposite see behind the post, stays empty.
    /// </summary>
    [Fact]
    public void Space_only_a_few_maps_see_behind_the_piece_stays_empty()
    {
        var post = new BoxSdf(new Vector3(0, 0.06f, 0), new Vector3(0.03f, 0.06f, 0.03f));
        var scene = new UnionSdf([post, new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.5f, 0.01f, 0.5f))]);
        var k = new CameraIntrinsics(160, 120, 150f, 150f, 80f, 60f);
        var maps = new List<(PhotoView, float[])>();
        var look = new Vector3(0, 0.05f, 0);
        for (int i = 0; i < 12; i++)
        {
            float azimuth = i * 250f / 11 * MathF.PI / 180, elevation = 50 * MathF.PI / 180;
            var eye = look + 0.4f * new Vector3(MathF.Cos(elevation) * MathF.Cos(azimuth), MathF.Sin(elevation),
                MathF.Cos(elevation) * MathF.Sin(azimuth));
            var view = new PhotoView(new GrayImage(k.Width, k.Height, new byte[k.Width * k.Height]), k,
                CameraPoses.LookAt(eye, look));
            var depth = SyntheticDepthRenderer.Render(scene, k, view.CameraToWorld).Depth;
            for (int v = 0, index = 0; v < k.Height; v++)
            for (int u = 0; u < k.Width; u++, index++)
                if (depth[index] > 0 && Pinhole.BackProject(view, u, v, depth[index]).Y < 0.001f)
                    depth[index] = 0; // the table matched nowhere
            maps.Add((view, depth));
        }
        var volume = new TsdfVolume(Voxel, 4 * Voxel);

        SpaceCarver.Complete(volume, maps, new Vector3(-0.12f, 0, -0.12f), new Vector3(0.12f, 0.14f, 0.12f),
            new FittedSupportPlane(0, 0, 0, 0), 1);

        Assert.True(Value(volume, new Vector3(0, 0.06f, 0)) < 0, "the post is solid");
        float gap = (250 + 55) * MathF.PI / 180;
        var beside = new Vector3(0.06f * MathF.Cos(gap), 0.06f, 0.06f * MathF.Sin(gap));
        Assert.True(Value(volume, beside) > 0, "the unseen side of the post is empty");
    }

    private static float Value(TsdfVolume volume, Vector3 p)
    {
        Assert.True(volume.TryGet((int)MathF.Round(p.X / Voxel), (int)MathF.Round(p.Y / Voxel), (int)MathF.Round(p.Z / Voxel),
            out float tsdf, out _));
        return tsdf;
    }
}
