using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Photogrammetry;

public class StereoTests
{
    private static readonly Vector3 Target = new(0, 0.04f, 0);
    private static readonly CameraIntrinsics K = new(240, 180, 220, 220, 119.5f, 89.5f);

    // An 8 cm box standing on a 60 cm table whose top is at y = 0.
    private static ISdf Scene() => new UnionSdf([
        new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.3f, 0.01f, 0.3f)),
        new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
    ]);

    // Nine photos 10 degrees apart on an arc 35 cm from the box and 30 cm above it, as a slow walk around it.
    private static List<(PhotoView View, float[] Depth)> Photos(Func<Vector3, float>? texture = null)
    {
        var scene = Scene();
        var photos = new List<(PhotoView, float[])>();
        for (int i = 0; i < 9; i++)
        {
            float azimuth = (i - 4) * 10f * MathF.PI / 180f;
            var eye = Target + new Vector3(MathF.Sin(azimuth) * 0.35f, 0.3f, -MathF.Cos(azimuth) * 0.35f);
            var pose = CameraPoses.LookAt(eye, Target);
            var (image, depth) = SyntheticPhotoRenderer.Render(scene, K, pose, texture);
            photos.Add((new PhotoView(image, K, pose), depth));
        }
        return photos;
    }

    private static DepthMap Stereo(List<PhotoView> views, int reference)
    {
        var neighbours = ViewSelection.Neighbours(views, reference, Target, 4).Select(i => views[i]).ToList();
        float z = Vector3.Distance(views[reference].CameraToWorld.Translation, Target);
        return PlaneSweepStereo.Compute(views[reference], neighbours, z - 0.3f, z + 0.35f);
    }

    [Fact]
    public void Plane_sweep_recovers_the_depth_of_a_textured_scene()
    {
        var photos = Photos();
        var views = photos.Select(p => p.View).ToList();

        var map = Stereo(views, 4);

        var truth = photos[4].Depth;
        var errors = new List<float>();
        int truthCount = 0;
        for (int i = 0; i < truth.Length; i++)
        {
            if (truth[i] <= 0) continue;
            truthCount++;
            if (map.Depth[i] > 0) errors.Add(MathF.Abs(map.Depth[i] - truth[i]) / truth[i]);
        }
        errors.Sort();
        Assert.True(errors.Count >= truthCount / 2, $"only {errors.Count} of {truthCount} pixels have depth");
        Assert.True(errors[errors.Count / 2] <= 0.005f, $"median relative error {errors[errors.Count / 2]:P2}");
        Assert.True(errors[errors.Count * 9 / 10] <= 0.02f, $"90th percentile relative error {errors[errors.Count * 9 / 10]:P2}");
    }

    // Uniform pixels everywhere: a plain surface filling the view. (A plain object against the black background would
    // still show edges, and edges are real texture.)
    [Fact]
    public void Plane_sweep_leaves_a_textureless_image_empty()
    {
        var plain = new GrayImage(K.Width, K.Height, Enumerable.Repeat((byte)128, K.Width * K.Height).ToArray());
        var views = Photos().Select(p => p.View with { Image = plain }).ToList();

        var map = Stereo(views, 4);

        Assert.All(map.Depth, d => Assert.Equal(0f, d));
    }

    [Fact]
    public void Fusion_keeps_consistent_points_and_rejects_a_wrong_depth_map()
    {
        var scene = Scene();
        var views = Photos().Select(p => p.View).ToList();
        var maps = Enumerable.Range(2, 5).Select(r => (views[r], Stereo(views, r))).ToList();
        // A depth map 5 % too far, as a bad match or a wrong pose would leave it: its points must not survive.
        var (badView, badMap) = maps[2];
        maps[2] = (badView, new DepthMap(badMap.Width, badMap.Height, badMap.Depth.Select(d => d * 1.05f).ToArray(), badMap.Score));

        var points = DepthMapFusion.Fuse(maps);

        Assert.True(points.Count > 5000, $"only {points.Count} fused points");
        int onSurface = points.Count(p => MathF.Abs(scene.Distance(p)) <= 0.003f);
        Assert.True(onSurface >= points.Count * 95 / 100, $"{points.Count - onSurface} of {points.Count} points are off the surface");
    }

    [Fact]
    public void Neighbours_are_nearby_views_with_a_useful_angle()
    {
        var views = Photos().Select(p => p.View).ToList();
        // The same position as the reference: no baseline, so useless for stereo.
        views.Add(views[4]);

        var neighbours = ViewSelection.Neighbours(views, 4, Target, 4);

        Assert.Equal(4, neighbours.Count);
        Assert.DoesNotContain(4, neighbours);
        Assert.DoesNotContain(9, neighbours);
        Assert.Contains(3, neighbours);
        Assert.Contains(5, neighbours);
    }
}
