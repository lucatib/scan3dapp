using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Photogrammetry;

public class PhotoReconstructionTests
{
    private static readonly Vector3 Target = new(0, 0.04f, 0);
    private static readonly CameraIntrinsics K = new(240, 180, 220, 220, 119.5f, 89.5f);

    private static ISdf Scene() => new UnionSdf([
        new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.3f, 0.01f, 0.3f)),
        new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
    ]);

    // A walk once around the box: 24 photos, 15 degrees apart, 35 cm out and 30 cm up.
    private static List<PhotoView> Walk()
    {
        var scene = Scene();
        var views = new List<PhotoView>();
        for (int i = 0; i < 24; i++)
        {
            float azimuth = i * 15f * MathF.PI / 180f;
            var eye = Target + new Vector3(MathF.Sin(azimuth) * 0.35f, 0.3f, -MathF.Cos(azimuth) * 0.35f);
            var pose = CameraPoses.LookAt(eye, Target);
            views.Add(new PhotoView(SyntheticPhotoRenderer.Render(scene, K, pose).Image, K, pose));
        }
        return views;
    }

    [Fact]
    public void Dense_points_lie_on_the_scene_and_cover_the_box()
    {
        var scene = Scene();

        var points = PhotoReconstruction.DensePoints(Walk(), Target);

        Assert.True(points.Count > 5000, $"only {points.Count} points");
        int onSurface = points.Count(p => MathF.Abs(scene.Distance(p)) <= 0.003f);
        Assert.True(onSurface >= points.Count * 95 / 100, $"{points.Count - onSurface} of {points.Count} off the surface");
        // Every side of the box is seen by some reference view, so points reach all four side faces.
        foreach (var side in new[] { Vector3.UnitX, -Vector3.UnitX, Vector3.UnitZ, -Vector3.UnitZ })
            Assert.Contains(points, p => MathF.Abs(Vector3.Dot(p, side) - 0.04f) < 0.003f && p.Y > 0.01f && p.Y < 0.07f);
    }

    [Fact]
    public void Points_outside_the_scan_region_are_dropped()
    {
        var points = PhotoReconstruction.DensePoints(Walk(), Target, new ReconstructionOptions(RegionRadius: 0.1f));

        Assert.NotEmpty(points);
        Assert.All(points, p => Assert.True(Vector3.Distance(p, Target) <= 0.1f));
    }

    [Fact]
    public void Too_few_photos_give_no_points()
    {
        Assert.Empty(PhotoReconstruction.DensePoints(Walk().Take(1).ToList(), Target));
    }
}
