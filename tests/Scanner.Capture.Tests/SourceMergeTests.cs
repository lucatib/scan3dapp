using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class SourceMergeTests
{
    [Fact]
    public void Photo_points_are_kept_and_depth_points_only_fill_gaps()
    {
        Vector3[] photo = [new(0, 0, 0), new(0.1f, 0, 0)];
        // One ARCore point 6 mm from a photo point (redundant, and less precise), one 5 cm from any (a gap).
        Vector3[] depth = [new(0.006f, 0, 0), new(0, 0.05f, 0)];

        var merged = SourceMerge.Merge(photo, depth);

        Assert.Equal(3, merged.Length);
        Assert.Contains(merged, p => Vector3.Distance(p, new Vector3(0, 0.05f, 0)) < 0.003f);
        Assert.DoesNotContain(merged, p => Vector3.Distance(p, new Vector3(0.006f, 0, 0)) < 0.002f);
    }

    [Fact]
    public void Nearby_points_of_one_source_share_a_voxel()
    {
        Vector3[] photo = [new(0.001f, 0.001f, 0.001f), new(0.002f, 0.002f, 0.002f)];

        var merged = SourceMerge.Merge(photo, []);

        var point = Assert.Single(merged);
        Assert.Equal(0.0015f, point.X, 5);
    }

    [Fact]
    public void Without_photo_points_the_depth_points_are_all_kept()
    {
        Vector3[] depth = [new(0, 0, 0), new(0.2f, 0, 0)];

        Assert.Equal(2, SourceMerge.Merge([], depth).Length);
    }
}
