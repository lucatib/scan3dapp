using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class FittedSupportPlaneTests
{
    private static List<Vector3> Scene()
    {
        var points = new List<Vector3>();
        for (int x = -40; x <= 40; x++)
        for (int z = -40; z <= 40; z++)
        {
            float px = x * .005f, pz = z * .005f;
            points.Add(new(px, .08f * px - .05f * pz, pz));
        }
        // Low curved object on a tilted table, with a connected 8 mm rim.
        for (int x = -8; x <= 8; x++)
        for (int z = -12; z <= 12; z++)
        {
            float px = x * .005f, pz = z * .005f;
            float height = .008f + .025f * MathF.Max(0, 1 - x*x/64f - z*z/144f);
            points.Add(new(px, .08f * px - .05f * pz + height, pz));
        }
        return points;
    }

    [Fact]
    public void Tilted_table_is_removed_without_cutting_the_low_object_rim()
    {
        var points = Scene();
        var plane = SupportPlaneFinder.Fit(points);
        Assert.NotNull(plane);
        Assert.InRange(plane.Value.HeightAt(.2f, -.2f), .024f, .028f);
        var result = ObjectIsolator.IsolateAbovePlane(points, new(0, .03f, 0), plane.Value, .01f);
        Assert.Equal(17 * 25, result.Length);
        Assert.True(result.All(p => MathF.Abs(p.X) <= .041f && MathF.Abs(p.Z) <= .061f));
    }

    [Fact]
    public void A_wall_is_not_a_support_plane()
    {
        var wall = new List<Vector3>();
        for (int x = 0; x < 50; x++)
        for (int y = 0; y < 50; y++) wall.Add(new(x*.005f, y*.005f, 0));
        Assert.Null(SupportPlaneFinder.Fit(wall));
    }

    [Fact]
    public void Empty_and_collinear_points_have_no_fitted_plane()
    {
        Assert.Null(SupportPlaneFinder.Fit([]));
        Assert.Null(SupportPlaneFinder.Fit(Enumerable.Range(0,100).Select(i => new Vector3(i*.005f,0,0)).ToArray()));
    }
}
