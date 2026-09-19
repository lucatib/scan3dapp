using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class ObjectIsolatorTests
{
    private const float Step = 0.005f;

    // A 4 cm cube of points resting on a 40 cm table at y = 0, plus a second box 20 cm away.
    private static List<Vector3> Scene()
    {
        var points = new List<Vector3>();
        for (float x = -0.2f; x <= 0.2f; x += Step)
        for (float z = -0.2f; z <= 0.2f; z += Step)
            points.Add(new Vector3(x, 0, z));
        AddBox(points, new Vector3(0, 0.02f + Step, 0), 0.02f);
        AddBox(points, new Vector3(0.15f, 0.02f + Step, 0.15f), 0.02f);
        return points;
    }

    private static void AddBox(List<Vector3> points, Vector3 center, float half)
    {
        for (float x = -half; x <= half; x += Step)
        for (float y = -half; y <= half; y += Step)
        for (float z = -half; z <= half; z += Step)
            points.Add(center + new Vector3(x, y, z));
    }

    [Fact]
    public void Removes_table_and_keeps_only_the_piece_near_the_target()
    {
        var result = ObjectIsolator.Isolate(Scene(), target: new Vector3(0, 0.03f, 0), supportPlaneHeight: 0f, linkDistance: 0.01f);

        Assert.NotEmpty(result);
        Assert.All(result, p =>
        {
            Assert.True(p.Y > 0.004f);
            Assert.True(MathF.Abs(p.X) <= 0.021f && MathF.Abs(p.Z) <= 0.021f);
        });
    }

    [Fact]
    public void Without_plane_height_the_table_links_everything()
    {
        var scene = Scene();

        var result = ObjectIsolator.Isolate(scene, target: new Vector3(0, 0.03f, 0), supportPlaneHeight: null, linkDistance: 0.01f);

        Assert.Equal(scene.Count, result.Length);
    }

    [Fact]
    public void Picks_the_other_box_when_the_target_is_there()
    {
        var result = ObjectIsolator.Isolate(Scene(), target: new Vector3(0.15f, 0.03f, 0.15f), supportPlaneHeight: 0f, linkDistance: 0.01f);

        Assert.NotEmpty(result);
        Assert.All(result, p => Assert.True(p.X > 0.1f && p.Z > 0.1f));
    }

    [Fact]
    public void Empty_input_returns_empty()
    {
        Assert.Empty(ObjectIsolator.Isolate([], Vector3.Zero, 0f, 0.01f));
    }
}
