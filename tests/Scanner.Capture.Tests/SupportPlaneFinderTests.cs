using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class SupportPlaneFinderTests
{
    private const float Step = 0.005f;

    // A 40 cm table at y = 0 whose points are up to 2 mm off, as depth noise leaves them.
    private static void AddNoisyTable(List<Vector3> points, float half, Func<float, float, bool>? hidden = null)
    {
        var random = new Random(7);
        for (float x = -half; x <= half; x += Step)
        for (float z = -half; z <= half; z += Step)
            if (hidden is null || !hidden(x, z))
                points.Add(new Vector3(x, (float)(random.NextDouble() * 0.004 - 0.002), z));
    }

    // The faces a scan sees of a box standing on the table: the top and the four sides.
    private static void AddBoxSurface(List<Vector3> points, float half, float height)
    {
        for (float a = -half; a <= half; a += Step)
        for (float b = -half; b <= half; b += Step)
            points.Add(new Vector3(a, height, b));
        for (float y = Step; y < height; y += Step)
        for (float a = -half; a <= half; a += Step)
        {
            points.Add(new Vector3(a, y, -half));
            points.Add(new Vector3(a, y, half));
            points.Add(new Vector3(-half, y, a));
            points.Add(new Vector3(half, y, a));
        }
    }

    [Fact]
    public void Finds_a_noisy_table_under_a_box()
    {
        var points = new List<Vector3>();
        AddNoisyTable(points, 0.2f);
        AddBoxSurface(points, 0.04f, 0.08f);

        float? height = SupportPlaneFinder.Find(points);

        Assert.NotNull(height);
        Assert.InRange(height.Value, -0.003f, 0.003f);
    }

    // A wall has points at every height and no level that stands out; its lowest row is not a table.
    [Fact]
    public void Finds_nothing_without_a_horizontal_level()
    {
        var points = new List<Vector3>();
        for (float x = -0.2f; x <= 0.2f; x += Step)
        for (float y = 0; y <= 0.3f; y += Step)
            points.Add(new Vector3(x, y, 0));

        Assert.Null(SupportPlaneFinder.Find(points));
    }

    [Fact]
    public void Finds_nothing_in_an_empty_scan()
    {
        Assert.Null(SupportPlaneFinder.Find([]));
    }

    // A wide box hides most of the table, so its top face holds more points than the table ring around it.
    // The table is still the one below everything else.
    [Fact]
    public void Picks_the_table_under_a_top_face_bigger_than_the_visible_table()
    {
        var points = new List<Vector3>();
        AddNoisyTable(points, 0.2f, hidden: (x, z) => MathF.Abs(x) <= 0.15f && MathF.Abs(z) <= 0.15f);
        AddBoxSurface(points, 0.15f, 0.1f);

        float? height = SupportPlaneFinder.Find(points);

        Assert.NotNull(height);
        Assert.InRange(height.Value, -0.003f, 0.003f);
    }
}
