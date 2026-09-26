using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Finds the height of the surface a scanned piece stands on, from the scan's own points
/// (world frame, +Y up), or null when no height level stands out.</summary>
public static class SupportPlaneFinder
{
    public static float? Find(IReadOnlyList<Vector3> points, float binSize = 0.005f)
    {
        if (points.Count == 0) return null;

        var counts = new Dictionary<int, int>();
        foreach (var p in points)
        {
            int bin = Bin(p.Y, binSize);
            counts[bin] = counts.GetValueOrDefault(bin) + 1;
        }

        // A level is a bin with its two neighbours, so a surface whose noise straddles a bin edge still counts whole.
        int Window(int bin) => counts.GetValueOrDefault(bin - 1) + counts.GetValueOrDefault(bin) + counts.GetValueOrDefault(bin + 1);
        int busiest = counts.Keys.Max(Window);

        // The support is below everything it carries, so it is the lowest level with a real share of the points,
        // not the busiest one: a wide piece can hide most of the table and show more top face than table.
        // A real share is half the busiest level and a tenth of the scan. Device scans put 18-41% of their points
        // on the table level; a wall, with points at every height, puts about 5% on any one level.
        var levels = counts.Keys.Where(bin => 2 * Window(bin) >= busiest && 10 * Window(bin) >= points.Count).ToList();
        if (levels.Count == 0) return null;
        int best = levels.Min();
        // The lowest qualifying bin can sit on the rising flank of that level; settle on its peak.
        while (Window(best + 1) > Window(best)) best++;

        float sum = 0;
        int n = 0;
        foreach (var p in points)
        {
            if (Math.Abs(Bin(p.Y, binSize) - best) > 1) continue;
            sum += p.Y;
            n++;
        }
        return sum / n;
    }

    private static int Bin(float y, float binSize) => (int)MathF.Floor(y / binSize);
}
