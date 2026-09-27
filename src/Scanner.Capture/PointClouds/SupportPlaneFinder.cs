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

    /// <summary>Fits a gently tilted table near the detected support level. Robust refitting excludes
    /// object points; a bounded noise margin accounts for the thickness of the observed surface.</summary>
    public static FittedSupportPlane? Fit(IReadOnlyList<Vector3> points)
    {
        if (Find(points) is not { } height) return null;
        var selected = points.Where(p => MathF.Abs(p.Y - height) <= .015f).ToArray();
        FittedSupportPlane plane = new(0, 0, height);
        for (int iteration = 0; iteration < 5; iteration++)
        {
            if (selected.Length < 30) return null;
            double mx = selected.Average(p => (double)p.X), my = selected.Average(p => (double)p.Y),
                mz = selected.Average(p => (double)p.Z);
            double xx = 0, zz = 0, xz = 0, xy = 0, zy = 0;
            foreach (var p in selected)
            {
                double x = p.X - mx, y = p.Y - my, z = p.Z - mz;
                xx += x*x; zz += z*z; xz += x*z; xy += x*y; zy += z*y;
            }
            double det = xx*zz - xz*xz;
            if (xx / selected.Length < .0001 || zz / selected.Length < .0001 || det <= 1e-6 * xx * zz)
                return null; // insufficient two-dimensional support
            double a = (xy*zz - zy*xz)/det, b = (zy*xx - xy*xz)/det;
            if (a*a + b*b > .13) return null; // about 20 degrees: walls are not tables
            plane = new((float)a, (float)b, (float)(my - a*mx - b*mz));
            selected = points.Where(p => MathF.Abs(p.Y - plane.HeightAt(p.X, p.Z)) <= .005f).ToArray();
        }
        if (selected.Length < points.Count / 10) return null;
        // Estimate the full noise tail below the table, where object geometry cannot inflate it.
        // The refit inliers are truncated at 5 mm and cannot measure a thicker depth band.
        // Reflect the lower tail above the plane, assuming approximately symmetric depth noise.
        var residuals = points.Select(p => plane.HeightAt(p.X, p.Z) - p.Y)
            .Where(r => r >= 0 && r <= .05f).Order().ToArray();
        if (residuals.Length == 0) return null;
        float margin = Math.Clamp(residuals[(int)((residuals.Length - 1) * .95)] + .002f, .004f, .025f);
        return plane with { Margin = margin };
    }
    private static int Bin(float y, float binSize) => (int)MathF.Floor(y / binSize);
}
