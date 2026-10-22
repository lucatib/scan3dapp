using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Turntable;

/// <summary>A mat dot found in a photo: which one it is and where its centre is, in pixels.</summary>
public readonly record struct MatDot(int Index, Vector2 Pixel);

/// <summary>
/// Finds the <see cref="TurntableMat"/> dots in a photo: dark blobs shaped like seen-at-an-angle discs, the ellipse most
/// of them lie on (the ring), their order around it, and from runs of consecutive dots which dots they are.
/// </summary>
public static class MatDetector
{
    /// <summary>A dot this much darker than its surroundings (grey levels) counts as dark.</summary>
    private const int Contrast = 25;

    private const int MinArea = 12;

    public static IReadOnlyList<MatDot> Detect(GrayImage image)
    {
        var blobs = Blobs(image);
        if (blobs.Count < TurntableMat.WindowLength) return [];
        var ring = FitRing(blobs);
        if (ring is null) return [];
        return Identify(ring.Value.Centre, ring.Value.Normalize, ring.Value.Members);
    }

    internal readonly record struct Blob(Vector2 Centre, float Area, bool Hole);

    /// <summary>Dark, compact blobs: each one a dot candidate, with whether its centre is light (a ring dot).</summary>
    internal static List<Blob> Blobs(GrayImage image)
    {
        int w = image.Width, h = image.Height;
        var px = image.Pixels;
        // Integral image for the local mean over a window a fortieth of the image wide.
        var sum = new long[(w + 1) * (h + 1)];
        for (int y = 0; y < h; y++)
        {
            long row = 0;
            for (int x = 0; x < w; x++)
            {
                row += px[y * w + x];
                sum[(y + 1) * (w + 1) + x + 1] = sum[y * (w + 1) + x + 1] + row;
            }
        }
        int r = Math.Max(4, Math.Max(w, h) / 40);
        var dark = new bool[w * h];
        for (int y = 0; y < h; y++)
        {
            int y0 = Math.Max(0, y - r), y1 = Math.Min(h, y + r + 1);
            for (int x = 0; x < w; x++)
            {
                int x0 = Math.Max(0, x - r), x1 = Math.Min(w, x + r + 1);
                long s = sum[y1 * (w + 1) + x1] - sum[y0 * (w + 1) + x1] - sum[y1 * (w + 1) + x0] + sum[y0 * (w + 1) + x0];
                dark[y * w + x] = px[y * w + x] * (long)(x1 - x0) * (y1 - y0) < (s - Contrast * (long)(x1 - x0) * (y1 - y0));
            }
        }

        var blobs = new List<Blob>();
        var label = new int[w * h];
        var stack = new Stack<int>();
        int maxArea = w * h / 300;
        int next = 0;
        for (int start = 0; start < w * h; start++)
        {
            if (!dark[start] || label[start] != 0) continue;
            next++;
            double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
            bool border = false, tooBig = false;
            label[start] = next;
            stack.Push(start);
            while (stack.Count > 0)
            {
                int i = stack.Pop();
                int x = i % w, y = i / w;
                if (!tooBig)
                {
                    n++; sx += x; sy += y; sxx += (double)x * x; sxy += (double)x * y; syy += (double)y * y;
                    if (n > maxArea) tooBig = true;
                }
                if (x == 0 || y == 0 || x == w - 1 || y == h - 1) { border = true; continue; }
                foreach (int j in (ReadOnlySpan<int>)[i - 1, i + 1, i - w, i + w])
                    if (dark[j] && label[j] == 0) { label[j] = next; stack.Push(j); }
            }
            if (border || tooBig || n < MinArea) continue;
            double mx = sx / n, my = sy / n;
            double cxx = sxx / n - mx * mx + 1.0 / 12, cyy = syy / n - my * my + 1.0 / 12, cxy = sxy / n - mx * my;
            double det = cxx * cyy - cxy * cxy;
            if (det <= 0) continue;
            // A filled ellipse covers 4π·sqrt(det) of its second moments; a ring dot, with half its radius as hole, 0.6 of that.
            double fill = n / (4 * Math.PI * Math.Sqrt(det));
            if (fill < 0.45 || fill > 1.2) continue;
            double trace = cxx + cyy, root = Math.Sqrt(Math.Max(0, trace * trace / 4 - det));
            if ((trace / 2 + root) / (trace / 2 - root) > 64) continue; // longer than 8:1: a line, not a dot
            int cx = (int)Math.Round(mx), cy = (int)Math.Round(my);
            bool hole = label[cy * w + cx] != next;
            blobs.Add(new Blob(new Vector2((float)mx, (float)my), (float)n, hole));
        }
        return blobs;
    }

    /// <summary>
    /// The ellipse through the most blobs (RANSAC over five-point conics), refined over its members; with the map that
    /// turns it into a unit circle, so that the members can be ordered by angle.
    /// </summary>
    internal static (Vector2 Centre, Matrix3x2 Normalize, List<Blob> Members)? FitRing(List<Blob> blobs)
    {
        var mean = blobs.Aggregate(Vector2.Zero, (s, b) => s + b.Centre) / blobs.Count;
        float scale = blobs.Average(b => Vector2.Distance(b.Centre, mean));
        var points = blobs.Select(b => (b.Centre - mean) / scale).ToArray();
        var rng = new Random(7);
        double[]? best = null;
        int bestCount = 0;
        const float tolerance = 0.02f; // of the blobs' spread: about a dot diameter
        Span<int> pick = stackalloc int[5];
        for (int iteration = 0; iteration < 1500; iteration++)
        {
            for (int k = 0; k < 5; k++) pick[k] = rng.Next(points.Length);
            var conic = Conic([points[pick[0]], points[pick[1]], points[pick[2]], points[pick[3]], points[pick[4]]]);
            if (conic is null) continue;
            int count = points.Count(p => ConicDistance(conic, p) < tolerance);
            if (count > bestCount) { bestCount = count; best = conic; }
        }
        if (best is null || bestCount < TurntableMat.WindowLength) return null;
        var members = Enumerable.Range(0, points.Length).Where(i => ConicDistance(best, points[i]) < tolerance).ToList();
        var refined = Conic(members.Select(i => points[i]).ToArray()) ?? best;
        members = Enumerable.Range(0, points.Length).Where(i => ConicDistance(refined, points[i]) < tolerance).ToList();
        if (members.Count < TurntableMat.WindowLength) return null;

        // A x² + B xy + C y² + D x + E y + F = 0 with F = -1: centre and the map to a unit circle.
        double a = refined[0], b = refined[1], c = refined[2], d = refined[3], e = refined[4];
        double det = a * c - b * b / 4;
        double x0 = (-d * c / 2 + e * b / 4) / det, y0 = (-e * a / 2 + d * b / 4) / det;
        double k0 = 1 - (a * x0 * x0 + b * x0 * y0 + c * y0 * y0 + d * x0 + e * y0); // value of -F at the centre, shifted
        // (p - c)ᵀ M (p - c) = k0 with M = [[a, b/2], [b/2, c]]: the circle map is M^½ / sqrt(k0).
        double tr = a + c, s = Math.Sqrt(det), t = Math.Sqrt(tr + 2 * s);
        double m11 = (a + s) / t, m12 = b / 2 / t, m22 = (c + s) / t;
        double norm = 1 / Math.Sqrt(k0);
        var normalize = new Matrix3x2((float)(m11 * norm), (float)(m12 * norm), (float)(m12 * norm), (float)(m22 * norm), 0, 0);
        var centre = new Vector2((float)x0, (float)y0) * scale + mean;
        // Back to pixels: the normalizing map works on the scaled coordinates.
        var normalizePixels = Matrix3x2.CreateTranslation(-centre) * Matrix3x2.CreateScale(1 / scale) * normalize;
        return (centre, normalizePixels, members.Select(i => blobs[i]).ToList());
    }

    /// <summary>Orders the ring's blobs by angle, splits them where dots are missing, and names each run by its code.</summary>
    internal static List<MatDot> Identify(Vector2 centre, Matrix3x2 normalize, List<Blob> members)
    {
        var ordered = members
            .Select(b => (Blob: b, Angle: Angle(Vector2.Transform(b.Centre, normalize))))
            .OrderBy(m => m.Angle).ToList();
        int n = ordered.Count;
        var gaps = new float[n];
        for (int i = 0; i < n; i++)
        {
            float gap = ordered[(i + 1) % n].Angle - ordered[i].Angle;
            gaps[i] = gap <= 0 ? gap + 2 * MathF.PI : gap;
        }
        // A gap much wider than both its neighbours is missing dots (hidden by the piece, or out of the picture).
        var breaks = new bool[n];
        float nominal = 2 * MathF.PI / TurntableMat.DotCount;
        for (int i = 0; i < n; i++)
        {
            float neighbours = MathF.Min(gaps[(i + n - 1) % n], gaps[(i + 1) % n]);
            breaks[i] = gaps[i] > 1.6f * neighbours || gaps[i] > 2.2f * nominal;
        }
        int first = Array.IndexOf(breaks, true);
        if (first < 0 && n != TurntableMat.DotCount) return [];

        var result = new List<MatDot>();
        int startAt = first < 0 ? 0 : (first + 1) % n;
        var run = new List<(Blob Blob, float Angle)>();
        for (int k = 0; k < n; k++)
        {
            int i = (startAt + k) % n;
            run.Add(ordered[i]);
            if (breaks[i] || k == n - 1)
            {
                NameRun(run, result);
                run.Clear();
            }
        }
        return result;
    }

    /// <summary>Every 7-dot window of the run votes for where the run starts on the ring; the dots that agree with the
    /// winning place are named.</summary>
    private static void NameRun(List<(Blob Blob, float Angle)> run, List<MatDot> result)
    {
        int w = TurntableMat.WindowLength;
        if (run.Count < w) return;
        var votes = new Dictionary<(int Start, int Direction), int>();
        var codes = run.Select(r => r.Blob.Hole).ToArray();
        for (int offset = 0; offset + w <= run.Count; offset++)
        {
            if (TurntableMat.Locate(codes[offset..(offset + w)]) is not { } place) continue;
            var key = (Mod(place.Start - place.Direction * offset), place.Direction);
            votes[key] = votes.GetValueOrDefault(key) + 1;
        }
        if (votes.Count == 0) return;
        var ((start, direction), count) = votes.MaxBy(v => v.Value);
        if (count * 2 < run.Count - w + 1) return; // most windows must agree
        for (int k = 0; k < run.Count; k++)
        {
            int index = Mod(start + direction * k);
            if (TurntableMat.Code[index] == run[k].Blob.Hole) result.Add(new MatDot(index, run[k].Blob.Centre));
        }
    }

    /// <summary>The conic A x² + B xy + C y² + D x + E y = 1 through the points (least squares beyond five), if an ellipse.</summary>
    private static double[]? Conic(Vector2[] points)
    {
        var normal = new double[5, 5];
        var rhs = new double[5];
        foreach (var p in points)
        {
            Span<double> row = [p.X * p.X, p.X * p.Y, p.Y * p.Y, p.X, p.Y];
            for (int i = 0; i < 5; i++)
            {
                rhs[i] += row[i];
                for (int j = 0; j < 5; j++) normal[i, j] += row[i] * row[j];
            }
        }
        if (!Solve(normal, rhs, out var x)) return null;
        return x[1] * x[1] - 4 * x[0] * x[2] < 0 ? x : null;
    }

    /// <summary>Approximate distance from a point to the conic: its value over its gradient's length.</summary>
    private static float ConicDistance(double[] q, Vector2 p)
    {
        double x = p.X, y = p.Y;
        double value = q[0] * x * x + q[1] * x * y + q[2] * y * y + q[3] * x + q[4] * y - 1;
        double gx = 2 * q[0] * x + q[1] * y + q[3], gy = q[1] * x + 2 * q[2] * y + q[4];
        return (float)(Math.Abs(value) / Math.Max(1e-9, Math.Sqrt(gx * gx + gy * gy)));
    }

    /// <summary>Gaussian elimination with partial pivoting (5×5).</summary>
    private static bool Solve(double[,] a, double[] b, out double[] x)
    {
        int n = b.Length;
        var m = (double[,])a.Clone();
        x = (double[])b.Clone();
        for (int col = 0; col < n; col++)
        {
            int pivot = col;
            for (int row = col + 1; row < n; row++) if (Math.Abs(m[row, col]) > Math.Abs(m[pivot, col])) pivot = row;
            if (Math.Abs(m[pivot, col]) < 1e-12) return false;
            if (pivot != col)
            {
                for (int k = 0; k < n; k++) (m[col, k], m[pivot, k]) = (m[pivot, k], m[col, k]);
                (x[col], x[pivot]) = (x[pivot], x[col]);
            }
            for (int row = col + 1; row < n; row++)
            {
                double f = m[row, col] / m[col, col];
                for (int k = col; k < n; k++) m[row, k] -= f * m[col, k];
                x[row] -= f * x[col];
            }
        }
        for (int row = n - 1; row >= 0; row--)
        {
            for (int k = row + 1; k < n; k++) x[row] -= m[row, k] * x[k];
            x[row] /= m[row, row];
        }
        return true;
    }

    private static float Angle(Vector2 v) => MathF.Atan2(v.Y, v.X);

    private static int Mod(int i) => ((i % TurntableMat.DotCount) + TurntableMat.DotCount) % TurntableMat.DotCount;
}
