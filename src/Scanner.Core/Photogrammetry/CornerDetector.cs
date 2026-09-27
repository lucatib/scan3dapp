using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <param name="MaxCorners">Most corners returned; the strongest are kept.</param>
/// <param name="CellSize">Side, in pixels, of the square cells the photo is divided into from its top-left pixel to
/// spread its corners; a corner belongs to the cell of the pixel it rounds to.</param>
/// <param name="PerCell">Most corners kept in one cell, so that a patch of strong texture cannot take them all: a pose
/// is pinned down by points all over the photo, not by many in one place.</param>
/// <param name="Border">A corner rounds to a pixel at least this far from the outermost pixels: matching needs a
/// window around each corner inside the image.</param>
/// <param name="WindowRadius">The structure tensor is summed over (2r+1)² pixels.</param>
/// <param name="MinResponseRatio">Weakest response kept, as a fraction of the photo's strongest.</param>
public sealed record CornerOptions(int MaxCorners = 2000, int CellSize = 32, int PerCell = 6, int Border = 8,
    int WindowRadius = 2, float MinResponseRatio = 0.01f);

/// <summary>
/// Shi-Tomasi corners: the response is the smaller eigenvalue of the structure tensor (the gradient's outer product
/// summed over a window), which is large only where the gradient varies in two directions, so a straight edge,
/// however strong, scores nothing. Corners are the 3×3 local maxima, spread over the photo by a per-cell quota.
/// <para>
/// Sampled on a pixel grid, a slanted edge wobbles, and a sharp one of high contrast scores more than a faint real
/// corner, so no threshold on the response alone rejects it; an image of nothing but that edge would have its wobble
/// as the strongest response. What gives the edge away is that its weaker direction is a tiny fraction of its
/// stronger one, whatever the contrast, so that ratio is bounded too. Matched along an epipolar line, an edge would
/// match wherever the line crosses it: a wrong point that looks right.
/// </para>
/// <para>
/// The response's peak is not the corner: a window prefers to cover more of both edges, so for a right-angled corner
/// the peak sits about <see cref="CornerOptions.WindowRadius"/> pixels inside it, and how far depends on the angle,
/// which changes between views. Fitting a parabola to the response keeps that bias; instead each corner moves to the
/// point every gradient in its window is orthogonal to (Förstner's operator, as in OpenCV's cornerSubPix), which is
/// exact for an ideal corner and the centre of a blob or a saddle. Bundle adjustment needs that: a bias that differs
/// between views is reprojection error no pose can explain.
/// </para>
/// </summary>
public static class CornerDetector
{
    // Weakest response per window pixel, in (grey levels per pixel)²: about a right-angled corner of 3 grey levels'
    // contrast, which is within the noise of a phone photo. Keeps a plain, noisy photo from yielding noise as corners.
    private const float MinResponsePerPixel = 0.5f;

    // Smallest ratio of the weaker to the stronger eigenvalue. Aliased straight edges stayed below 0.01 at every angle
    // (3° to 80°), blur (1 to 3 px) and contrast tried; textured surfaces lose almost no corners up to 0.1.
    private const double MinIsotropy = 0.05;

    // Window moves allowed while refining a corner; it settles in one or two unless the point is not a corner.
    private const int MaxRefinements = 5;

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<GrayImage, CachedCorners> Cache = new();

    private sealed record CachedCorners(CornerOptions Options, List<Feature> Features);

    /// <summary>
    /// <see cref="Detect"/>, remembered for as long as <paramref name="image"/> lives: a photo can have its corners found
    /// while the scan is still running, so that finishing the scan does not wait for them. Thread-safe.
    /// </summary>
    public static List<Feature> DetectCached(GrayImage image, CornerOptions options)
    {
        if (Cache.TryGetValue(image, out var cached) && cached.Options == options) return cached.Features;
        var features = Detect(image, options);
        Cache.AddOrUpdate(image, new CachedCorners(options, features));
        return features;
    }

    /// <summary>The photo's corners, strongest first, at sub-pixel positions (x right, y down, pixel centres at
    /// integers). The same photo and options always give the same list.</summary>
    public static List<Feature> Detect(GrayImage image, CornerOptions? options = null)
    {
        options ??= new CornerOptions();
        if (options.MaxCorners < 0) throw new ArgumentOutOfRangeException(nameof(options), "MaxCorners must not be negative.");
        if (options.CellSize < 1) throw new ArgumentOutOfRangeException(nameof(options), "CellSize must be positive.");
        if (options.WindowRadius < 1) throw new ArgumentOutOfRangeException(nameof(options), "WindowRadius must be positive.");

        int w = image.Width, h = image.Height, r = options.WindowRadius;
        int border = Math.Max(options.Border, 1);
        if (w <= 2 * border || h <= 2 * border || options.MaxCorners == 0 || options.PerCell <= 0) return [];

        var (gx, gy) = Gradients(image);
        var response = ResponseMap(gx, gy, w, h, r);

        var rowMax = new float[h];
        Parallel.For(border, h - border, y =>
        {
            float max = 0;
            for (int i = y * w + border, end = y * w + w - border; i < end; i++) max = MathF.Max(max, response[i]);
            rowMax[y] = max;
        });
        float threshold = MathF.Max(options.MinResponseRatio * rowMax.Max(), MinResponsePerPixel * (2 * r + 1) * (2 * r + 1));

        // 3×3 maxima; of two equal neighbours only the first in raster order can be one.
        var rows = new List<(int Index, float Response)>?[h];
        Parallel.For(border, h - border, y =>
        {
            for (int i = y * w + border, end = y * w + w - border; i < end; i++)
            {
                float v = response[i];
                if (v < threshold) continue;
                if (v <= response[i - w - 1] || v <= response[i - w] || v <= response[i - w + 1] || v <= response[i - 1]) continue;
                if (v < response[i + 1] || v < response[i + w - 1] || v < response[i + w] || v < response[i + w + 1]) continue;
                (rows[y] ??= []).Add((i, v));
            }
        });
        var candidates = rows.Where(row => row != null).SelectMany(row => row!).ToList();
        candidates.Sort((p, q) => p.Response != q.Response ? q.Response.CompareTo(p.Response) : p.Index.CompareTo(q.Index));

        // Strongest first: each corner is refined, then kept if its cell still has room and no stronger corner
        // already settled on the same point.
        int cellColumns = (w + options.CellSize - 1) / options.CellSize;
        var perCell = new int[cellColumns * ((h + options.CellSize - 1) / options.CellSize)];
        var taken = new Dictionary<int, (float X, float Y)>();
        float low = border - 0.5f, highX = w - border - 0.5f, highY = h - border - 0.5f;
        var result = new List<Feature>();
        foreach (var (index, value) in candidates)
        {
            if (result.Count >= options.MaxCorners) break;
            if (!Refine(gx, gy, w, h, r, index % w, index / w, out float x, out float y)) continue;
            if (x < low || y < low || x >= highX || y >= highY) continue;
            int px = (int)(x + 0.5f), py = (int)(y + 0.5f);
            int cell = px / options.CellSize + py / options.CellSize * cellColumns;
            if (perCell[cell] >= options.PerCell || Taken(taken, w, px, py, x, y)) continue;
            perCell[cell]++;
            taken[py * w + px] = (x, y);
            result.Add(new Feature(x, y, value));
        }
        return result;
    }

    /// <summary>Sobel gradients in grey levels per pixel; zero on the outermost pixels, which have no neighbours.</summary>
    private static (float[] Gx, float[] Gy) Gradients(GrayImage image)
    {
        int w = image.Width, h = image.Height;
        byte[] p = image.Pixels;
        var gx = new float[w * h];
        var gy = new float[w * h];
        Parallel.For(1, h - 1, y =>
        {
            for (int i = y * w + 1, end = y * w + w - 1; i < end; i++)
            {
                int left = p[i - w - 1] + 2 * p[i - 1] + p[i + w - 1];
                int right = p[i - w + 1] + 2 * p[i + 1] + p[i + w + 1];
                int top = p[i - w - 1] + 2 * p[i - w] + p[i - w + 1];
                int bottom = p[i + w - 1] + 2 * p[i + w] + p[i + w + 1];
                gx[i] = (right - left) * 0.125f;
                gy[i] = (bottom - top) * 0.125f;
            }
        });
        return (gx, gy);
    }

    /// <summary>Smaller eigenvalue of the structure tensor summed over the window around each pixel, clipped at the
    /// image edge. Separable sums, each window summed afresh so a pixel's response depends on its window alone.</summary>
    private static float[] ResponseMap(float[] gx, float[] gy, int w, int h, int r)
    {
        int n = w * h;
        var sxx = new float[n];
        var syy = new float[n];
        var sxy = new float[n];
        Parallel.For(0, h, y =>
        {
            int row = y * w;
            for (int x = 0; x < w; x++)
            {
                float xx = 0, yy = 0, xy = 0;
                for (int k = row + Math.Max(0, x - r), end = row + Math.Min(w - 1, x + r); k <= end; k++)
                {
                    float u = gx[k], v = gy[k];
                    xx += u * u;
                    yy += v * v;
                    xy += u * v;
                }
                sxx[row + x] = xx;
                syy[row + x] = yy;
                sxy[row + x] = xy;
            }
        });

        var response = new float[n];
        Parallel.For(0, h, y =>
        {
            int first = Math.Max(0, y - r) * w, last = Math.Min(h - 1, y + r) * w;
            for (int x = 0, i = y * w; x < w; x++, i++)
            {
                double a = 0, b = 0, c = 0;
                for (int k = first + x; k <= last + x; k += w)
                {
                    a += sxx[k];
                    b += syy[k];
                    c += sxy[k];
                }
                response[i] = Response(a, b, c);
            }
        });
        return response;
    }

    // Zero for an edge-like window. det / larger rather than mean − root: exact zero for an axis-aligned edge, and no
    // cancellation for a weak corner.
    private static float Response(double a, double b, double c)
    {
        double larger = 0.5 * (a + b) + Math.Sqrt(0.25 * (a - b) * (a - b) + c * c);
        double smaller = larger > 0 ? (a * b - c * c) / larger : 0;
        return smaller > 0 && smaller >= MinIsotropy * larger ? (float)smaller : 0f;
    }

    /// <summary>
    /// The point q minimizing Σ (gᵢ · (q − pᵢ))² over the window: on an edge through the corner the gradient is normal
    /// to the edge, so q − pᵢ must run along it; in flat parts g is zero. The window is re-centred on the pixel nearest
    /// q until q lies within that pixel (with a little slack, so a corner on a pixel boundary does not oscillate).
    /// Fails when the window never settles or wanders off the window the corner was detected in, as for a curved edge,
    /// whose "corner" is its centre of curvature.
    /// </summary>
    private static bool Refine(float[] gx, float[] gy, int w, int h, int r, int px, int py, out float x, out float y)
    {
        x = y = 0;
        int cx = px, cy = py;
        for (int move = 0; move <= MaxRefinements; move++)
        {
            double a = 0, b = 0, c = 0, ex = 0, ey = 0;
            for (int dy = Math.Max(-r, -cy), dyEnd = Math.Min(r, h - 1 - cy); dy <= dyEnd; dy++)
            for (int dx = Math.Max(-r, -cx), dxEnd = Math.Min(r, w - 1 - cx); dx <= dxEnd; dx++)
            {
                int i = (cy + dy) * w + cx + dx;
                double u = gx[i], v = gy[i];
                double uu = u * u, vv = v * v, uv = u * v;
                a += uu;
                b += vv;
                c += uv;
                ex += uu * dx + uv * dy;
                ey += uv * dx + vv * dy;
            }
            double det = a * b - c * c;
            if (!(det > 1e-9 * (a + b) * (a + b))) return false;
            double qx = (b * ex - c * ey) / det, qy = (a * ey - c * ex) / det;
            if (Math.Abs(qx) <= 0.75 && Math.Abs(qy) <= 0.75)
            {
                x = (float)(cx + qx);
                y = (float)(cy + qy);
                return Math.Abs(x - px) <= r + 0.5f && Math.Abs(y - py) <= r + 0.5f;
            }
            if (Math.Abs(qx) > 2 * r + 1 || Math.Abs(qy) > 2 * r + 1) return false;
            cx += (int)Math.Round(qx);
            cy += (int)Math.Round(qy);
            if (Math.Abs(cx - px) > r + 1 || Math.Abs(cy - py) > r + 1) return false;
        }
        return false;
    }

    // Whether a stronger corner already sits less than a pixel from (x, y).
    private static bool Taken(Dictionary<int, (float X, float Y)> taken, int w, int px, int py, float x, float y)
    {
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            if (taken.TryGetValue((py + dy) * w + px + dx, out var other) &&
                (other.X - x) * (other.X - x) + (other.Y - y) * (other.Y - y) < 1f)
                return true;
        return false;
    }
}
