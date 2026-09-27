using System.Numerics;
using System.Runtime.InteropServices;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <param name="EpipolarTolerancePx">How far, in pixels of the photo searched, a candidate may lie from the predicted
/// epipolar segment. It absorbs pose error: a few pixels for exact poses, 20-40 px at 1080p for ARCore's.</param>
/// <param name="PatchRadius">Patches of (2r+1)² pixels are compared.</param>
/// <param name="MinNcc">Weakest correlation accepted as a match.</param>
/// <param name="MinMargin">How far the best candidate must beat the second: when two candidates correlate about
/// equally (repeated texture, two corners of one blob) the winner is a coin toss, and a wrong match costs more in
/// bundle adjustment than a missing one.</param>
/// <param name="MinDepth">Nearest a surface can be to either camera, in metres.</param>
/// <param name="MaxDepth">Farthest a surface can be from the camera whose feature is searched for, in metres.</param>
public sealed record MatchOptions(float EpipolarTolerancePx = 8f, int PatchRadius = 5, float MinNcc = 0.8f,
    float MinMargin = 0.05f, float MinDepth = 0.05f, float MaxDepth = 3f);

/// <summary>
/// Matches the features of two photos whose poses are roughly known. A feature's 3D point lies on its pixel ray, so
/// its counterpart lies near the projection of that ray into the other photo (the epipolar segment, between the
/// nearest and farthest depths considered): only features near the segment are candidates, found through a grid,
/// and compared by zero-mean normalized cross-correlation of patches around them. A match must beat every other
/// candidate clearly and be mutual: searching back from it must land on the feature it came from. Poses only select
/// candidates; the patches decide, so pose error costs nothing as long as the tolerance covers it.
/// </summary>
public static class GuidedMatcher
{
    // Patches with a standard deviation under half a gray level are flat: their correlation is rounding noise.
    private const float MinPatchVariance = 0.25f;

    /// <summary>One-to-one matches between the features of photos <paramref name="a"/> and <paramref name="b"/>, in
    /// increasing order of A's feature index, scored by their correlation. Features whose patch does not lie wholly
    /// inside their photo, or is flat, are never matched.</summary>
    public static List<FeatureMatch> Match(int viewA, PhotoView a, IReadOnlyList<Feature> featuresA, int viewB, PhotoView b,
        IReadOnlyList<Feature> featuresB, MatchOptions? options = null)
    {
        options ??= new MatchOptions();
        Validate(options);
        float tolerance = options.EpipolarTolerancePx;
        var patchesA = new Patches(a.Image, featuresA, options.PatchRadius);
        var patchesB = new Patches(b.Image, featuresB, options.PatchRadius);
        var gridA = new FeatureGrid(featuresA, patchesA.Valid, a.Image.Width, a.Image.Height, tolerance);
        var gridB = new FeatureGrid(featuresB, patchesB.Valid, b.Image.Width, b.Image.Height, tolerance);
        var aToB = new Epipolar(a, b, options);
        var bToA = new Epipolar(b, a, options);

        var bestB = new int[featuresA.Count];
        var scores = new float[featuresA.Count];
        Parallel.For(0, featuresA.Count, i =>
        {
            bestB[i] = -1;
            if (!patchesA.Valid[i]) return;
            var (best, score, second) = Search(aToB, featuresA[i], patchesA.Patch(i), gridB, patchesB, tolerance);
            if (best < 0 || score < options.MinNcc || score - second < options.MinMargin) return;
            bestB[i] = best;
            scores[i] = score;
        });

        // The way back is needed only from the features of B that some feature of A chose.
        var chosen = new bool[featuresB.Count];
        foreach (int j in bestB)
            if (j >= 0) chosen[j] = true;
        var bestA = new int[featuresB.Count];
        Parallel.For(0, featuresB.Count, j =>
        {
            bestA[j] = chosen[j] ? Search(bToA, featuresB[j], patchesB.Patch(j), gridA, patchesA, tolerance).Best : -1;
        });

        var matches = new List<FeatureMatch>();
        for (int i = 0; i < featuresA.Count; i++)
            if (bestB[i] >= 0 && bestA[bestB[i]] == i) matches.Add(new FeatureMatch(viewA, i, viewB, bestB[i], scores[i]));
        return matches;
    }

    private static void Validate(MatchOptions options)
    {
        if (!(options.EpipolarTolerancePx > 0) || float.IsInfinity(options.EpipolarTolerancePx))
            throw new ArgumentOutOfRangeException(nameof(options), "The epipolar tolerance must be a positive number of pixels.");
        if (options.PatchRadius < 1)
            throw new ArgumentOutOfRangeException(nameof(options), "The patch radius must be at least one pixel.");
        if (!(options.MinDepth > 0) || !(options.MaxDepth > options.MinDepth))
            throw new ArgumentOutOfRangeException(nameof(options), "The depth range must be positive and not empty.");
    }

    /// <summary>The candidate of <paramref name="other"/> near the feature's epipolar segment that correlates best
    /// with it (-1 when none), its score, and the runner-up's score (-∞ when there is no runner-up).</summary>
    private static (int Best, float Score, float Second) Search(in Epipolar geometry, Feature feature, ReadOnlySpan<float> patch,
        FeatureGrid grid, Patches other, float tolerance)
    {
        int best = -1;
        float bestScore = float.NegativeInfinity, second = float.NegativeInfinity;
        if (!geometry.Segment(feature.X, feature.Y, out var p0, out var p1)) return (best, bestScore, second);

        var direction = p1 - p0;
        float lengthSquared = direction.LengthSquared(), toleranceSquared = tolerance * tolerance, cell = grid.CellSize;
        int firstRow = Math.Max(0, (int)MathF.Floor((MathF.Min(p0.Y, p1.Y) - tolerance) / cell));
        int lastRow = Math.Min(grid.Rows - 1, (int)MathF.Floor((MathF.Max(p0.Y, p1.Y) + tolerance) / cell));
        for (int row = firstRow; row <= lastRow; row++)
        {
            // A feature in this row of cells near the segment is near the part of it that passes within the tolerance
            // of the row: only the columns under that part, widened by the tolerance, can hold one.
            float t0 = 0, t1 = 1;
            float low = row * cell - tolerance, high = (row + 1) * cell + tolerance;
            if (direction.Y != 0)
            {
                float ta = (low - p0.Y) / direction.Y, tb = (high - p0.Y) / direction.Y;
                t0 = MathF.Max(0, MathF.Min(ta, tb));
                t1 = MathF.Min(1, MathF.Max(ta, tb));
                if (t0 > t1) continue;
            }
            else if (p0.Y < low || p0.Y > high) continue;
            float xa = p0.X + t0 * direction.X, xb = p0.X + t1 * direction.X;
            int firstColumn = Math.Max(0, (int)MathF.Floor((MathF.Min(xa, xb) - tolerance) / cell));
            int lastColumn = Math.Min(grid.Columns - 1, (int)MathF.Floor((MathF.Max(xa, xb) + tolerance) / cell));

            for (int column = firstColumn; column <= lastColumn; column++)
            {
                int c = row * grid.Columns + column;
                for (int k = grid.Start[c], end = grid.Start[c + 1]; k < end; k++)
                {
                    int j = grid.Items[k];
                    var q = grid.Positions[k] - p0;
                    float t = lengthSquared > 0 ? Math.Clamp(Vector2.Dot(q, direction) / lengthSquared, 0f, 1f) : 0f;
                    if (Vector2.DistanceSquared(q, t * direction) > toleranceSquared) continue;
                    float score = Dot(patch, other.Patch(j));
                    if (score > bestScore)
                    {
                        second = bestScore;
                        bestScore = score;
                        best = j;
                    }
                    else if (score > second) second = score;
                }
            }
        }
        return (best, bestScore, second);
    }

    // Patches are zero-mean and unit-length, padded with zeros to whole SIMD vectors, so their correlation is a dot product.
    private static float Dot(ReadOnlySpan<float> x, ReadOnlySpan<float> y)
    {
        var vx = MemoryMarshal.Cast<float, Vector<float>>(x);
        var vy = MemoryMarshal.Cast<float, Vector<float>>(y);
        var sum = Vector<float>.Zero;
        for (int i = 0; i < vx.Length; i++) sum += vx[i] * vy[i];
        return Vector.Sum(sum);
    }

    /// <summary>Projects pixel rays of one photo ("from") into another ("to").</summary>
    private readonly struct Epipolar
    {
        private readonly CameraIntrinsics from, to;
        private readonly Vector3 rowX, rowY, rowZ, origin;
        private readonly float minDepth, maxDepth, margin, right, bottom;

        public Epipolar(PhotoView from, PhotoView to, MatchOptions options)
        {
            this.from = from.Intrinsics;
            this.to = to.Intrinsics;
            // Rotation from the "from" camera frame to the "to" camera frame: entry (i, j) is the dot product of
            // to's i-th axis with from's j-th axis, both in world coordinates (the rows of a camera→world pose).
            var f = from.CameraToWorld;
            var t = to.CameraToWorld;
            Vector3 fx = new(f.M11, f.M12, f.M13), fy = new(f.M21, f.M22, f.M23), fz = new(f.M31, f.M32, f.M33);
            Vector3 tx = new(t.M11, t.M12, t.M13), ty = new(t.M21, t.M22, t.M23), tz = new(t.M31, t.M32, t.M33);
            rowX = new Vector3(Vector3.Dot(tx, fx), Vector3.Dot(tx, fy), Vector3.Dot(tx, fz));
            rowY = new Vector3(Vector3.Dot(ty, fx), Vector3.Dot(ty, fy), Vector3.Dot(ty, fz));
            rowZ = new Vector3(Vector3.Dot(tz, fx), Vector3.Dot(tz, fy), Vector3.Dot(tz, fz));
            origin = Pinhole.ToCamera(t, f.Translation);
            minDepth = options.MinDepth;
            maxDepth = options.MaxDepth;
            // Features lie inside to's image, so only the segment within the tolerance of it can reach one.
            margin = options.EpipolarTolerancePx;
            right = to.Image.Width - 1 + margin;
            bottom = to.Image.Height - 1 + margin;
        }

        /// <summary>The pixel's ray between the depth limits, in "to"'s pixels: only the part at least the minimum depth
        /// in front of "to", and only the part near its image. False when nothing is left.</summary>
        public bool Segment(float x, float y, out Vector2 p0, out Vector2 p1)
        {
            p0 = p1 = default;
            var ray = new Vector3((x - from.Cx) / from.Fx, (y - from.Cy) / from.Fy, 1f);
            // At depth z along from's axis the point is origin + z·m in to's frame; its depth there is linear in z.
            var m = new Vector3(Vector3.Dot(rowX, ray), Vector3.Dot(rowY, ray), Vector3.Dot(rowZ, ray));
            float z0 = minDepth, z1 = maxDepth;
            if (m.Z > 0) z0 = MathF.Max(z0, (minDepth - origin.Z) / m.Z);
            else if (m.Z < 0) z1 = MathF.Min(z1, (minDepth - origin.Z) / m.Z);
            else if (origin.Z < minDepth) return false;
            if (!(z0 <= z1)) return false;

            p0 = Project(origin + z0 * m);
            p1 = Project(origin + z1 * m);
            return Clip(ref p0, ref p1, -margin, -margin, right, bottom);
        }

        private Vector2 Project(Vector3 p) => new(to.Fx * p.X / p.Z + to.Cx, to.Fy * p.Y / p.Z + to.Cy);

        // Liang-Barsky: keeps the part of the segment inside the rectangle, which also bounds the grid walk when a
        // near depth projects thousands of pixels away.
        private static bool Clip(ref Vector2 p0, ref Vector2 p1, float left, float top, float right, float bottom)
        {
            var d = p1 - p0;
            float t0 = 0, t1 = 1;
            static bool Edge(float p, float q, ref float t0, ref float t1)
            {
                if (p == 0) return q >= 0;
                float r = q / p;
                if (p < 0) { if (r > t1) return false; if (r > t0) t0 = r; }
                else { if (r < t0) return false; if (r < t1) t1 = r; }
                return true;
            }
            if (!Edge(-d.X, p0.X - left, ref t0, ref t1) || !Edge(d.X, right - p0.X, ref t0, ref t1)
                || !Edge(-d.Y, p0.Y - top, ref t0, ref t1) || !Edge(d.Y, bottom - p0.Y, ref t0, ref t1))
                return false;
            var start = p0;
            p0 = start + t0 * d;
            p1 = start + t1 * d;
            return true;
        }
    }

    /// <summary>Zero-mean, unit-length patches around features, sampled bilinearly at their sub-pixel positions.</summary>
    private sealed class Patches
    {
        private readonly float[] data;
        private readonly int stride;

        public Patches(GrayImage image, IReadOnlyList<Feature> features, int radius)
        {
            int side = 2 * radius + 1, area = side * side, w = image.Width, h = image.Height;
            stride = (area + Vector<float>.Count - 1) / Vector<float>.Count * Vector<float>.Count;
            data = new float[features.Count * stride];
            Valid = new bool[features.Count];
            byte[] pixels = image.Pixels;
            for (int i = 0; i < features.Count; i++)
            {
                float left = features[i].X - radius, top = features[i].Y - radius;
                // Written so that NaN coordinates fail too.
                if (!(left >= 0 && top >= 0 && left + 2 * radius <= w - 1 && top + 2 * radius <= h - 1)) continue;
                // Bilinear weights are the same for every pixel of the patch. A patch ending exactly on the last
                // column or row samples it with full weight from one to the left, so it never reads past the image.
                int x0 = Math.Min((int)left, w - 1 - side), y0 = Math.Min((int)top, h - 1 - side);
                if (x0 < 0 || y0 < 0) continue;
                float ax = left - x0, ay = top - y0;
                float w00 = (1 - ax) * (1 - ay), w10 = ax * (1 - ay), w01 = (1 - ax) * ay, w11 = ax * ay;

                var patch = data.AsSpan(i * stride, area);
                float sum = 0;
                for (int y = 0, k = 0; y < side; y++)
                {
                    int o = (y0 + y) * w + x0;
                    for (int x = 0; x < side; x++, k++, o++)
                    {
                        float value = w00 * pixels[o] + w10 * pixels[o + 1] + w01 * pixels[o + w] + w11 * pixels[o + w + 1];
                        patch[k] = value;
                        sum += value;
                    }
                }
                float mean = sum / area, squares = 0;
                for (int k = 0; k < area; k++)
                {
                    patch[k] -= mean;
                    squares += patch[k] * patch[k];
                }
                if (squares < MinPatchVariance * area) continue;
                float scale = 1f / MathF.Sqrt(squares);
                for (int k = 0; k < area; k++) patch[k] *= scale;
                Valid[i] = true;
            }
        }

        public bool[] Valid { get; }

        public ReadOnlySpan<float> Patch(int i) => data.AsSpan(i * stride, stride);
    }

    /// <summary>Features with a valid patch, bucketed into square cells about the tolerance wide, so a search visits
    /// only the cells along its segment. Cells hold their features in index order, which keeps ties deterministic.</summary>
    private sealed class FeatureGrid
    {
        public FeatureGrid(IReadOnlyList<Feature> features, bool[] valid, int width, int height, float tolerance)
        {
            CellSize = MathF.Max(tolerance, 4f);
            Columns = Math.Max(1, (int)MathF.Ceiling(width / CellSize));
            Rows = Math.Max(1, (int)MathF.Ceiling(height / CellSize));
            var cellOf = new int[features.Count];
            Start = new int[Rows * Columns + 1];
            for (int i = 0; i < features.Count; i++)
            {
                if (!valid[i]) { cellOf[i] = -1; continue; }
                int column = Math.Clamp((int)(features[i].X / CellSize), 0, Columns - 1);
                int row = Math.Clamp((int)(features[i].Y / CellSize), 0, Rows - 1);
                cellOf[i] = row * Columns + column;
                Start[cellOf[i] + 1]++;
            }
            for (int c = 0; c < Rows * Columns; c++) Start[c + 1] += Start[c];
            Items = new int[Start[^1]];
            Positions = new Vector2[Start[^1]];
            var next = Start[..^1];
            for (int i = 0; i < features.Count; i++)
            {
                if (cellOf[i] < 0) continue;
                int k = next[cellOf[i]]++;
                Items[k] = i;
                Positions[k] = new Vector2(features[i].X, features[i].Y);
            }
        }

        public float CellSize { get; }
        public int Columns { get; }
        public int Rows { get; }
        public int[] Start { get; }
        public int[] Items { get; }
        public Vector2[] Positions { get; }
    }
}
