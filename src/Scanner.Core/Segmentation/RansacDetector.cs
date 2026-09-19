using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

/// <summary>
/// Sequential RANSAC: each round picks the primitive (plane or cylinder) with the most inliers among the
/// remaining points, refines it via least squares, and removes its inliers.
/// </summary>
public static class RansacDetector
{
    /// <summary>Bins of the 360° sweep around a cylinder axis used by the angular-coverage gate (5° each).</summary>
    private const int CoverageBins = 72;

    /// <summary>A cylinder candidate must occupy a contiguous arc of at least this many bins (90°).</summary>
    private const int MinCoverageBins = 18;

    public static IReadOnlyList<DetectedShape> Detect(PointCloud cloud, RansacOptions options)
    {
        var rng = new Random(options.Seed);
        float cosThreshold = MathF.Cos(options.NormalThresholdDegrees * MathF.PI / 180f);
        var remaining = Enumerable.Range(0, cloud.Count).ToList();
        var shapes = new List<DetectedShape>();

        while (remaining.Count >= options.MinInliers && shapes.Count < options.MaxShapes)
        {
            Primitive? best = null;
            int bestScore = 0;
            for (int iteration = 0; iteration < options.IterationsPerShape; iteration++)
            {
                Primitive? candidate = iteration % 2 == 0
                    ? PlaneFromSample(cloud, remaining, rng)
                    : CylinderFromSample(cloud, remaining, rng, options);
                if (candidate is null) continue;
                int score = Score(cloud, remaining, candidate, options.DistanceThreshold, cosThreshold);
                if (score > bestScore)
                {
                    bestScore = score;
                    best = candidate;
                }
            }
            if (best is null || bestScore < options.MinInliers) break;

            var inliers = CollectInliers(cloud, remaining, best, options.DistanceThreshold, cosThreshold);
            for (int pass = 0; pass < 2; pass++)
            {
                var refined = PrimitiveFitter.Refine(best, cloud, inliers);
                if (refined is null) break;
                // Refinement moves the surface, so it can shed more inliers than it gains. Keep it only when it
                // does not lose any, otherwise stop refining and keep the better shape found so far.
                var refinedInliers = CollectInliers(cloud, remaining, refined, options.DistanceThreshold, cosThreshold);
                if (refinedInliers.Count < inliers.Count) break;
                best = refined;
                inliers = refinedInliers;
            }
            if (inliers.Count < options.MinInliers) break;
            // Refinement can turn an acceptable cylinder into one whose inliers no longer sweep a real arc.
            if (!HasAngularCoverage(best, cloud, inliers)) break;

            shapes.Add(new DetectedShape(best, inliers.ToArray()));
            var taken = inliers.ToHashSet();
            remaining.RemoveAll(taken.Contains);
        }
        return shapes;
    }

    public static bool IsInlier(Primitive shape, Vector3 p, Vector3 n, float distance, float cosThreshold) => shape switch
    {
        PlanePrimitive plane => MathF.Abs(plane.SignedDistance(p)) <= distance && Vector3.Dot(plane.Normal, n) >= cosThreshold,
        CylinderPrimitive cylinder => IsCylinderInlier(cylinder, p, n, distance, cosThreshold),
        _ => false,
    };

    private static bool IsCylinderInlier(CylinderPrimitive cylinder, Vector3 p, Vector3 n, float distance, float cosThreshold)
    {
        var radial = cylinder.RadialVector(p);
        float length = radial.Length();
        if (length < 1e-9f || MathF.Abs(length - cylinder.Radius) > distance) return false;
        float alignment = Vector3.Dot(n, radial / length);
        return cylinder.IsHole ? alignment <= -cosThreshold : alignment >= cosThreshold;
    }

    /// <summary>
    /// Inlier count of a candidate, or 0 when a cylinder candidate fails the angular-coverage gate. A flat face
    /// is tangent to any cylinder of comparable radius, so a candidate hugging the faces of a prism collects
    /// large numbers of inliers in a few narrow sectors; requiring one contiguous arc rejects it regardless of
    /// face count, object size and noise, while a real cylinder sweeps the full circle.
    /// </summary>
    private static int Score(PointCloud cloud, List<int> remaining, Primitive shape, float distance, float cosThreshold)
    {
        if (shape is not CylinderPrimitive cylinder)
            return CountInliers(cloud, remaining, shape, distance, cosThreshold);

        var occupied = new bool[CoverageBins];
        var (u, v) = Basis.Orthonormal(cylinder.Axis);
        int count = 0;
        foreach (int i in remaining)
        {
            if (!IsCylinderInlier(cylinder, cloud.Points[i], cloud.Normals[i], distance, cosThreshold)) continue;
            count++;
            MarkBin(occupied, cylinder, u, v, cloud.Points[i]);
        }
        return LongestOccupiedRun(occupied) >= MinCoverageBins ? count : 0;
    }

    /// <summary>True for any non-cylinder; for a cylinder, true when its inliers sweep a long enough contiguous arc.</summary>
    private static bool HasAngularCoverage(Primitive shape, PointCloud cloud, IReadOnlyList<int> indices)
    {
        if (shape is not CylinderPrimitive cylinder) return true;

        var occupied = new bool[CoverageBins];
        var (u, v) = Basis.Orthonormal(cylinder.Axis);
        foreach (int i in indices) MarkBin(occupied, cylinder, u, v, cloud.Points[i]);
        return LongestOccupiedRun(occupied) >= MinCoverageBins;
    }

    private static void MarkBin(bool[] occupied, CylinderPrimitive cylinder, Vector3 u, Vector3 v, Vector3 p)
    {
        var radial = cylinder.RadialVector(p);
        float angle = MathF.Atan2(Vector3.Dot(radial, v), Vector3.Dot(radial, u));
        int bin = (int)MathF.Floor((angle + MathF.PI) / (2 * MathF.PI) * occupied.Length);
        occupied[Math.Clamp(bin, 0, occupied.Length - 1)] = true;
    }

    /// <summary>
    /// Longest run of consecutive occupied bins, treating the array as circular. A single empty bin is bridged
    /// (and counted, since the arc is continuous in angle) so that sparse sampling does not split a genuine arc;
    /// two or more consecutive empty bins break the run.
    /// </summary>
    private static int LongestOccupiedRun(bool[] occupied)
    {
        int total = occupied.Length;
        int best = 0, run = 0, gap = 0;
        for (int k = 0; k < 2 * total; k++)
        {
            if (occupied[k % total])
            {
                run += gap + 1;
                gap = 0;
                if (run > best) best = run;
            }
            else if (gap == 0 && run > 0)
            {
                gap = 1;
            }
            else
            {
                run = 0;
                gap = 0;
            }
        }
        return Math.Min(best, total);
    }

    private static int CountInliers(PointCloud cloud, List<int> remaining, Primitive shape, float distance, float cosThreshold)
    {
        int count = 0;
        foreach (int i in remaining)
            if (IsInlier(shape, cloud.Points[i], cloud.Normals[i], distance, cosThreshold)) count++;
        return count;
    }

    private static List<int> CollectInliers(PointCloud cloud, List<int> remaining, Primitive shape, float distance, float cosThreshold) =>
        remaining.Where(i => IsInlier(shape, cloud.Points[i], cloud.Normals[i], distance, cosThreshold)).ToList();

    private static PlanePrimitive PlaneFromSample(PointCloud cloud, List<int> remaining, Random rng)
    {
        int i = remaining[rng.Next(remaining.Count)];
        var n = cloud.Normals[i];
        return new PlanePrimitive(n, Vector3.Dot(n, cloud.Points[i]));
    }

    // Two points with normals: axis = n1 × n2; the cross-section center is the intersection of the projected normals.
    private static CylinderPrimitive? CylinderFromSample(PointCloud cloud, List<int> remaining, Random rng, RansacOptions options)
    {
        int i = remaining[rng.Next(remaining.Count)];
        int j = remaining[rng.Next(remaining.Count)];
        if (i == j) return null;
        Vector3 p1 = cloud.Points[i], p2 = cloud.Points[j], n1 = cloud.Normals[i], n2 = cloud.Normals[j];

        var cross = Vector3.Cross(n1, n2);
        if (cross.Length() < 0.2f) return null;
        var axis = Vector3.Normalize(cross);
        var (u, v) = Basis.Orthonormal(axis);

        var q1 = new Vector2(Vector3.Dot(p1, u), Vector3.Dot(p1, v));
        var q2 = new Vector2(Vector3.Dot(p2, u), Vector3.Dot(p2, v));
        var m1 = Vector2.Normalize(new Vector2(Vector3.Dot(n1, u), Vector3.Dot(n1, v)));
        var m2 = Vector2.Normalize(new Vector2(Vector3.Dot(n2, u), Vector3.Dot(n2, v)));

        // q1 + t1·m1 = q2 + t2·m2
        float det = -m1.X * m2.Y + m2.X * m1.Y;
        if (MathF.Abs(det) < 1e-6f) return null;
        var r = q2 - q1;
        float t1 = (-r.X * m2.Y + m2.X * r.Y) / det;
        var center = q1 + t1 * m1;
        float radius = (Vector2.Distance(q1, center) + Vector2.Distance(q2, center)) / 2;
        if (radius < options.DistanceThreshold || radius > options.MaxCylinderRadius) return null;

        var cylinder = new CylinderPrimitive(u * center.X + v * center.Y, axis, radius, IsHole: false);
        // Both sampled normals must agree on the polarity: deciding it from one of them lets a single noisy
        // normal invert the candidate, and IsCylinderInlier would then reject every genuine inlier.
        bool hole1 = Vector3.Dot(n1, cylinder.RadialVector(p1)) < 0;
        bool hole2 = Vector3.Dot(n2, cylinder.RadialVector(p2)) < 0;
        if (hole1 != hole2) return null;
        return cylinder with { IsHole = hole1 };
    }
}
