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
                int score = CountInliers(cloud, remaining, candidate, options.DistanceThreshold, cosThreshold);
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
                best = refined;
                inliers = CollectInliers(cloud, remaining, best, options.DistanceThreshold, cosThreshold);
            }
            if (inliers.Count < options.MinInliers) break;

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
        bool isHole = Vector3.Dot(n1, cylinder.RadialVector(p1)) < 0;
        return cylinder with { IsHole = isHole };
    }
}
