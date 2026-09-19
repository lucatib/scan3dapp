using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Builders;

/// <summary>Convex solid as the intersection of the half-spaces Normal·x ≤ D.</summary>
public static class ConvexPolyhedronBuilder
{
    private const float DuplicateAngleDegrees = 5f;

    public static BrepSolid Build(IReadOnlyList<PlanePrimitive> planes, float tolerance)
    {
        var unique = MergeDuplicates(planes, tolerance);
        if (unique.Count < 4) throw new InvalidOperationException($"At least 4 distinct planes are required, found {unique.Count}.");

        var points = IntersectionVertices(unique, tolerance);
        if (points.Count < 4) throw new InvalidOperationException("Degenerate polyhedron: fewer than 4 vertices.");

        var vertices = points.Select(p => new BrepVertex(p)).ToList();
        var edges = new Dictionary<(int, int), BrepEdge>();
        var faces = new List<BrepFace>();

        foreach (var plane in unique)
        {
            var onPlane = Enumerable.Range(0, points.Count)
                .Where(i => MathF.Abs(plane.SignedDistance(points[i])) <= tolerance)
                .ToList();
            if (onPlane.Count < 3) continue;

            var centroid = onPlane.Aggregate(Vector3.Zero, (sum, i) => sum + points[i]) / onPlane.Count;
            var (u, v) = Basis.Orthonormal(plane.Normal);
            // Increasing angle in the (u, v) basis with u × v = normal: counterclockwise as seen from outside.
            var ordered = onPlane
                .OrderBy(i => MathF.Atan2(Vector3.Dot(points[i] - centroid, v), Vector3.Dot(points[i] - centroid, u)))
                .ToList();

            var loop = new List<OrientedEdge>();
            for (int k = 0; k < ordered.Count; k++)
            {
                int a = ordered[k], b = ordered[(k + 1) % ordered.Count];
                var key = (Math.Min(a, b), Math.Max(a, b));
                if (!edges.TryGetValue(key, out var edge))
                {
                    var start = points[key.Item1];
                    var end = points[key.Item2];
                    edge = new BrepEdge(vertices[key.Item1], vertices[key.Item2], new LineCurve(start, Vector3.Normalize(end - start)));
                    edges[key] = edge;
                }
                loop.Add(new OrientedEdge(edge, SameSense: a == key.Item1));
            }
            faces.Add(new BrepFace(new PlaneSurface(centroid, plane.Normal, u), [new BrepLoop(loop)], sameSense: true));
        }

        return new BrepSolid(faces);
    }

    private static List<PlanePrimitive> MergeDuplicates(IReadOnlyList<PlanePrimitive> planes, float tolerance)
    {
        float cosLimit = MathF.Cos(DuplicateAngleDegrees * MathF.PI / 180f);
        var unique = new List<PlanePrimitive>();
        foreach (var plane in planes)
        {
            bool duplicate = unique.Any(u =>
                Vector3.Dot(u.Normal, plane.Normal) > cosLimit && MathF.Abs(u.D - plane.D) < 3 * tolerance + 1e-3f);
            if (!duplicate) unique.Add(plane);
        }
        return unique;
    }

    private static List<Vector3> IntersectionVertices(List<PlanePrimitive> planes, float tolerance)
    {
        var points = new List<Vector3>();
        for (int i = 0; i < planes.Count; i++)
        for (int j = i + 1; j < planes.Count; j++)
        for (int k = j + 1; k < planes.Count; k++)
        {
            PlanePrimitive a = planes[i], b = planes[j], c = planes[k];
            var m = new double[,]
            {
                { a.Normal.X, a.Normal.Y, a.Normal.Z },
                { b.Normal.X, b.Normal.Y, b.Normal.Z },
                { c.Normal.X, c.Normal.Y, c.Normal.Z },
            };
            if (!Linear3.TrySolve(m, new double[] { a.D, b.D, c.D }, out var x)) continue;

            var p = new Vector3((float)x[0], (float)x[1], (float)x[2]);
            if (planes.Any(plane => plane.SignedDistance(p) > tolerance)) continue;
            if (points.Any(q => Vector3.Distance(p, q) <= tolerance)) continue;
            points.Add(p);
        }
        return points;
    }
}
