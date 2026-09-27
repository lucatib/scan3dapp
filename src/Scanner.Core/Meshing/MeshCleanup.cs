using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Core.Meshing;

public static class MeshCleanup
{
    /// <summary>
    /// The piece standing on the table: drops the vertices on or below <paramref name="plane"/> (with its margin) and
    /// keeps every sizeable part: parts of at least <paramref name="minVertices"/> vertices and <paramref name="minFraction"/>
    /// of the largest part. Only when none is that large, the part nearest <paramref name="target"/>. Keeping the one part
    /// nearest the target instead made the result flip to a speck of noise that happened to lie closer. Parts that only
    /// touch through a gap of up to <paramref name="linkDistance"/> still count as connected, since a TSDF surface has
    /// small holes where no photo matched.
    /// </summary>
    public static TriangleMesh Piece(TriangleMesh mesh, Vector3 target, FittedSupportPlane? plane, float linkDistance,
        int minVertices = 30, float minFraction = 0.05f)
    {
        int n = mesh.Positions.Count;
        var keep = new bool[n];
        for (int i = 0; i < n; i++)
        {
            var p = mesh.Positions[i];
            keep[i] = plane is not { } table || p.Y > table.HeightAt(p.X, p.Z) + table.Margin;
        }

        // Union-find over kept vertices: joined by shared triangles and by proximity.
        var parent = Enumerable.Range(0, n).ToArray();
        int Find(int i)
        {
            while (parent[i] != i) i = parent[i] = parent[parent[i]];
            return i;
        }
        void Union(int a, int b) => parent[Find(a)] = Find(b);

        for (int t = 0; t < mesh.Indices.Count; t += 3)
        {
            int a = mesh.Indices[t], b = mesh.Indices[t + 1], c = mesh.Indices[t + 2];
            if (!keep[a] || !keep[b] || !keep[c]) continue;
            Union(a, b);
            Union(b, c);
        }
        var cells = new Dictionary<(int, int, int), List<int>>();
        for (int i = 0; i < n; i++)
        {
            if (!keep[i]) continue;
            var key = Cell(mesh.Positions[i], linkDistance);
            if (!cells.TryGetValue(key, out var list)) cells[key] = list = [];
            list.Add(i);
        }
        float linkSquared = linkDistance * linkDistance;
        foreach (var ((cx, cy, cz), list) in cells)
        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
        {
            if (!cells.TryGetValue((cx + dx, cy + dy, cz + dz), out var other)) continue;
            foreach (int i in list)
            foreach (int j in other)
                if (i < j && Find(i) != Find(j) && Vector3.DistanceSquared(mesh.Positions[i], mesh.Positions[j]) <= linkSquared)
                    Union(i, j);
        }

        int nearest = -1;
        float best = float.PositiveInfinity;
        for (int i = 0; i < n; i++)
        {
            if (!keep[i]) continue;
            float d = Vector3.DistanceSquared(mesh.Positions[i], target);
            if (d < best) { best = d; nearest = i; }
        }
        var result = new TriangleMesh();
        if (nearest < 0) return result;
        var sizes = new Dictionary<int, int>();
        for (int i = 0; i < n; i++)
            if (keep[i]) sizes[Find(i)] = sizes.GetValueOrDefault(Find(i)) + 1;
        int threshold = Math.Max(minVertices, (int)(minFraction * sizes.Values.Max()));
        var kept = sizes.Where(s => s.Value >= threshold).Select(s => s.Key).ToHashSet();
        if (kept.Count == 0) kept.Add(Find(nearest));
        var map = new int[n];
        for (int i = 0; i < n; i++)
        {
            map[i] = -1;
            if (!keep[i] || !kept.Contains(Find(i))) continue;
            map[i] = result.Positions.Count;
            result.Positions.Add(mesh.Positions[i]);
            result.Normals.Add(mesh.Normals[i]);
        }
        for (int t = 0; t < mesh.Indices.Count; t += 3)
        {
            int a = map[mesh.Indices[t]], b = map[mesh.Indices[t + 1]], c = map[mesh.Indices[t + 2]];
            if (a >= 0 && b >= 0 && c >= 0) result.Indices.AddRange([a, b, c]);
        }
        return result;
    }

    private static (int, int, int) Cell(Vector3 p, float size) =>
        ((int)MathF.Floor(p.X / size), (int)MathF.Floor(p.Y / size), (int)MathF.Floor(p.Z / size));
}
