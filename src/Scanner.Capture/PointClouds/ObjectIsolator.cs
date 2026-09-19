using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Separates the scanned piece from its surroundings: drops the support plane, then keeps the
/// connected cluster of points nearest to the target point.</summary>
public static class ObjectIsolator
{
    public static Vector3[] Isolate(IReadOnlyList<Vector3> points, Vector3 target, float? supportPlaneHeight,
        float linkDistance, float planeMargin = 0.004f)
    {
        if (linkDistance <= 0) throw new ArgumentOutOfRangeException(nameof(linkDistance));
        var kept = supportPlaneHeight is { } height
            ? points.Where(p => p.Y > height + planeMargin).ToList()
            : points.ToList();
        if (kept.Count == 0) return [];

        var cells = new Dictionary<(int, int, int), List<int>>();
        for (int i = 0; i < kept.Count; i++)
        {
            var key = CellOf(kept[i], linkDistance);
            if (!cells.TryGetValue(key, out var list)) cells[key] = list = [];
            list.Add(i);
        }

        var visited = new HashSet<(int, int, int)>();
        List<(int, int, int)>? best = null;
        float bestDistance = float.PositiveInfinity;
        foreach (var start in cells.Keys)
        {
            if (!visited.Add(start)) continue;
            var component = new List<(int, int, int)>();
            var queue = new Queue<(int, int, int)>();
            queue.Enqueue(start);
            float nearest = float.PositiveInfinity;
            while (queue.Count > 0)
            {
                var cell = queue.Dequeue();
                component.Add(cell);
                foreach (int i in cells[cell]) nearest = MathF.Min(nearest, Vector3.DistanceSquared(kept[i], target));
                foreach (var neighbour in Neighbours(cell))
                    if (cells.ContainsKey(neighbour) && visited.Add(neighbour)) queue.Enqueue(neighbour);
            }
            if (nearest < bestDistance)
            {
                bestDistance = nearest;
                best = component;
            }
        }

        return best!.SelectMany(cell => cells[cell]).Select(i => kept[i]).ToArray();
    }

    private static (int, int, int) CellOf(Vector3 p, float size) =>
        ((int)MathF.Floor(p.X / size), (int)MathF.Floor(p.Y / size), (int)MathF.Floor(p.Z / size));

    private static IEnumerable<(int, int, int)> Neighbours((int X, int Y, int Z) c)
    {
        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            if (dx != 0 || dy != 0 || dz != 0) yield return (c.X + dx, c.Y + dy, c.Z + dz);
    }
}
