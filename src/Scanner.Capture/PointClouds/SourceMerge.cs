using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>
/// Joins photogrammetry points and ARCore depth points into one voxel cloud. Photo points are far more precise
/// (millimetres against centimetres), so averaging the two would only blur them: every photo point goes in, and an
/// ARCore point goes in only where no photo point lies within <c>fillRadius</c> — on plain surfaces the photos
/// could not match.
/// </summary>
public static class SourceMerge
{
    public static Vector3[] Merge(IReadOnlyList<Vector3> photoPoints, IReadOnlyList<Vector3> depthPoints,
        float voxelSize = 0.005f, float fillRadius = 0.01f)
    {
        var grid = new Dictionary<(int, int, int), List<Vector3>>();
        foreach (var p in photoPoints)
        {
            var cell = Cell(p, fillRadius);
            if (!grid.TryGetValue(cell, out var list)) grid[cell] = list = [];
            list.Add(p);
        }

        var accumulator = new VoxelPointAccumulator(voxelSize);
        accumulator.AddRange(photoPoints);
        accumulator.AddRange(depthPoints.Where(p => !HasNeighbour(grid, p, fillRadius)));
        return accumulator.Snapshot();
    }

    private static bool HasNeighbour(Dictionary<(int, int, int), List<Vector3>> grid, Vector3 p, float radius)
    {
        var (cx, cy, cz) = Cell(p, radius);
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            if (!grid.TryGetValue((cx + dx, cy + dy, cz + dz), out var list)) continue;
            foreach (var q in list)
                if (Vector3.DistanceSquared(p, q) <= radius * radius) return true;
        }
        return false;
    }

    private static (int, int, int) Cell(Vector3 p, float size) =>
        ((int)MathF.Floor(p.X / size), (int)MathF.Floor(p.Y / size), (int)MathF.Floor(p.Z / size));
}
