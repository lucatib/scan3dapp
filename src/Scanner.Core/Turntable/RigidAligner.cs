using System.Numerics;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Turntable;

/// <summary>Points with normals on a voxel grid: the first point to land in a voxel keeps it, so the model grows with
/// new surface rather than with repeated views of the same one.</summary>
public sealed class PointModel(float voxelSize)
{
    private readonly Dictionary<(int, int, int), int> _voxels = new();
    private readonly Dictionary<(int, int, int), List<int>> _cells = new();
    private readonly float _cellSize = 4 * voxelSize;

    public List<Vector3> Positions { get; } = new();
    public List<Vector3> Normals { get; } = new();
    public int Count => Positions.Count;

    public void Add(Vector3 position, Vector3 normal)
    {
        var voxel = Key(position, voxelSize);
        if (!_voxels.TryAdd(voxel, Positions.Count)) return;
        var cell = Key(position, _cellSize);
        if (!_cells.TryGetValue(cell, out var list)) _cells[cell] = list = [];
        list.Add(Positions.Count);
        Positions.Add(position);
        Normals.Add(normal);
    }

    /// <summary>The model point closest to <paramref name="p"/> within <paramref name="maxDistance"/> (at most two
    /// cells, <c>8 × voxel size</c>), or -1.</summary>
    public int Nearest(Vector3 p, float maxDistance)
    {
        int reach = (int)MathF.Ceiling(maxDistance / _cellSize);
        var (cx, cy, cz) = Key(p, _cellSize);
        int best = -1;
        float bestSquared = maxDistance * maxDistance;
        for (int dz = -reach; dz <= reach; dz++)
        for (int dy = -reach; dy <= reach; dy++)
        for (int dx = -reach; dx <= reach; dx++)
        {
            if (!_cells.TryGetValue((cx + dx, cy + dy, cz + dz), out var list)) continue;
            foreach (int i in list)
            {
                float d = Vector3.DistanceSquared(Positions[i], p);
                if (d < bestSquared) { bestSquared = d; best = i; }
            }
        }
        return best;
    }

    private static (int, int, int) Key(Vector3 p, float size) =>
        ((int)MathF.Floor(p.X / size), (int)MathF.Floor(p.Y / size), (int)MathF.Floor(p.Z / size));
}

/// <summary>How well a set of points was fitted onto a model.</summary>
/// <param name="Transform">Maps the source points onto the model (row-vector convention).</param>
/// <param name="RmsError">Point-to-plane error of the matched points, metres.</param>
/// <param name="InlierFraction">Share of the source points that found a model point within the match distance.</param>
public readonly record struct Alignment(Matrix4x4 Transform, float RmsError, float InlierFraction);

/// <summary>
/// Point-to-plane ICP: moves a depth frame's points onto a model of the same surface. Each round matches every point to
/// its nearest model point, then solves the small rotation and translation that bring the points onto the matched
/// points' tangent planes (Gauss-Newton on six unknowns, Huber-weighted so that a few wrong matches do not pull).
/// </summary>
public static class RigidAligner
{
    /// <summary>Residuals above this (metres) count linearly rather than squared.</summary>
    private const float HuberThreshold = 0.001f;

    /// <summary>A point whose normal turns away from its match's by more than about 60° is not the same surface.</summary>
    private const float MinNormalAgreement = 0.5f;

    public static Alignment Align(IReadOnlyList<Vector3> points, IReadOnlyList<Vector3> normals, PointModel model,
        Matrix4x4 initial, float maxDistance, int iterations = 30)
    {
        var transform = initial;
        float rms = float.PositiveInfinity, fraction = 0;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            // Shrinks to a third of the reach over the rounds: wide to catch the first matches, tight once it is close.
            float reach = maxDistance * MathF.Max(1f / 3, 1 - iteration / 10f);
            var normal = new double[6, 6];
            var rhs = new double[6];
            double squared = 0;
            int matched = 0;
            for (int i = 0; i < points.Count; i++)
            {
                var p = Vector3.Transform(points[i], transform);
                int j = model.Nearest(p, reach);
                if (j < 0) continue;
                var n = model.Normals[j];
                if (Vector3.Dot(Vector3.TransformNormal(normals[i], transform), n) < MinNormalAgreement) continue;
                float r = Vector3.Dot(p - model.Positions[j], n);
                float w = MathF.Abs(r) <= HuberThreshold ? 1 : HuberThreshold / MathF.Abs(r);
                var c = Vector3.Cross(p, n);
                Span<double> jacobian = [c.X, c.Y, c.Z, n.X, n.Y, n.Z];
                for (int a = 0; a < 6; a++)
                {
                    rhs[a] -= w * jacobian[a] * r;
                    for (int b = 0; b <= a; b++) normal[a, b] += w * jacobian[a] * jacobian[b];
                }
                squared += r * r;
                matched++;
            }
            if (matched < 6) break;
            rms = (float)Math.Sqrt(squared / matched);
            fraction = (float)matched / points.Count;

            double[] x;
            try
            {
                x = DenseCholesky.Solve(normal, rhs);
            }
            catch (ArgumentException)
            {
                break; // degenerate (a plane alone cannot fix all six): keep the last transform
            }
            var omega = new Vector3((float)x[0], (float)x[1], (float)x[2]);
            var step = omega.LengthSquared() > 0
                ? Matrix4x4.CreateFromAxisAngle(Vector3.Normalize(omega), omega.Length())
                : Matrix4x4.Identity;
            step.Translation = new Vector3((float)x[3], (float)x[4], (float)x[5]);
            transform *= step;
            if (omega.Length() < 1e-5f && step.Translation.Length() < 1e-6f) break;
        }
        return new Alignment(transform, rms, fraction);
    }
}
