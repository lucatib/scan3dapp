using System.Numerics;
using System.Runtime.InteropServices;

namespace Scanner.Capture.PointClouds;

/// <summary>Accumulates points into a voxel grid; each occupied voxel yields the mean of its points. Thread-safe.</summary>
public sealed class VoxelPointAccumulator
{
    private readonly Dictionary<(int, int, int), Cell> _cells = new();
    private readonly object _gate = new();

    public VoxelPointAccumulator(float voxelSize)
    {
        if (voxelSize <= 0) throw new ArgumentOutOfRangeException(nameof(voxelSize));
        VoxelSize = voxelSize;
    }

    public float VoxelSize { get; }

    public int CellCount
    {
        get { lock (_gate) return _cells.Count; }
    }

    public void AddRange(IEnumerable<Vector3> points)
    {
        lock (_gate)
        {
            foreach (var p in points)
            {
                var key = ((int)MathF.Floor(p.X / VoxelSize), (int)MathF.Floor(p.Y / VoxelSize), (int)MathF.Floor(p.Z / VoxelSize));
                ref var cell = ref CollectionsMarshal.GetValueRefOrAddDefault(_cells, key, out _);
                cell.Sum += p;
                cell.Count++;
            }
        }
    }

    public Vector3[] Snapshot(int minObservations = 1)
    {
        lock (_gate)
        {
            var result = new List<Vector3>(_cells.Count);
            foreach (var cell in _cells.Values)
                if (cell.Count >= minObservations) result.Add(cell.Sum / cell.Count);
            return result.ToArray();
        }
    }

    public void Clear()
    {
        lock (_gate) _cells.Clear();
    }

    private struct Cell
    {
        public Vector3 Sum;
        public int Count;
    }
}
