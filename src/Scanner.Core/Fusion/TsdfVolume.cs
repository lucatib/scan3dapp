using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Fusion;

/// <summary>
/// Sparse TSDF volume: blocks of 8³ voxels allocated only near the observed surface.
/// The voxel (x, y, z) has its center at (x, y, z)·VoxelSize in the world coordinate system.
/// Values normalized in [-1, 1], positive outside the object.
/// </summary>
public sealed class TsdfVolume
{
    public const int BlockSize = 8;
    private const int BlockVoxelCount = BlockSize * BlockSize * BlockSize;
    private const int KeyOffset = 1 << 20;
    private const float MaxWeight = 64f;

    private readonly Dictionary<long, Block> _blocks = new();

    public TsdfVolume(float voxelSize, float truncationDistance)
    {
        if (voxelSize <= 0) throw new ArgumentOutOfRangeException(nameof(voxelSize));
        if (truncationDistance < voxelSize) throw new ArgumentOutOfRangeException(nameof(truncationDistance));
        VoxelSize = voxelSize;
        TruncationDistance = truncationDistance;
    }

    public float VoxelSize { get; }
    public float TruncationDistance { get; }
    public int BlockCount => _blocks.Count;
    public IEnumerable<(int X, int Y, int Z)> AllocatedBlocks => _blocks.Keys.Select(Unpack);

    public Vector3 VoxelToWorld(int x, int y, int z) => new Vector3(x, y, z) * VoxelSize;

    public bool TryGet(int x, int y, int z, out float tsdf, out float weight)
    {
        if (_blocks.TryGetValue(Pack(x >> 3, y >> 3, z >> 3), out var block))
        {
            int i = LocalIndex(x, y, z);
            tsdf = block.Tsdf[i];
            weight = block.Weight[i];
            return weight > 0;
        }
        tsdf = 0;
        weight = 0;
        return false;
    }

    public void Set(int x, int y, int z, float tsdf, float weight)
    {
        var block = GetOrAdd(Pack(x >> 3, y >> 3, z >> 3));
        int i = LocalIndex(x, y, z);
        block.Tsdf[i] = tsdf;
        block.Weight[i] = weight;
    }

    public void Integrate(DepthFrame frame)
    {
        if (!Matrix4x4.Invert(frame.CameraToWorld, out var worldToCamera))
            throw new ArgumentException("Camera pose is not invertible.", nameof(frame));

        var touched = AllocateBlocksNearSurface(frame);
        Parallel.ForEach(touched, key => UpdateBlock(key, _blocks[key], frame, worldToCamera));
    }

    private HashSet<long> AllocateBlocksNearSurface(DepthFrame frame)
    {
        var k = frame.Intrinsics;
        var touched = new HashSet<long>();
        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
        {
            float d = frame.DepthAt(u, v);
            if (d <= 0) continue;
            var ray = new Vector3((u - k.Cx) / k.Fx, (v - k.Cy) / k.Fy, 1f);
            for (float z = d - TruncationDistance; z <= d + TruncationDistance; z += VoxelSize)
            {
                if (z <= 0) continue;
                var p = Vector3.Transform(ray * z, frame.CameraToWorld) / VoxelSize;
                touched.Add(Pack((int)MathF.Round(p.X) >> 3, (int)MathF.Round(p.Y) >> 3, (int)MathF.Round(p.Z) >> 3));
            }
        }
        foreach (long key in touched) GetOrAdd(key);
        return touched;
    }

    private void UpdateBlock(long key, Block block, DepthFrame frame, Matrix4x4 worldToCamera)
    {
        var k = frame.Intrinsics;
        var (bx, by, bz) = Unpack(key);
        for (int lz = 0; lz < BlockSize; lz++)
        for (int ly = 0; ly < BlockSize; ly++)
        for (int lx = 0; lx < BlockSize; lx++)
        {
            var world = VoxelToWorld(bx * BlockSize + lx, by * BlockSize + ly, bz * BlockSize + lz);
            var pc = Vector3.Transform(world, worldToCamera);
            if (pc.Z <= 0) continue;
            int u = (int)MathF.Round(k.Fx * pc.X / pc.Z + k.Cx);
            int v = (int)MathF.Round(k.Fy * pc.Y / pc.Z + k.Cy);
            if (u < 0 || v < 0 || u >= k.Width || v >= k.Height) continue;
            float d = frame.DepthAt(u, v);
            if (d <= 0) continue;
            float sdf = d - pc.Z;
            if (sdf < -TruncationDistance) continue;

            float tsdf = MathF.Min(1f, sdf / TruncationDistance);
            int i = lx + BlockSize * (ly + BlockSize * lz);
            float w = block.Weight[i];
            block.Tsdf[i] = (block.Tsdf[i] * w + tsdf) / (w + 1);
            block.Weight[i] = MathF.Min(w + 1, MaxWeight);
        }
    }

    private Block GetOrAdd(long key)
    {
        if (!_blocks.TryGetValue(key, out var block))
        {
            block = new Block();
            _blocks[key] = block;
        }
        return block;
    }

    private static long Pack(int bx, int by, int bz) =>
        ((long)(bx + KeyOffset) << 42) | ((long)(by + KeyOffset) << 21) | (long)(bz + KeyOffset);

    private static (int X, int Y, int Z) Unpack(long key) =>
        ((int)(key >> 42) - KeyOffset, (int)((key >> 21) & 0x1FFFFF) - KeyOffset, (int)(key & 0x1FFFFF) - KeyOffset);

    private static int LocalIndex(int x, int y, int z) => (x & 7) + BlockSize * ((y & 7) + BlockSize * (z & 7));

    private sealed class Block
    {
        public readonly float[] Tsdf = new float[BlockVoxelCount];
        public readonly float[] Weight = new float[BlockVoxelCount];
    }
}
