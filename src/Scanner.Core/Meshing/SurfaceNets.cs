using System.Numerics;
using Scanner.Core.Fusion;

namespace Scanner.Core.Meshing;

/// <summary>
/// Naive Surface Nets: one vertex per cell crossed by the surface (average of the intersections
/// on the edges), one quad for each voxel edge with a sign change.
/// Cell (x, y, z) has as its corners the voxels from (x, y, z) to (x+1, y+1, z+1);
/// corner i has offset (i &amp; 1, (i &gt;&gt; 1) &amp; 1, (i &gt;&gt; 2) &amp; 1).
/// </summary>
public static class SurfaceNets
{
    private static readonly int[] EdgeBits = [1, 2, 4];

    public static TriangleMesh Extract(TsdfVolume volume)
    {
        var mesh = new TriangleMesh();
        var cells = new Dictionary<(int, int, int), int>();
        var corners = new float[8];

        foreach (var (bx, by, bz) in volume.AllocatedBlocks)
        for (int lz = 0; lz < TsdfVolume.BlockSize; lz++)
        for (int ly = 0; ly < TsdfVolume.BlockSize; ly++)
        for (int lx = 0; lx < TsdfVolume.BlockSize; lx++)
        {
            int x = bx * TsdfVolume.BlockSize + lx;
            int y = by * TsdfVolume.BlockSize + ly;
            int z = bz * TsdfVolume.BlockSize + lz;
            if (!TryReadCorners(volume, x, y, z, corners) || !HasSignChange(corners)) continue;

            cells[(x, y, z)] = mesh.Positions.Count;
            mesh.Positions.Add(CellVertex(volume.VoxelSize, x, y, z, corners));
            mesh.Normals.Add(CellNormal(corners));
        }

        foreach (var ((x, y, z), _) in cells)
        {
            volume.TryGet(x, y, z, out float v0, out _);
            // Order of cells around the edge: counterclockwise as seen from the positive axis direction.
            if (volume.TryGet(x + 1, y, z, out float vx, out _))
                EmitQuad(mesh, cells, v0, vx, (x, y - 1, z - 1), (x, y, z - 1), (x, y, z), (x, y - 1, z));
            if (volume.TryGet(x, y + 1, z, out float vy, out _))
                EmitQuad(mesh, cells, v0, vy, (x - 1, y, z - 1), (x - 1, y, z), (x, y, z), (x, y, z - 1));
            if (volume.TryGet(x, y, z + 1, out float vz, out _))
                EmitQuad(mesh, cells, v0, vz, (x - 1, y - 1, z), (x, y - 1, z), (x, y, z), (x - 1, y, z));
        }

        return mesh;
    }

    private static void EmitQuad(TriangleMesh mesh, Dictionary<(int, int, int), int> cells, float v0, float v1,
        (int, int, int) c0, (int, int, int) c1, (int, int, int) c2, (int, int, int) c3)
    {
        if ((v0 < 0) == (v1 < 0)) return;
        if (!cells.TryGetValue(c0, out int i0) || !cells.TryGetValue(c1, out int i1)
            || !cells.TryGetValue(c2, out int i2) || !cells.TryGetValue(c3, out int i3)) return;

        // v0 inside and v1 outside: the outward normal has the positive axis direction.
        if (v0 < 0) mesh.AddQuad(i0, i1, i2, i3);
        else mesh.AddQuad(i0, i3, i2, i1);
    }

    private static bool TryReadCorners(TsdfVolume volume, int x, int y, int z, float[] corners)
    {
        for (int i = 0; i < 8; i++)
            if (!volume.TryGet(x + (i & 1), y + ((i >> 1) & 1), z + ((i >> 2) & 1), out corners[i], out _))
                return false;
        return true;
    }

    private static bool HasSignChange(float[] corners)
    {
        bool anyInside = false, anyOutside = false;
        foreach (float c in corners)
        {
            if (c < 0) anyInside = true;
            else anyOutside = true;
        }
        return anyInside && anyOutside;
    }

    private static Vector3 CellVertex(float voxelSize, int x, int y, int z, float[] corners)
    {
        var sum = Vector3.Zero;
        int count = 0;
        for (int i = 0; i < 8; i++)
        foreach (int bit in EdgeBits)
        {
            if ((i & bit) != 0) continue;
            int j = i | bit;
            float a = corners[i], b = corners[j];
            if ((a < 0) == (b < 0)) continue;
            sum += Vector3.Lerp(Offset(i), Offset(j), a / (a - b));
            count++;
        }
        return (new Vector3(x, y, z) + sum / count) * voxelSize;
    }

    // Gradient of the trilinear interpolation at the cell center: points outward.
    private static Vector3 CellNormal(float[] c)
    {
        var g = new Vector3(
            c[1] + c[3] + c[5] + c[7] - (c[0] + c[2] + c[4] + c[6]),
            c[2] + c[3] + c[6] + c[7] - (c[0] + c[1] + c[4] + c[5]),
            c[4] + c[5] + c[6] + c[7] - (c[0] + c[1] + c[2] + c[3]));
        return g.LengthSquared() > 0 ? Vector3.Normalize(g) : Vector3.UnitZ;
    }

    private static Vector3 Offset(int i) => new(i & 1, (i >> 1) & 1, (i >> 2) & 1);
}
