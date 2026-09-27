using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Photogrammetry;

namespace Scanner.Core.Fusion;

/// <summary>
/// Completes a photo TSDF into a solid standing on the table. Stereo measures surfaces that face the cameras; steep
/// walls and plain patches it barely matches, so the TSDF leaves them ragged or open. Every depth map also says where
/// space is empty, though: the ray to each measured pixel crosses nothing. A voxel above the table that no depth map
/// sees through, and that some depth map sees behind a measured surface, is inside the piece. A wall then stands where
/// the photos stop seeing the table beside it, holes where nothing matched fill, and the underside closes at the table.
/// </summary>
public static class SpaceCarver
{
    /// <summary>Depth maps that must see a voxel behind their surface: one alone left flaps of unmatched table
    /// standing at the edge of the box.</summary>
    private const int MinHiddenMaps = 2;

    /// <param name="volume">The TSDF, completed in place inside the box. Voxels it measured near the surface keep their
    /// value; the rest take the carved solid, smoothed, with <paramref name="weight"/>.</param>
    /// <param name="maps">Cross-checked depth maps and their views.</param>
    /// <param name="table">Voxels at or below the table and its margin are empty: the solid ends there.</param>
    public static void Complete(TsdfVolume volume, IReadOnlyList<(PhotoView View, float[] Depth)> maps,
        Vector3 boxMin, Vector3 boxMax, FittedSupportPlane table, float weight)
    {
        float s = volume.VoxelSize;
        int x0 = (int)MathF.Ceiling(boxMin.X / s), y0 = (int)MathF.Ceiling(boxMin.Y / s), z0 = (int)MathF.Ceiling(boxMin.Z / s);
        int nx = (int)MathF.Floor(boxMax.X / s) - x0 + 1, ny = (int)MathF.Floor(boxMax.Y / s) - y0 + 1,
            nz = (int)MathF.Floor(boxMax.Z / s) - z0 + 1;
        if (nx < 3 || ny < 3 || nz < 3) return;

        // +1 empty, -1 solid.
        var solid = new float[nx * ny * nz];
        Parallel.For(0, nz, k =>
        {
            for (int j = 0; j < ny; j++)
            for (int i = 0; i < nx; i++)
            {
                var p = volume.VoxelToWorld(x0 + i, y0 + j, z0 + k);
                solid[(k * ny + j) * nx + i] = p.Y > table.HeightAt(p.X, p.Z) + table.Margin && IsSolid(maps, p, s) ? -1 : 1;
            }
        });

        // A 3x3x3 average turns the staircase of whole voxels into a slope Surface Nets can place vertices on.
        var smooth = new float[solid.Length];
        Parallel.For(0, nz, k =>
        {
            for (int j = 0; j < ny; j++)
            for (int i = 0; i < nx; i++)
            {
                float sum = 0;
                int count = 0;
                for (int dk = -1; dk <= 1; dk++)
                for (int dj = -1; dj <= 1; dj++)
                for (int di = -1; di <= 1; di++)
                {
                    int a = i + di, b = j + dj, c = k + dk;
                    if (a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
                    sum += solid[(c * ny + b) * nx + a];
                    count++;
                }
                smooth[(k * ny + j) * nx + i] = sum / count;
            }
        });

        for (int k = 0; k < nz; k++)
        for (int j = 0; j < ny; j++)
        for (int i = 0; i < nx; i++)
        {
            int x = x0 + i, y = y0 + j, z = z0 + k;
            var p = volume.VoxelToWorld(x, y, z);
            float value = smooth[(k * ny + j) * nx + i];
            if (p.Y <= table.HeightAt(p.X, p.Z) + table.Margin) value = 1; // no table surface, a closed underside
            // Measured surface is more precise than the carved one: keep it where the TSDF has it.
            else if (volume.TryGet(x, y, z, out float tsdf, out float measured) && measured > 0 && MathF.Abs(tsdf) < 1
                     && MathF.Sign(tsdf) == MathF.Sign(value)) continue;
            volume.Set(x, y, z, value, weight);
        }
    }

    /// <summary>No depth map sees through <paramref name="p"/>, and at least one sees it behind its surface.</summary>
    private static bool IsSolid(IReadOnlyList<(PhotoView View, float[] Depth)> maps, Vector3 p, float voxelSize)
    {
        int hidden = 0;
        foreach (var (view, depth) in maps)
        {
            var camera = Pinhole.ToCamera(view.CameraToWorld, p);
            if (!Pinhole.Project(view.Intrinsics, camera, out float u, out float v)) continue;
            int cu = (int)MathF.Round(u), cv = (int)MathF.Round(v);
            var k = view.Intrinsics;
            if (cu < 1 || cv < 1 || cu >= k.Width - 1 || cv >= k.Height - 1) continue;
            float tolerance = 1.5f * voxelSize + 0.005f * camera.Z;
            // The 3x3 pixels around the projection: depth maps are sparse, most pixels of plain surface are empty.
            bool behind = false;
            for (int dv = -1; dv <= 1; dv++)
            for (int du = -1; du <= 1; du++)
            {
                float d = depth[(cv + dv) * k.Width + cu + du];
                if (d <= 0) continue;
                if (d > camera.Z + tolerance) return false; // this ray passes the voxel and lands beyond it
                if (d < camera.Z - tolerance) behind = true;
            }
            if (behind) hidden++;
        }
        return hidden >= MinHiddenMaps;
    }
}
