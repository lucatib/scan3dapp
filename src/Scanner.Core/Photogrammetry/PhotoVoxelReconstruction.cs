using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Photogrammetry;

/// <param name="ReferenceViews">How many photos get a depth map, spread evenly over the capture.</param>
/// <param name="Neighbours">How many photos each depth map is matched against.</param>
/// <param name="VoxelSize">Edge of a TSDF voxel, metres.</param>
/// <param name="TruncationVoxels">TSDF truncation distance, in voxels.</param>
/// <param name="BoxPadding">Added around the object's ARCore points on every side, metres: ARCore depth is noisy and
/// thin parts go missing in it, so the box must be generous.</param>
/// <param name="MinDepthSamples">Fewest depth hypotheses per depth map.</param>
/// <param name="MaxDepthSamples">Most depth hypotheses per depth map. The count is otherwise chosen so that
/// consecutive hypotheses are about one pixel of disparity apart.</param>
/// <param name="MinWeight">Surface is only extracted where at least this many depth maps observed it.</param>
/// <param name="MaxHalfWidth">The box reaches at most this far from the target horizontally, metres, padding included:
/// the ARCore cluster of a small piece is often a blob several times its size.</param>
/// <param name="Stereo">Matching settings; <see cref="StereoOptions.DepthSamples"/> is replaced per depth map.</param>
public sealed record VoxelReconstructionOptions(int ReferenceViews = 12, int Neighbours = 4, float VoxelSize = 0.003f,
    float TruncationVoxels = 4, float BoxPadding = 0.03f, int MinDepthSamples = 24, int MaxDepthSamples = 96,
    float MinWeight = 2, float MaxHalfWidth = 0.10f, StereoOptions? Stereo = null);

/// <summary>The reconstructed piece: a surface mesh cut from the table (<paramref name="Surface"/> is before the cut), the table plane it was cut at,
/// and the world box that was searched.</summary>
public sealed record VoxelReconstruction(TriangleMesh Mesh, TriangleMesh Surface, FittedSupportPlane? Plane, Vector3 BoxMin, Vector3 BoxMax,
    int DepthMaps);

/// <summary>
/// Photogrammetry fused in voxels. The ARCore points only say where the piece is: they give a world box around it,
/// and each depth map is computed for just the part of its photo that box covers, over just the depths the box
/// spans. That is what makes it fast, and the narrow depth range is also what lets the hypotheses be close enough to
/// find the correlation peak. The depth maps are cross-checked, fused into a TSDF, meshed, and cut from the table.
/// </summary>
public static class PhotoVoxelReconstruction
{
    public static VoxelReconstruction? Reconstruct(IReadOnlyList<PhotoView> views, Vector3 target,
        IReadOnlyList<Vector3> guide, VoxelReconstructionOptions? options = null, TextWriter? log = null)
    {
        options ??= new VoxelReconstructionOptions();
        var guidePlane = SupportPlaneFinder.Fit(guide);
        var (boxMin, boxMax) = ObjectBox(guide, target, guidePlane, options.BoxPadding, options.MaxHalfWidth);
        var center = (boxMin + boxMax) / 2;

        var seeing = Enumerable.Range(0, views.Count).Where(i => Pinhole.Sees(views[i], center)).ToList();
        var maps = new List<(PhotoView View, DepthMap Map)>();
        foreach (int r in ViewSelection.References(seeing.Count, options.ReferenceViews))
        {
            int reference = seeing[r];
            var view = views[reference];
            if (Window(view, boxMin, boxMax) is not { } window) continue;
            var (x0, y0, width, height, near, far) = window;
            var neighbours = ViewSelection.Neighbours(views, reference, center, options.Neighbours);
            if (neighbours.Count == 0) continue;
            var crop = view.Crop(x0, y0, width, height);
            var others = neighbours.Select(i => views[i]).ToList();
            int samples = Samples(crop, others, near, far, options);
            var stereo = (options.Stereo ?? new StereoOptions()) with { DepthSamples = samples };
            var map = PlaneSweepStereo.Compute(crop, others, near, far, stereo);
            log?.WriteLine($"  photo {reference}: {width}x{height} px, {near:F3}-{far:F3} m, {samples} depths, {map.Depth.Count(d => d > 0)} matched");
            maps.Add((crop, map));
        }
        if (maps.Count < 3) return null;

        var filtered = DepthMapFusion.Filter(maps);
        var volume = new TsdfVolume(options.VoxelSize, options.TruncationVoxels * options.VoxelSize);
        for (int i = 0; i < maps.Count; i++)
        {
            var view = maps[i].View;
            var depth = filtered[i].Depth;
            // Only surface inside the box: a matched background would otherwise grow blocks far from the piece.
            for (int v = 0, index = 0; v < view.Intrinsics.Height; v++)
            for (int u = 0; u < view.Intrinsics.Width; u++, index++)
                if (depth[index] > 0 && !Inside(Pinhole.BackProject(view, u, v, depth[index]), boxMin, boxMax))
                    depth[index] = 0;
            log?.WriteLine($"  map {i}: {depth.Count(d => d > 0)} confirmed points in the box");
            volume.Integrate(new DepthFrame(view.Intrinsics, depth, view.CameraToWorld, 0));
        }

        var mesh = SurfaceNets.Extract(volume, options.MinWeight);
        var plane = TablePlane(mesh.Positions, guidePlane);
        var piece = MeshCleanup.Piece(mesh, target, plane, 2 * options.VoxelSize);
        log?.WriteLine($"  mesh {mesh.Positions.Count} vertices, table {plane}, piece {piece.Positions.Count} vertices");
        return new VoxelReconstruction(piece, mesh, plane, boxMin, boxMax, maps.Count);
    }

    /// <summary>
    /// The table under the photo surface. A flat piece can hold more of the surface than the table ring around it, so
    /// the fit only looks near the ARCore table height: ARCore places the table within about a centimetre, the
    /// photos then give its exact height and slope.
    /// </summary>
    public static FittedSupportPlane? TablePlane(IReadOnlyList<Vector3> surface, FittedSupportPlane? guide) =>
        TablePlane(surface, guide, out _);

    /// <param name="measured">True when the photos measured the plane; false when it is ARCore's (or none).</param>
    public static FittedSupportPlane? TablePlane(IReadOnlyList<Vector3> surface, FittedSupportPlane? guide, out bool measured)
    {
        measured = false;
        if (guide is not { } g)
        {
            var own = SupportPlaneFinder.Fit(surface);
            measured = own is not null;
            return own;
        }
        // The ARCore margin is its depth noise, often thicker than a book; the photo surface is far thinner than that.
        var fallback = g with { Margin = 0.005f };
        var near = surface.Where(p => MathF.Abs(p.Y - g.HeightAt(p.X, p.Z)) <= 0.01f).ToArray();
        if (near.Length < Math.Max(30, surface.Count / 6)) return fallback; // too little table in view to measure it
        float x = near.Average(p => p.X), z = near.Average(p => p.Z);
        if (SupportPlaneFinder.Fit(near) is not { } fitted || MathF.Abs(fitted.HeightAt(x, z) - g.HeightAt(x, z)) > 0.01f)
            return fallback;
        measured = true;
        return fitted;
    }

    /// <summary>
    /// A world box around the piece: the ARCore points above the table in the cluster nearest the target, padded,
    /// and reaching down past the table so that a ring of it is reconstructed too (the photo table plane is fitted
    /// on that ring). Without a usable cluster (a flat piece hides inside the ARCore noise band), a 16 cm cube around the
    /// target, which the reach clamp then trims.
    /// </summary>
    public static (Vector3 Min, Vector3 Max) ObjectBox(IReadOnlyList<Vector3> guide, Vector3 target,
        FittedSupportPlane? plane, float padding, float maxHalfWidth = float.PositiveInfinity)
    {
        var piece = plane is { } p
            ? ObjectIsolator.IsolateAbovePlane(guide, target, p, 0.01f)
            : ObjectIsolator.Isolate(guide, target, null, 0.01f);
        Vector3 min, max;
        if (piece.Length >= 20)
        {
            // Percentiles, not extremes: a few stray ARCore points must not blow the box up.
            min = new Vector3(Percentile(piece, q => q.X, .02f), Percentile(piece, q => q.Y, .02f), Percentile(piece, q => q.Z, .02f));
            max = new Vector3(Percentile(piece, q => q.X, .98f), Percentile(piece, q => q.Y, .98f), Percentile(piece, q => q.Z, .98f));
            min = Vector3.Min(min, target);
            max = Vector3.Max(max, target);
        }
        else
        {
            min = target - new Vector3(0.08f);
            max = target + new Vector3(0.08f);
        }
        min -= new Vector3(padding);
        max += new Vector3(padding);
        var reach = new Vector3(maxHalfWidth, float.PositiveInfinity, maxHalfWidth);
        min = Vector3.Max(min, target - reach);
        max = Vector3.Min(max, target + reach);
        if (plane is { } table)
        {
            float lowest = MathF.Min(MathF.Min(table.HeightAt(min.X, min.Z), table.HeightAt(max.X, min.Z)),
                MathF.Min(table.HeightAt(min.X, max.Z), table.HeightAt(max.X, max.Z)));
            min.Y = MathF.Min(min.Y, lowest - padding);
        }
        return (min, max);
    }

    /// <summary>The pixel window of <paramref name="view"/> the box covers, and the depth range it spans; null when the
    /// box is behind the camera, partly or wholly, or covers too few pixels to match.</summary>
    internal static (int X, int Y, int Width, int Height, float Near, float Far)? Window(PhotoView view, Vector3 min, Vector3 max)
    {
        var k = view.Intrinsics;
        float u0 = float.MaxValue, v0 = float.MaxValue, u1 = float.MinValue, v1 = float.MinValue;
        float near = float.MaxValue, far = 0;
        for (int corner = 0; corner < 8; corner++)
        {
            var world = new Vector3((corner & 1) == 0 ? min.X : max.X, (corner & 2) == 0 ? min.Y : max.Y,
                (corner & 4) == 0 ? min.Z : max.Z);
            var camera = Pinhole.ToCamera(view.CameraToWorld, world);
            if (camera.Z < 0.05f || !Pinhole.Project(k, camera, out float u, out float v)) return null;
            u0 = MathF.Min(u0, u); u1 = MathF.Max(u1, u);
            v0 = MathF.Min(v0, v); v1 = MathF.Max(v1, v);
            near = MathF.Min(near, camera.Z);
            far = MathF.Max(far, camera.Z);
        }
        int x0 = Math.Max(0, (int)MathF.Floor(u0)), y0 = Math.Max(0, (int)MathF.Floor(v0));
        int x1 = Math.Min(k.Width, (int)MathF.Ceiling(u1) + 1), y1 = Math.Min(k.Height, (int)MathF.Ceiling(v1) + 1);
        if (x1 - x0 < 24 || y1 - y0 < 24) return null;
        return (x0, y0, x1 - x0, y1 - y0, near, far);
    }

    /// <summary>Hypotheses about one pixel of disparity apart along the longest baseline, within the option bounds.</summary>
    internal static int Samples(PhotoView reference, IReadOnlyList<PhotoView> neighbours, float near, float far,
        VoxelReconstructionOptions options)
    {
        float baseline = neighbours.Max(n => Vector3.Distance(n.CameraToWorld.Translation, reference.CameraToWorld.Translation));
        float disparity = reference.Intrinsics.Fx * baseline * (1f / near - 1f / far);
        return Math.Clamp((int)MathF.Ceiling(disparity), options.MinDepthSamples, options.MaxDepthSamples);
    }

    internal static bool Inside(Vector3 p, Vector3 min, Vector3 max) =>
        p.X >= min.X && p.Y >= min.Y && p.Z >= min.Z && p.X <= max.X && p.Y <= max.Y && p.Z <= max.Z;

    private static float Percentile(Vector3[] points, Func<Vector3, float> axis, float q)
    {
        var values = points.Select(axis).Order().ToArray();
        return values[(int)((values.Length - 1) * q)];
    }
}
