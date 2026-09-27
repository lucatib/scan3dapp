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
/// <param name="Carve">Complete the surface into a solid from where the photos see through (<see cref="SpaceCarver"/>).</param>
/// <param name="PieceMargin">Margin around a known piece extent, metres: the table ring beside the walls must be in the
/// box for the carving to find them.</param>
/// <param name="MaxCropPixels">Longest side of a reference crop; a larger one (a photo taken close to the piece) is averaged
/// down. That bounds the cost of a depth map, and keeps the depth steps near a pixel of disparity: close up, the box spans
/// a depth range the step cap could only cover coarsely, and coarse steps matched wrongly.</param>
/// <param name="Stereo">Matching settings; <see cref="StereoOptions.DepthSamples"/> is replaced per depth map.</param>
public sealed record VoxelReconstructionOptions(int ReferenceViews = 12, int Neighbours = 4, float VoxelSize = 0.003f,
    float TruncationVoxels = 4, float BoxPadding = 0.03f, int MinDepthSamples = 24, int MaxDepthSamples = 96,
    float MinWeight = 2, float MaxHalfWidth = 0.10f, bool Carve = true, float PieceMargin = 0.025f,
    int MaxCropPixels = 256, StereoOptions? Stereo = null);

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
    /// <param name="piece">The extent of the piece when already known (the live surface): the box then hugs it
    /// instead of ARCore's cluster, which is usually several times larger. Smaller crops, faster stereo.</param>
    public static VoxelReconstruction? Reconstruct(IReadOnlyList<PhotoView> views, Vector3 target,
        IReadOnlyList<Vector3> guide, VoxelReconstructionOptions? options = null, TextWriter? log = null,
        (Vector3 Min, Vector3 Max)? piece = null)
    {
        options ??= new VoxelReconstructionOptions();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var guidePlane = SupportPlaneFinder.Fit(guide);
        var (boxMin, boxMax) = piece is { } known
            ? PadToTable(known.Min - new Vector3(options.PieceMargin), known.Max + new Vector3(options.PieceMargin),
                guidePlane, options.BoxPadding)
            : ObjectBox(guide, target, guidePlane, options.BoxPadding, options.MaxHalfWidth);
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
            var crop = FitCrop(view.Crop(x0, y0, width, height), options);
            var others = neighbours.Select(i => views[i]).ToList();
            int samples = Samples(crop, others, near, far, options);
            var stereo = (options.Stereo ?? new StereoOptions()) with { DepthSamples = samples };
            var map = PlaneSweepStereo.Compute(crop, others, near, far, stereo);
            log?.WriteLine($"  photo {reference}: {crop.Image.Width}x{crop.Image.Height} px, {near:F3}-{far:F3} m, {samples} depths, {map.Depth.Count(d => d > 0)} matched, {clock.ElapsedMilliseconds} ms");
            maps.Add((crop, map));
        }
        if (maps.Count < 3) return null;

        var filtered = DepthMapFusion.Filter(maps);
        log?.WriteLine($"  cross-checked, {clock.ElapsedMilliseconds} ms");
        var carvingMaps = new List<(PhotoView View, float[] Depth)>();
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
            carvingMaps.Add((view, depth));
        }

        var mesh = SurfaceNets.Extract(volume, options.MinWeight);
        log?.WriteLine($"  fused, {clock.ElapsedMilliseconds} ms");
        var plane = TablePlane(mesh.Positions, guidePlane);
        if (options.Carve && plane is { } table)
        {
            // Walls and plain patches from where the photos see through, not only from where they matched.
            SpaceCarver.Complete(volume, carvingMaps, boxMin, boxMax, table, MathF.Max(options.MinWeight, 1));
            mesh = SurfaceNets.Extract(volume, options.MinWeight);
            // The carved underside lies a voxel or two above the table: cut at the table itself so that it stays.
            plane = table with { Margin = 0 };
        }
        var result = MeshCleanup.Piece(mesh, target, plane, 2 * options.VoxelSize);
        if (options.Carve && plane is { } carvedOn)
            MeshCleanup.FlattenOnto(result, carvedOn, 2.5f * options.VoxelSize); // a flat base and a clean bottom edge
        log?.WriteLine($"  mesh {mesh.Positions.Count} vertices, table {plane}, piece {result.Positions.Count} vertices, {clock.ElapsedMilliseconds} ms");
        return new VoxelReconstruction(result, mesh, plane, boxMin, boxMax, maps.Count);
    }

    /// <summary>
    /// The table under the photo surface, measured by the photos: the lowest large flat level of the surface. ARCore's
    /// table only narrows the search to <see cref="TableSearch"/> around it: its depth comes from camera motion, and a
    /// close pass once put it 2 cm under the floor the photos saw, which left the whole floor uncut.
    /// A table carries the piece: a level with next to nothing above it is the top of a flat piece whose floor the photos
    /// did not match (plain wood), unless it is where ARCore puts the table; then ARCore's table is used.
    /// </summary>
    public static FittedSupportPlane? TablePlane(IReadOnlyList<Vector3> surface, FittedSupportPlane? guide) =>
        TablePlane(surface, guide, out _);

    /// <param name="measured">True when the photos measured the plane; false when it is ARCore's (or none).</param>
    public static FittedSupportPlane? TablePlane(IReadOnlyList<Vector3> surface, FittedSupportPlane? guide, out bool measured)
    {
        measured = false;
        var near = guide is { } g
            ? surface.Where(p => MathF.Abs(p.Y - g.HeightAt(p.X, p.Z)) <= TableSearch).ToArray()
            : surface.ToArray();
        // The ARCore margin is its depth noise, often thicker than a book; the photo surface is far thinner than that.
        var fallback = guide is { } h ? h with { Margin = 0.005f } : (FittedSupportPlane?)null;
        if (near.Length < 30) return fallback;

        // Levels: 5 mm bins with their neighbours. Going up from the lowest, the table is the first level with a real
        // share of the surface (a floor ring can hold far less than a flat piece's top) that has piece standing on it.
        const float Bin = 0.005f;
        var counts = near.GroupBy(p => (int)MathF.Floor(p.Y / Bin)).ToDictionary(b => b.Key, b => b.Count());
        int Level(int bin) => counts.GetValueOrDefault(bin - 1) + counts.GetValueOrDefault(bin) + counts.GetValueOrDefault(bin + 1);
        var candidates = counts.Keys.Where(bin => Level(bin) >= Math.Max(30, near.Length * 8 / 100)).Order().ToList();
        foreach (int bin in candidates)
        {
            float height = (bin + 0.5f) * Bin;
            int above = near.Count(p => p.Y > height + 0.012f);
            if (above < near.Length / 20) continue;
            var level = near.Where(p => MathF.Abs(p.Y - height) <= 0.012f).ToArray();
            if (SupportPlaneFinder.Fit(level) is not { } fitted) continue;
            measured = true;
            return fitted;
        }
        // Nothing stands on any level: either the floor alone (then trust it where ARCore agrees) or a flat piece's top.
        if (candidates.Count > 0 && guide is { } a)
        {
            var lowest = near.Where(p => MathF.Abs(p.Y - (candidates[0] + 0.5f) * Bin) <= 0.012f).ToArray();
            if (SupportPlaneFinder.Fit(lowest) is { } floor && MathF.Abs(floor.HeightAt(lowest.Average(p => p.X), lowest.Average(p => p.Z))
                    - a.HeightAt(lowest.Average(p => p.X), lowest.Average(p => p.Z))) <= 0.01f)
            {
                measured = true;
                return floor;
            }
        }
        return fallback;
    }

    /// <summary>How far from ARCore's table height the photo table is searched for, metres.</summary>
    public const float TableSearch = 0.08f;


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
        return PadToTable(min, max, plane, padding);
    }

    /// <summary>The box reaching down past the table, so that a ring of it is reconstructed too.</summary>
    private static (Vector3 Min, Vector3 Max) PadToTable(Vector3 min, Vector3 max, FittedSupportPlane? plane, float padding)
    {
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

    /// <summary>The crop averaged down so that its longest side is at most <see cref="VoxelReconstructionOptions.MaxCropPixels"/>.</summary>
    internal static PhotoView FitCrop(PhotoView crop, VoxelReconstructionOptions options)
    {
        int longest = Math.Max(crop.Image.Width, crop.Image.Height);
        return longest <= options.MaxCropPixels ? crop : crop.Downscale((longest + options.MaxCropPixels - 1) / options.MaxCropPixels);
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
