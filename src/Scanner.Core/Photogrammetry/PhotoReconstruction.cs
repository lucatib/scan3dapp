using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <param name="ReferenceViews">How many photos get a depth map, spread evenly over the capture.</param>
/// <param name="Neighbours">How many photos each depth map is matched against.</param>
/// <param name="DepthMargin">Depth range searched around the target's depth in each reference photo, metres.</param>
/// <param name="RegionRadius">Points farther than this from the target are dropped, as ARCore depth is.</param>
/// <param name="Stereo">Stereo settings; the defaults when null.</param>
public sealed record ReconstructionOptions(int ReferenceViews = 12, int Neighbours = 4, float DepthMargin = 0.35f,
    float RegionRadius = 0.30f, StereoOptions? Stereo = null);

/// <summary>Dense points from photos whose poses are known: a plane-sweep depth map for each reference photo,
/// fused so that only points other depth maps confirm survive.</summary>
public static class PhotoReconstruction
{
    public static List<Vector3> DensePoints(IReadOnlyList<PhotoView> views, Vector3 target, ReconstructionOptions? options = null)
    {
        options ??= new ReconstructionOptions();
        var maps = new List<(PhotoView, DepthMap)>();
        foreach (int reference in ViewSelection.References(views.Count, options.ReferenceViews))
        {
            var view = views[reference];
            if (!Pinhole.Sees(view, target)) continue;
            var neighbours = ViewSelection.Neighbours(views, reference, target, options.Neighbours);
            if (neighbours.Count == 0) continue;
            float depth = Pinhole.ToCamera(view.CameraToWorld, target).Z;
            maps.Add((view, PlaneSweepStereo.Compute(view, neighbours.Select(i => views[i]).ToList(),
                depth - options.DepthMargin, depth + options.DepthMargin, options.Stereo)));
        }

        // Fusion confirms a point with two other depth maps, so fewer than three can confirm nothing.
        if (maps.Count < 3) return [];
        float radiusSquared = options.RegionRadius * options.RegionRadius;
        return DepthMapFusion.Fuse(maps).Where(p => Vector3.DistanceSquared(p, target) <= radiusSquared).ToList();
    }
}
