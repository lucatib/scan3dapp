using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

public static class DepthMapFusion
{
    /// <summary>
    /// World points from every depth map, keeping only those that at least <paramref name="minAgreeing"/> other
    /// depth maps confirm: projected into another view, the point's depth must be within
    /// <paramref name="relativeTolerance"/> of what that view measured at that pixel. A wrong match is rarely wrong
    /// the same way in two other views, so this removes most of them.
    /// </summary>
    public static List<Vector3> Fuse(IReadOnlyList<(PhotoView View, DepthMap Map)> maps, int minAgreeing = 2,
        float relativeTolerance = 0.01f)
    {
        var perMap = new List<Vector3>[maps.Count];
        Parallel.For(0, maps.Count, i =>
        {
            var (view, map) = maps[i];
            var kept = new List<Vector3>();
            for (int v = 0; v < map.Height; v++)
            for (int u = 0; u < map.Width; u++)
            {
                float d = map.Depth[v * map.Width + u];
                if (d <= 0) continue;
                var world = Pinhole.BackProject(view, u, v, d);
                int agreeing = 0;
                for (int j = 0; j < maps.Count && agreeing < minAgreeing; j++)
                {
                    if (j == i) continue;
                    var (other, otherMap) = maps[j];
                    var camera = Pinhole.ToCamera(other.CameraToWorld, world);
                    if (!Pinhole.Project(other.Intrinsics, camera, out float x, out float y)) continue;
                    int px = (int)MathF.Round(x), py = (int)MathF.Round(y);
                    if (px < 0 || py < 0 || px >= otherMap.Width || py >= otherMap.Height) continue;
                    float measured = otherMap.Depth[py * otherMap.Width + px];
                    if (measured > 0 && MathF.Abs(camera.Z - measured) <= relativeTolerance * measured) agreeing++;
                }
                if (agreeing >= minAgreeing) kept.Add(world);
            }
            perMap[i] = kept;
        });
        return perMap.SelectMany(p => p).ToList();
    }
}
