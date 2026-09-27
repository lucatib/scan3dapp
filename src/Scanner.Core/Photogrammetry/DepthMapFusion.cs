using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

public static class DepthMapFusion
{
    /// <summary>
    /// World points from every depth map, keeping only those that at least <paramref name="minAgreeing"/> other
    /// depth maps confirm (see <see cref="Filter"/>).
    /// </summary>
    public static List<Vector3> Fuse(IReadOnlyList<(PhotoView View, DepthMap Map)> maps, int minAgreeing = 2,
        float relativeTolerance = 0.01f)
    {
        var filtered = Filter(maps, minAgreeing, relativeTolerance);
        var points = new List<Vector3>();
        for (int i = 0; i < maps.Count; i++)
        {
            var (view, _) = maps[i];
            var map = filtered[i];
            for (int v = 0; v < map.Height; v++)
            for (int u = 0; u < map.Width; u++)
            {
                float d = map.Depth[v * map.Width + u];
                if (d > 0) points.Add(Pinhole.BackProject(view, u, v, d));
            }
        }
        return points;
    }

    /// <summary>
    /// Copies of the depth maps in which only the pixels that at least <paramref name="minAgreeing"/> other depth
    /// maps confirm keep their depth: projected into another view, the point's depth must be within
    /// <paramref name="relativeTolerance"/> of what that view measured at that pixel. A wrong match is rarely wrong
    /// the same way in two other views, so this removes most of them.
    /// </summary>
    public static DepthMap[] Filter(IReadOnlyList<(PhotoView View, DepthMap Map)> maps, int minAgreeing = 2,
        float relativeTolerance = 0.01f)
    {
        var result = new DepthMap[maps.Count];
        Parallel.For(0, maps.Count, i =>
        {
            var (view, map) = maps[i];
            var depth = new float[map.Depth.Length];
            var score = new float[map.Score.Length];
            for (int v = 0; v < map.Height; v++)
            for (int u = 0; u < map.Width; u++)
            {
                int index = v * map.Width + u;
                float d = map.Depth[index];
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
                if (agreeing < minAgreeing) continue;
                depth[index] = d;
                score[index] = map.Score[index];
            }
            result[i] = new DepthMap(map.Width, map.Height, depth, score);
        });
        return result;
    }
}
