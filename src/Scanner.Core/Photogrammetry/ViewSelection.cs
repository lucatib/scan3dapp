using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

public static class ViewSelection
{
    /// <summary>Below this the baseline is too short for depth; above it the views look too different to match.</summary>
    public const float MinAngleDegrees = 2f;
    public const float MaxAngleDegrees = 40f;

    /// <summary>The angle stereo pairs are ranked by: long enough baseline, still similar appearance.</summary>
    public const float BestAngleDegrees = 15f;

    /// <summary>
    /// Up to <paramref name="count"/> views that pair well with the reference: their rays meet the reference's at the
    /// target at 2-40 degrees (the closer to 15 the better), and the target is inside their image.
    /// </summary>
    public static List<int> Neighbours(IReadOnlyList<PhotoView> views, int reference, Vector3 target, int count)
    {
        var toReference = Vector3.Normalize(views[reference].CameraToWorld.Translation - target);
        var ranked = new List<(int Index, float Distance)>();
        for (int i = 0; i < views.Count; i++)
        {
            if (i == reference || !Pinhole.Sees(views[i], target)) continue;
            var toView = Vector3.Normalize(views[i].CameraToWorld.Translation - target);
            float angle = MathF.Acos(Math.Clamp(Vector3.Dot(toReference, toView), -1f, 1f)) * 180f / MathF.PI;
            if (angle < MinAngleDegrees || angle > MaxAngleDegrees) continue;
            ranked.Add((i, MathF.Abs(angle - BestAngleDegrees)));
        }
        return ranked.OrderBy(r => r.Distance).Take(count).Select(r => r.Index).ToList();
    }

    /// <summary><paramref name="count"/> reference views spread evenly over the capture order (all when fewer).</summary>
    public static List<int> References(int viewCount, int count)
    {
        if (viewCount <= count) return Enumerable.Range(0, viewCount).ToList();
        return Enumerable.Range(0, count).Select(i => (int)((i + 0.5) * viewCount / count)).ToList();
    }
}
