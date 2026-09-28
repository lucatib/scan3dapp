using System.Numerics;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// How far around the piece the photos went. The reconstruction only knows a side is empty when photos saw past it:
/// a grinder photographed over 250° of the loop came out with wings on the 110° nobody photographed.
/// </summary>
public static class LoopCoverage
{
    /// <summary>A gap wider than this leaves a side of the piece unseen: the scan should warn before finishing.</summary>
    public const float MaxGapDegrees = 60f;

    /// <summary>
    /// The widest turn around <paramref name="target"/> (about the vertical, Y) with no camera in it: 360 with fewer than
    /// two cameras, the step between photos for a closed loop.
    /// </summary>
    public static float LargestGapDegrees(IEnumerable<Vector3> cameraPositions, Vector3 target)
    {
        var azimuths = cameraPositions
            .Select(p => MathF.Atan2(p.Z - target.Z, p.X - target.X) * 180f / MathF.PI)
            .Order()
            .ToList();
        if (azimuths.Count < 2) return 360f;
        float largest = azimuths[0] + 360f - azimuths[^1]; // across the ±180° seam
        for (int i = 1; i < azimuths.Count; i++)
            largest = MathF.Max(largest, azimuths[i] - azimuths[i - 1]);
        return largest;
    }
}
