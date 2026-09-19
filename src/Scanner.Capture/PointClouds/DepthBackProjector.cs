using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Sphere in world space that bounds the piece being scanned.</summary>
public readonly record struct ScanRegion(Vector3 Center, float Radius)
{
    public bool Contains(Vector3 p) => Vector3.DistanceSquared(p, Center) <= Radius * Radius;
}

/// <summary>Which depth pixels become points. Confidence is 0–255 (ARCore raw depth confidence).</summary>
public sealed record DepthFilter(float MinDepth = 0.10f, float MaxDepth = 1.50f, byte MinConfidence = 128, ScanRegion? Region = null);

public static class DepthBackProjector
{
    /// <summary>Back-projects every accepted depth pixel into world space and appends it to <paramref name="output"/>.</summary>
    public static void Project(DepthFrame frame, byte[]? confidence, DepthFilter filter, List<Vector3> output)
    {
        var k = frame.Intrinsics;
        if (confidence is not null && confidence.Length != frame.Depth.Length)
            throw new ArgumentException("Confidence buffer size does not match the depth buffer.", nameof(confidence));

        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
        {
            int i = v * k.Width + u;
            float d = frame.Depth[i];
            if (d < filter.MinDepth || d > filter.MaxDepth) continue;
            if (confidence is not null && confidence[i] < filter.MinConfidence) continue;

            var cameraPoint = new Vector3((u - k.Cx) / k.Fx * d, (v - k.Cy) / k.Fy * d, d);
            var world = Vector3.Transform(cameraPoint, frame.CameraToWorld);
            if (filter.Region is { } region && !region.Contains(world)) continue;
            output.Add(world);
        }
    }
}
