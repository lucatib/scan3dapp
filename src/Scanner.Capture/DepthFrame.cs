using System.Numerics;

namespace Scanner.Capture;

/// <summary>Depth map (z in meters, 0 = invalid) with the camera→world pose at capture time.</summary>
public sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)
{
    public float DepthAt(int u, int v) => Depth[v * Intrinsics.Width + u];
}
