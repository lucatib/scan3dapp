using System.Numerics;

namespace Scanner.Capture;

/// <summary>Mappa di profondità (z in metri, 0 = non valida) con la posa camera→mondo al momento dello scatto.</summary>
public sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)
{
    public float DepthAt(int u, int v) => Depth[v * Intrinsics.Width + u];
}
