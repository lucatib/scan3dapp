using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <summary>Depth of each pixel of a reference view (metres along its optical axis, 0 = none) and the matching
/// score (normalized cross-correlation, -1 to 1) that chose it.</summary>
public sealed record DepthMap(int Width, int Height, float[] Depth, float[] Score);

/// <summary>Pinhole projection with a camera→world pose in the pipeline's convention (OpenCV axes, row vectors).</summary>
public static class Pinhole
{
    /// <summary>A world point in the camera's own frame.</summary>
    public static Vector3 ToCamera(Matrix4x4 cameraToWorld, Vector3 world)
    {
        var d = world - cameraToWorld.Translation;
        return new Vector3(
            cameraToWorld.M11 * d.X + cameraToWorld.M12 * d.Y + cameraToWorld.M13 * d.Z,
            cameraToWorld.M21 * d.X + cameraToWorld.M22 * d.Y + cameraToWorld.M23 * d.Z,
            cameraToWorld.M31 * d.X + cameraToWorld.M32 * d.Y + cameraToWorld.M33 * d.Z);
    }

    /// <summary>Projects a camera-frame point to pixel coordinates; false when it is behind the camera.</summary>
    public static bool Project(CameraIntrinsics k, Vector3 camera, out float u, out float v)
    {
        u = v = 0;
        if (camera.Z <= 1e-6f) return false;
        u = k.Fx * camera.X / camera.Z + k.Cx;
        v = k.Fy * camera.Y / camera.Z + k.Cy;
        return true;
    }

    /// <summary>True when the world point lies in front of the camera and inside its image.</summary>
    public static bool Sees(PhotoView view, Vector3 world)
    {
        var k = view.Intrinsics;
        return Project(k, ToCamera(view.CameraToWorld, world), out float u, out float v)
               && u >= 0 && v >= 0 && u <= k.Width - 1 && v <= k.Height - 1;
    }

    /// <summary>The pixel (u, v) at the given depth, in world coordinates.</summary>
    public static Vector3 BackProject(PhotoView view, float u, float v, float depth)
    {
        var k = view.Intrinsics;
        return Vector3.Transform(new Vector3((u - k.Cx) / k.Fx * depth, (v - k.Cy) / k.Fy * depth, depth), view.CameraToWorld);
    }
}
