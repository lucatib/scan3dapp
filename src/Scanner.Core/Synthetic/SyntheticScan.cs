using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Synthetic;

/// <summary>Scansione simulata: viste distribuite su una sfera attorno all'origine.</summary>
public static class SyntheticScan
{
    public static CameraIntrinsics DefaultIntrinsics => new(320, 240, 300f, 300f, 160f, 120f);

    public static IReadOnlyList<DepthFrame> Capture(ISdf shape, int views, float cameraDistance,
        CameraIntrinsics intrinsics, float noiseSigma, int seed)
    {
        var poses = CameraPoses.FibonacciSphere(views, cameraDistance, Vector3.Zero);
        return poses
            .Select((pose, i) => SyntheticDepthRenderer.Render(shape, intrinsics, pose, noiseSigma, seed + i, i))
            .ToList();
    }
}
