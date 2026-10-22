using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;

namespace Scanner.Core.Turntable;

/// <summary>What one photo told about the mat.</summary>
/// <param name="Kept">The photo was posed and kept for the model.</param>
/// <param name="AngleDegrees">Where around the mat the camera is (0–360), when posed.</param>
public readonly record struct MatPhotoResult(bool Kept, MatPoseResult? Pose, float? AngleDegrees, string? Reason);

/// <summary>
/// Turntable photogrammetry on the printed mat: every photo is posed from the mat dots it shows, in the mat's frame
/// (the mat is y = 0, +Y up, true scale), and the posed photos go through the same voxel reconstruction as a handheld
/// scan. With exact poses there is nothing to refine, and the table is known to be y = 0.
/// </summary>
public sealed class MatTurntableScan
{
    /// <summary>A new photo is kept once the mat has turned this far since the last one kept.</summary>
    public const float StepDegrees = 5;

    /// <summary>Photos whose dots fit worse than this are not trusted (a blurred or partly misread mat).</summary>
    public const float MaxRmsPixels = 1.5f;

    /// <summary>The piece is assumed no taller than this.</summary>
    public const float MaxPieceHeight = 0.15f;

    private readonly List<PhotoView> _views = new();
    private readonly List<float> _angles = new();

    public IReadOnlyList<PhotoView> Views => _views;

    /// <summary>Which 5° steps around the mat have a photo (72 for a full turn).</summary>
    public int CoveredSteps => _angles.Select(a => (int)(a / StepDegrees)).Distinct().Count();

    /// <summary>Poses a photo and keeps it when it adds a new angle. <paramref name="photo"/>'s pose is ignored.</summary>
    /// <param name="force">Keep it even at an angle already covered (a second ring from another height).</param>
    public MatPhotoResult Offer(PhotoView photo, bool force = false)
    {
        var dots = MatDetector.Detect(photo.Image);
        var pose = MatPose.Solve(dots, photo.Intrinsics);
        if (pose is null) return new MatPhotoResult(false, null, null, $"Mat not found ({dots.Count} dots): keep the ring in view");
        if (pose.RmsPixels > MaxRmsPixels) return new MatPhotoResult(false, pose, null, "Blurred: hold the phone still");
        float angle = Angle(pose.CameraToMat);
        bool fresh = force || _angles.All(a => AngleBetween(a, angle) >= StepDegrees);
        if (!fresh) return new MatPhotoResult(false, pose, angle, null);
        _views.Add(photo with { CameraToWorld = pose.CameraToMat });
        _angles.Add(angle);
        return new MatPhotoResult(true, pose, angle, null);
    }

    /// <summary>Removes the last kept photo (a photo taken while the piece was still moving shows a smear).</summary>
    public void RemoveLast()
    {
        if (_views.Count == 0) return;
        _views.RemoveAt(_views.Count - 1);
        _angles.RemoveAt(_angles.Count - 1);
    }

    public VoxelReconstruction? Reconstruct(VoxelReconstructionOptions? options = null, TextWriter? log = null)
    {
        // The mat is the table: a guide disc of points on y = 0 fixes the plane, and the piece stands inside the ring.
        var guide = new List<Vector3>();
        for (float x = -TurntableMat.RingRadius; x <= TurntableMat.RingRadius; x += 0.005f)
        for (float z = -TurntableMat.RingRadius; z <= TurntableMat.RingRadius; z += 0.005f)
            if (x * x + z * z <= TurntableMat.RingRadius * TurntableMat.RingRadius) guide.Add(new Vector3(x, 0, z));
        float r = TurntableMat.FreeRadius;
        options ??= new VoxelReconstructionOptions(ReferenceViews: 12, MaxDepthSamples: 96, MaxHalfWidth: r + 0.02f,
            MinWeight: 1, MaxCropPixels: 384);
        return PhotoVoxelReconstruction.Reconstruct(_views, new Vector3(0, 0.01f, 0), guide, options, log,
            (new Vector3(-r, 0, -r), new Vector3(r, MaxPieceHeight, r)));
    }

    /// <summary>Where the camera stands around the mat, degrees from +X towards +Z.</summary>
    public static float Angle(Matrix4x4 cameraToMat)
    {
        var c = cameraToMat.Translation;
        float degrees = MathF.Atan2(c.Z, c.X) * 180 / MathF.PI;
        return degrees < 0 ? degrees + 360 : degrees;
    }

    private static float AngleBetween(float a, float b)
    {
        float d = MathF.Abs(a - b) % 360;
        return d > 180 ? 360 - d : d;
    }
}
