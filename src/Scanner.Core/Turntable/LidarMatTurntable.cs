using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Turntable;

/// <param name="VoxelSize">Resolution of the model (the LiDAR depth is 256–320 pixels wide: about a millimetre a
/// pixel at 25 cm).</param>
/// <param name="TableMargin">Points closer than this to the platter count as platter.</param>
/// <param name="Radius">The piece stands within this distance of the platter's axis; everything outside is ignored
/// (the stand, the room).</param>
/// <param name="MaxHeight">The piece is assumed no taller than this.</param>
/// <param name="MatchDistance">Farthest a frame's point may sit from the model and still be matched (ICP).</param>
/// <param name="Stride">Every how many depth pixels a point is taken for ICP and for the model.</param>
/// <param name="MinInlierFraction">An ICP pose with fewer matched points than this share is not trusted.</param>
/// <param name="MaxRms">An ICP pose whose matched points sit further than this (RMS) from the model is not trusted.</param>
/// <param name="MaxMatRms">A mat pose whose dots fit worse than this (pixels) is not trusted.</param>
/// <param name="MaxPlatterOffset">The platter, measured by the depth in the mat pose, must lie this close to y = 0:
/// a misread ring or a depth not registered to the photo shows up here.</param>
public sealed record LidarTurntableOptions(float VoxelSize = 0.0015f, float TableMargin = 0.004f, float Radius = 0.1f,
    float MaxHeight = 0.2f, float MatchDistance = 0.008f, int Stride = 2, float MinInlierFraction = 0.5f,
    float MaxRms = 0.002f, float MaxMatRms = MatTurntableScan.MaxRmsPixels, float MaxPlatterOffset = 0.006f);

/// <summary>Where a frame's pose came from.</summary>
public enum TurntablePoseSource
{
    /// <summary>The dot ring on the platter, seen in the photo: absolute, no drift.</summary>
    Mat,

    /// <summary>ICP of the depth against the piece fused so far, while the ring is hidden.</summary>
    Icp,
}

/// <summary>What happened to one frame.</summary>
/// <param name="CameraToMat">The (still) camera in the (turning) mat's frame, OpenCV axes; set when accepted.</param>
/// <param name="AngleDegrees">Where around the platter the camera looks from (0–360), when posed.</param>
public readonly record struct LidarTurntableFrame(bool Accepted, Matrix4x4? CameraToMat, TurntablePoseSource? Source,
    float? AngleDegrees, string? Reason);

/// <summary>
/// Turntable scan with a still depth camera looking at the printed dot ring (the iPhone's rear camera and LiDAR on a
/// stand, the piece turning on the motorised platter). Each frame is posed from the ring in its photo
/// (<see cref="MatDetector"/>, <see cref="MatPose"/>): the mat frame is the piece's own (y = 0 the platter, +Y up, true
/// scale), so the depth goes straight into a TSDF there, the platter and everything off it cut away. When the ring
/// is hidden (the piece in front of too many dots, a blurred photo), ICP of the depth against the piece fused so far
/// carries the pose on from the last one, predicted by the turn rate; the next frame that sees the ring snaps it back.
/// </summary>
public sealed class LidarMatTurntable
{
    private readonly LidarTurntableOptions _options;
    private readonly TsdfVolume _volume;
    private readonly PointModel _model;
    private readonly HashSet<int> _steps = new();
    private Matrix4x4? _last;
    private double _lastTime;
    private float _radiansPerSecond;

    public LidarMatTurntable(LidarTurntableOptions? options = null)
    {
        _options = options ?? new LidarTurntableOptions();
        _volume = new TsdfVolume(_options.VoxelSize, 4 * _options.VoxelSize);
        _model = new PointModel(_options.VoxelSize);
    }

    public int AcceptedFrames => MatFrames + IcpFrames;
    public int MatFrames { get; private set; }
    public int IcpFrames { get; private set; }

    /// <summary>Which 5° steps around the platter have been seen (72 for a full turn).</summary>
    public int CoveredSteps => _steps.Count;

    /// <summary>The degrees of a turn covered so far.</summary>
    public float CoveredDegrees => CoveredSteps * MatTurntableScan.StepDegrees;

    /// <summary>
    /// Poses and fuses one frame: <paramref name="depth"/> in camera coordinates (its own pose is ignored) and
    /// <paramref name="photo"/>, the same camera's image at the same instant (its pose is ignored; null when there is
    /// none). The depth and the photo may differ in size, not in field of view.
    /// </summary>
    public LidarTurntableFrame Add(DepthFrame depth, PhotoView? photo)
    {
        string? matProblem = null;
        if (photo is not null)
        {
            var dots = MatDetector.Detect(photo.Image);
            var pose = MatPose.Solve(dots, photo.Intrinsics);
            if (pose is null) matProblem = $"Dot ring not found ({dots.Count} dots)";
            else if (pose.RmsPixels > _options.MaxMatRms) matProblem = "Dot ring blurred";
            else if (PlatterOffset(depth, pose.CameraToMat) is { } offset && MathF.Abs(offset) > _options.MaxPlatterOffset)
                matProblem = $"Depth and ring disagree by {offset * 1000:F0} mm";
            else return Fuse(depth, pose.CameraToMat, TurntablePoseSource.Mat);
        }
        else matProblem = "No photo";

        if (_last is not { } last)
            return new LidarTurntableFrame(false, null, null, null, $"{matProblem}: get the whole dot ring in view");

        // Still camera, platter turning about its own Y axis: the next pose is the last one turned on by the rate.
        float predictedTurn = (float)(_radiansPerSecond * (depth.TimestampSeconds - _lastTime));
        var predicted = last * Matrix4x4.CreateRotationY(predictedTurn);
        var (points, normals) = Points(depth, predicted, _options.Stride);
        if (points.Count < 50)
            return new LidarTurntableFrame(false, null, null, null, $"{matProblem}, and the piece is not in view");
        var alignment = RigidAligner.Align(points, normals, _model, predicted, _options.MatchDistance);
        if (alignment.InlierFraction < _options.MinInlierFraction || alignment.RmsError > _options.MaxRms)
            return new LidarTurntableFrame(false, null, null, null, $"{matProblem}, and lost the piece: turn slower");
        return Fuse(depth, alignment.Transform, TurntablePoseSource.Icp);
    }

    /// <summary>The fused piece in the mat frame, standing on y = 0.</summary>
    public TriangleMesh Mesh(float minWeight = 2)
    {
        var mesh = SurfaceNets.Extract(_volume, minWeight);
        var platter = new FittedSupportPlane(0, 0, 0, _options.TableMargin / 2);
        var piece = MeshCleanup.Piece(mesh, new Vector3(0, 0.01f, 0), platter, linkDistance: 2 * _options.VoxelSize);
        // The band cut away as platter is missing from the mesh: its lowest vertices come down onto the platter.
        MeshCleanup.FlattenOnto(piece, platter, band: _options.TableMargin + 2 * _options.VoxelSize);
        return piece;
    }

    private LidarTurntableFrame Fuse(DepthFrame depth, Matrix4x4 cameraToMat, TurntablePoseSource source)
    {
        if (_last is { } last && depth.TimestampSeconds > _lastTime)
        {
            Matrix4x4.Invert(last, out var inverse);
            float turn = TurnAboutY(inverse * cameraToMat);
            _radiansPerSecond = (float)(turn / (depth.TimestampSeconds - _lastTime));
        }

        var (points, normals) = Points(depth, cameraToMat, _options.Stride);
        for (int i = 0; i < points.Count; i++)
            _model.Add(Vector3.Transform(points[i], cameraToMat),
                Vector3.Normalize(Vector3.TransformNormal(normals[i], cameraToMat)));
        _volume.Integrate(Masked(depth, cameraToMat) with { CameraToWorld = cameraToMat });

        _last = cameraToMat;
        _lastTime = depth.TimestampSeconds;
        if (source == TurntablePoseSource.Mat) MatFrames++;
        else IcpFrames++;
        float angle = MatTurntableScan.Angle(cameraToMat);
        _steps.Add((int)(angle / MatTurntableScan.StepDegrees) % (int)(360 / MatTurntableScan.StepDegrees));
        return new LidarTurntableFrame(true, cameraToMat, source, angle, null);
    }

    /// <summary>The angle of a rotation about +Y (System.Numerics' sense), radians.</summary>
    public static float TurnAboutY(Matrix4x4 m) => MathF.Atan2(m.M31, m.M11);

    /// <summary>
    /// The median height, in the mat frame, of the depth that falls on the platter's dot ring band (outside where the
    /// piece may stand): zero when the depth and the pose agree. Null when the band is not seen.
    /// </summary>
    private float? PlatterOffset(DepthFrame depth, Matrix4x4 cameraToMat)
    {
        float inner = TurntableMat.FreeRadius, outer = TurntableMat.RingRadius + 0.015f;
        var heights = new List<float>();
        var k = depth.Intrinsics;
        for (int v = 0; v < k.Height; v += 2)
        for (int u = 0; u < k.Width; u += 2)
        {
            var c = BackProject(depth, u, v);
            if (c.Z <= 0) continue;
            var p = Vector3.Transform(c, cameraToMat);
            float r2 = p.X * p.X + p.Z * p.Z;
            if (r2 < inner * inner || r2 > outer * outer || MathF.Abs(p.Y) > 0.03f) continue;
            heights.Add(p.Y);
        }
        if (heights.Count < 30) return null;
        heights.Sort();
        return heights[heights.Count / 2];
    }

    /// <summary>The piece's points of a frame with their normals (from the neighbouring pixels), in camera coordinates.</summary>
    private (List<Vector3> Points, List<Vector3> Normals) Points(DepthFrame frame, Matrix4x4 cameraToMat, int stride)
    {
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        var k = frame.Intrinsics;
        for (int v = stride; v + stride < k.Height; v += stride)
        for (int u = stride; u + stride < k.Width; u += stride)
        {
            if (!IsPiece(frame, cameraToMat, u, v, out var p)) continue;
            // Neighbours on the piece only: across its silhouette the depth jumps and the normal would be meaningless.
            if (!IsPiece(frame, cameraToMat, u + stride, v, out var right)
                || !IsPiece(frame, cameraToMat, u, v + stride, out var down)) continue;
            var n = Vector3.Cross(down - p, right - p);
            if (n.LengthSquared() == 0) continue;
            n = Vector3.Normalize(n);
            if (Vector3.Dot(n, p) > 0) n = -n; // towards the camera
            points.Add(p);
            normals.Add(n);
        }
        return (points, normals);
    }

    /// <summary>The frame with every pixel that is not the piece cleared, for the TSDF.</summary>
    private DepthFrame Masked(DepthFrame frame, Matrix4x4 cameraToMat)
    {
        var depth = (float[])frame.Depth.Clone();
        var k = frame.Intrinsics;
        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
            if (!IsPiece(frame, cameraToMat, u, v, out _)) depth[v * k.Width + u] = 0;
        return frame with { Depth = depth };
    }

    /// <summary>On the piece: above the platter, below the height limit, within the radius of the axis.</summary>
    private bool IsPiece(DepthFrame frame, Matrix4x4 cameraToMat, int u, int v, out Vector3 point)
    {
        point = BackProject(frame, u, v);
        if (point.Z <= 0) return false;
        var p = Vector3.Transform(point, cameraToMat);
        return p.Y > _options.TableMargin && p.Y < _options.MaxHeight
               && p.X * p.X + p.Z * p.Z < _options.Radius * _options.Radius;
    }

    private static Vector3 BackProject(DepthFrame frame, int u, int v)
    {
        var k = frame.Intrinsics;
        float d = frame.DepthAt(u, v);
        return d <= 0 ? Vector3.Zero : new Vector3((u - k.Cx) / k.Fx * d, (v - k.Cy) / k.Fy * d, d);
    }
}
