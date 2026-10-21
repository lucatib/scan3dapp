using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Meshing;

namespace Scanner.Core.Turntable;

/// <param name="Radius">How far from the target the piece reaches; everything else in the depth image is ignored.</param>
/// <param name="VoxelSize">Resolution of the model (TrueDepth resolves well under a millimetre at 25 cm).</param>
/// <param name="MatchDistance">Farthest a frame's point may sit from the model and still be matched to it.</param>
/// <param name="TableMargin">Points closer than this to the table count as table.</param>
/// <param name="Stride">Every how many pixels a point is taken for alignment (the TSDF takes every pixel).</param>
/// <param name="MinInlierFraction">Below this share of matched points a frame is not trusted, and is skipped.</param>
/// <param name="MaxRms">A frame whose matched points sit further than this (RMS) from the model is skipped.</param>
public sealed record TurntableOptions(float Radius = 0.12f, float VoxelSize = 0.001f, float MatchDistance = 0.006f,
    float TableMargin = 0.004f, int Stride = 2, float MinInlierFraction = 0.6f, float MaxRms = 0.0015f);

/// <summary>What happened to one depth frame.</summary>
/// <param name="CameraToObject">Where the (still) camera is in the frame of the (turning) piece; set when accepted.</param>
/// <param name="TurnDegrees">How far the piece has turned since the first frame, about the table's normal.</param>
public readonly record struct TurntableFrame(bool Accepted, Matrix4x4? CameraToObject, float TurnDegrees, Alignment? Alignment,
    string? Reason);

/// <summary>
/// Scans a piece that turns in front of a still depth camera (the phone on a stand, its front TrueDepth camera facing
/// a turntable). The table does not turn with the piece, so it is cut away first: everything near or behind the
/// table plane found in the first frame, and everything further than <see cref="TurntableOptions.Radius"/> from the
/// target. What is left turns rigidly, and ICP against the points fused so far gives each frame's pose in the piece's
/// own frame; a TSDF fuses the depth there. The piece's frame is the camera's at the first frame.
/// </summary>
public sealed class TurntableFusion
{
    private readonly TurntableOptions _options;
    private readonly TsdfVolume _volume;
    private readonly PointModel _model;
    private Matrix4x4 _previous = Matrix4x4.Identity;
    private Matrix4x4 _motion = Matrix4x4.Identity;
    private float _turn;

    /// <param name="target">The point picked on the piece, in camera coordinates (OpenCV axes, metres).</param>
    public TurntableFusion(Vector3 target, TurntableOptions? options = null)
    {
        Target = target;
        _options = options ?? new TurntableOptions();
        _volume = new TsdfVolume(_options.VoxelSize, 4 * _options.VoxelSize);
        _model = new PointModel(_options.VoxelSize);
    }

    public Vector3 Target { get; }

    /// <summary>The table, in camera (= piece) coordinates at the first frame: unit normal towards the camera and a
    /// point on it. Null before the first frame.</summary>
    public (Vector3 Normal, Vector3 Point)? Table { get; private set; }

    public int AcceptedFrames { get; private set; }

    /// <summary>The accumulated turn in degrees (one full loop is 360).</summary>
    public float TurnDegrees => _turn;

    public PointModel Model => _model;

    /// <summary>Aligns and fuses one depth frame (its own pose is ignored: the camera does not move).</summary>
    public TurntableFrame Add(DepthFrame frame)
    {
        if (Table is null)
        {
            Table = FitTable(frame);
            if (Table is null) return new TurntableFrame(false, null, 0, null, "No table under the piece");
        }

        var (points, normals) = Points(frame, _options.Stride);
        if (points.Count < 50) return new TurntableFrame(false, null, _turn, null, "The piece is not in view");

        Matrix4x4 pose;
        Alignment? alignment = null;
        if (AcceptedFrames == 0) pose = Matrix4x4.Identity;
        else
        {
            // Constant turn: the last frame's motion again is the best first guess.
            var result = RigidAligner.Align(points, normals, _model, _previous * _motion, _options.MatchDistance);
            alignment = result;
            if (result.InlierFraction < _options.MinInlierFraction || result.RmsError > _options.MaxRms)
                return new TurntableFrame(false, null, _turn, result, "Lost the piece: turn slower");
            pose = result.Transform;
            Matrix4x4.Invert(_previous, out var inverse);
            _motion = inverse * pose;
        }

        for (int i = 0; i < points.Count; i++)
            _model.Add(Vector3.Transform(points[i], pose), Vector3.Normalize(Vector3.TransformNormal(normals[i], pose)));
        _volume.Integrate(Masked(frame) with { CameraToWorld = pose });
        if (AcceptedFrames > 0) _turn += TurnBetween(_previous, pose);
        _previous = pose;
        AcceptedFrames++;
        return new TurntableFrame(true, pose, _turn, alignment, null);
    }

    /// <summary>Maps the piece's frame to an upright one: +Y along the table normal, the table at y = 0, the origin
    /// under the target. Photos and the mesh are both handed out in it.</summary>
    public Matrix4x4 Upright()
    {
        var (n, point) = Table ?? throw new InvalidOperationException("No frame has been added yet.");
        var origin = Target - Vector3.Dot(Target - point, n) * n;
        var (u, _) = Basis.Orthonormal(n);
        var z = Vector3.Cross(u, n);
        var rotation = new Matrix4x4(
            u.X, n.X, z.X, 0,
            u.Y, n.Y, z.Y, 0,
            u.Z, n.Z, z.Z, 0,
            0, 0, 0, 1);
        return Matrix4x4.CreateTranslation(-origin) * rotation;
    }

    /// <summary>The fused piece in the upright frame, standing on y = 0, open nowhere the camera saw.</summary>
    public TriangleMesh Mesh(float minWeight = 2)
    {
        var mesh = SurfaceNets.Extract(_volume, minWeight);
        var upright = Upright();
        for (int i = 0; i < mesh.Positions.Count; i++)
        {
            mesh.Positions[i] = Vector3.Transform(mesh.Positions[i], upright);
            mesh.Normals[i] = Vector3.Normalize(Vector3.TransformNormal(mesh.Normals[i], upright));
        }
        var table = new FittedSupportPlane(0, 0, 0, _options.TableMargin / 2);
        var piece = MeshCleanup.Piece(mesh, Vector3.Transform(Target, upright), table, linkDistance: 2 * _options.VoxelSize);
        // The band cut away as table is missing from the mesh: its lowest vertices come down onto the table.
        MeshCleanup.FlattenOnto(piece, table, band: _options.TableMargin + 2 * _options.VoxelSize);
        return piece;
    }

    /// <summary>The camera of a frame in the upright frame, for texturing.</summary>
    public Matrix4x4 CameraToUpright(Matrix4x4 cameraToObject) => cameraToObject * Upright();

    /// <summary>The piece's points of a frame with their normals (from the neighbouring pixels), in camera coordinates.</summary>
    private (List<Vector3> Points, List<Vector3> Normals) Points(DepthFrame frame, int stride)
    {
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        var k = frame.Intrinsics;
        for (int v = stride; v + stride < k.Height; v += stride)
        for (int u = stride; u + stride < k.Width; u += stride)
        {
            if (!IsPiece(frame, u, v, out var p)) continue;
            // Neighbours on the piece only: across its silhouette the depth jumps and the normal would be meaningless.
            if (!IsPiece(frame, u + stride, v, out var right) || !IsPiece(frame, u, v + stride, out var down)) continue;
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
    private DepthFrame Masked(DepthFrame frame)
    {
        var depth = (float[])frame.Depth.Clone();
        var k = frame.Intrinsics;
        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
            if (!IsPiece(frame, u, v, out _)) depth[v * k.Width + u] = 0;
        return frame with { Depth = depth };
    }

    private bool IsPiece(DepthFrame frame, int u, int v, out Vector3 point)
    {
        point = BackProject(frame, u, v);
        if (point.Z <= 0 || Vector3.DistanceSquared(point, Target) > _options.Radius * _options.Radius) return false;
        var (n, onTable) = Table!.Value;
        return Vector3.Dot(point - onTable, n) > _options.TableMargin;
    }

    private static Vector3 BackProject(DepthFrame frame, int u, int v)
    {
        var k = frame.Intrinsics;
        float d = frame.DepthAt(u, v);
        return d <= 0 ? Vector3.Zero : new Vector3((u - k.Cx) / k.Fx * d, (v - k.Cy) / k.Fy * d, d);
    }

    /// <summary>
    /// The table: the plane with the most points around the target that has (almost) nothing behind it. A face of the
    /// piece can hold more points, but the table shows past its edges, behind it; nothing shows behind the table.
    /// </summary>
    private (Vector3 Normal, Vector3 Point)? FitTable(DepthFrame frame)
    {
        var points = new List<Vector3>();
        var k = frame.Intrinsics;
        for (int v = 0; v < k.Height; v += 2)
        for (int u = 0; u < k.Width; u += 2)
        {
            var p = BackProject(frame, u, v);
            if (p.Z > 0 && Vector3.DistanceSquared(p, Target) <= _options.Radius * _options.Radius) points.Add(p);
        }
        if (points.Count < 100) return null;

        const float inlier = 0.003f;
        var rng = new Random(1);
        (Vector3 Normal, Vector3 Point)? best = null;
        int bestCount = 0;
        for (int iteration = 0; iteration < 300; iteration++)
        {
            var a = points[rng.Next(points.Count)];
            var n = Vector3.Cross(points[rng.Next(points.Count)] - a, points[rng.Next(points.Count)] - a);
            if (n.LengthSquared() < 1e-10f) continue;
            n = Vector3.Normalize(n);
            if (Vector3.Dot(n, a) > 0) n = -n; // towards the camera, at the origin
            if (Vector3.Dot(Target - a, n) < _options.TableMargin) continue; // the piece stands on the table
            int count = 0, behind = 0;
            foreach (var p in points)
            {
                float d = Vector3.Dot(p - a, n);
                if (MathF.Abs(d) <= inlier) count++;
                else if (d < -inlier) behind++;
            }
            if (behind > points.Count / 20 || count <= bestCount) continue;
            bestCount = count;
            best = (n, a);
        }
        if (best is not { } plane) return null;

        // Least squares over the inliers: the smallest axis of their spread is the normal.
        var inliers = points.Where(p => MathF.Abs(Vector3.Dot(p - plane.Point, plane.Normal)) <= inlier).ToArray();
        var centre = inliers.Aggregate(Vector3.Zero, (s, p) => s + p) / inliers.Length;
        var covariance = new double[3, 3];
        foreach (var p in inliers)
        {
            var d = p - centre;
            Span<float> c = [d.X, d.Y, d.Z];
            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) covariance[i, j] += c[i] * c[j];
        }
        var normal = SymmetricEigen3.Solve(covariance).Vectors[0];
        if (Vector3.Dot(normal, plane.Normal) < 0) normal = -normal;
        return (normal, centre);
    }

    /// <summary>The turn from one pose to the next about the table normal, in degrees.</summary>
    private float TurnBetween(Matrix4x4 from, Matrix4x4 to)
    {
        Matrix4x4.Invert(from, out var inverse);
        var step = inverse * to;
        var axis = Table!.Value.Normal;
        var (u, _) = Basis.Orthonormal(axis);
        var turned = Vector3.TransformNormal(u, step);
        turned -= Vector3.Dot(turned, axis) * axis;
        float angle = MathF.Atan2(Vector3.Dot(Vector3.Cross(u, turned), axis), Vector3.Dot(u, turned));
        return MathF.Abs(angle) * 180 / MathF.PI;
    }
}
