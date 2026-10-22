using System.Numerics;
using Scanner.Capture;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Turntable;

/// <summary>A photo's pose on the mat and how well the dots fit it.</summary>
/// <param name="CameraToMat">Camera→mat (OpenCV camera axes, row-vector convention): the photo's camera in the mat's frame.</param>
/// <param name="RmsPixels">RMS reprojection error of the dots used.</param>
public sealed record MatPoseResult(Matrix4x4 CameraToMat, float RmsPixels, int Dots);

/// <summary>
/// The camera pose from the mat dots in a photo: a homography from the printed dot positions to their pixels gives
/// the first guess, and Gauss-Newton on the reprojection error refines it.
/// </summary>
public static class MatPose
{
    /// <summary>Fewest identified dots that give a trustworthy pose (a homography needs four).</summary>
    public const int MinDots = 8;

    public static MatPoseResult? Solve(IReadOnlyList<MatDot> dots, CameraIntrinsics k)
    {
        if (dots.Count < MinDots) return null;
        var world = dots.Select(d => TurntableMat.DotPosition(d.Index)).ToArray();
        var pixels = dots.Select(d => d.Pixel).ToArray();
        if (Initial(world, pixels, k) is not var (rotation, translation)) return null;

        double previous = double.PositiveInfinity, rms = 0;
        for (int iteration = 0; iteration < 20; iteration++)
        {
            var normal = new double[6, 6];
            var rhs = new double[6];
            double squared = 0;
            for (int i = 0; i < world.Length; i++)
            {
                var c = Rotate(rotation, world[i]) + translation;
                if (c.Z <= 0) return null;
                float u = k.Fx * c.X / c.Z + k.Cx, v = k.Fy * c.Y / c.Z + k.Cy;
                float ru = u - pixels[i].X, rv = v - pixels[i].Y;
                squared += ru * ru + rv * rv;
                // d(u, v)/dc, and dc/d(ω, t) = [-[c - t]×, I] for a rotation update applied on the left.
                float iz = 1 / c.Z;
                var du = new Vector3(k.Fx * iz, 0, -k.Fx * c.X * iz * iz);
                var dv = new Vector3(0, k.Fy * iz, -k.Fy * c.Y * iz * iz);
                var rotated = c - translation;
                Span<double> ju = [.. Row(du, rotated)];
                Span<double> jv = [.. Row(dv, rotated)];
                for (int a = 0; a < 6; a++)
                {
                    rhs[a] -= ju[a] * ru + jv[a] * rv;
                    for (int b = 0; b <= a; b++) normal[a, b] += ju[a] * ju[b] + jv[a] * jv[b];
                }
            }
            rms = Math.Sqrt(squared / world.Length);
            if (previous - rms < 1e-6) break;
            previous = rms;
            double[] x;
            try
            {
                x = DenseCholesky.Solve(normal, rhs);
            }
            catch (ArgumentException)
            {
                break;
            }
            var omega = new Vector3((float)x[0], (float)x[1], (float)x[2]);
            if (omega.LengthSquared() > 0)
                rotation = Matrix4x4.CreateFromAxisAngle(Vector3.Normalize(omega), omega.Length()) is var step
                    ? Compose(step, rotation) : rotation;
            translation += new Vector3((float)x[3], (float)x[4], (float)x[5]);
        }
        return new MatPoseResult(CameraToMat(rotation, translation), (float)rms, world.Length);
    }

    /// <summary>The camera's look direction (OpenCV +Z) and position from the pose.</summary>
    public static Vector3 CameraCentre(Matrix4x4 cameraToMat) => cameraToMat.Translation;

    /// <summary>The Jacobian row of one pixel coordinate: its gradient <paramref name="g"/> in camera coordinates,
    /// through a left rotation update (dc/dω = −[r]×, so g·(ω × r) = ω·(r × g)) and the translation.</summary>
    private static double[] Row(Vector3 g, Vector3 r)
    {
        var w = Vector3.Cross(r, g);
        return [w.X, w.Y, w.Z, g.X, g.Y, g.Z];
    }

    /// <summary>Rotation as a column-vector 3×3 in a Matrix4x4 (M[row][col]): c = R·p.</summary>
    private static Vector3 Rotate(Matrix4x4 r, Vector3 p) => new(
        r.M11 * p.X + r.M12 * p.Y + r.M13 * p.Z,
        r.M21 * p.X + r.M22 * p.Y + r.M23 * p.Z,
        r.M31 * p.X + r.M32 * p.Y + r.M33 * p.Z);

    /// <summary>step ∘ rotation for column-vector rotations, where <paramref name="step"/> comes from System.Numerics
    /// (row-vector, so its column-vector form is its transpose).</summary>
    private static Matrix4x4 Compose(Matrix4x4 rowVectorStep, Matrix4x4 rotation) =>
        Matrix4x4.Transpose(Matrix4x4.Transpose(rotation) * rowVectorStep);

    /// <summary>The row-vector camera→mat matrix for c = R·p + t: p = Rᵀ(c − t).</summary>
    private static Matrix4x4 CameraToMat(Matrix4x4 r, Vector3 t)
    {
        var m = new Matrix4x4(
            r.M11, r.M12, r.M13, 0,
            r.M21, r.M22, r.M23, 0,
            r.M31, r.M32, r.M33, 0,
            0, 0, 0, 1);
        m.Translation = -Vector3.Transform(t, m);
        return m;
    }

    /// <summary>
    /// The pose from the plane homography: the dots lie on y = 0, so a pixel is K·(x·R₁ + z·R₃ + t) up to scale, and
    /// H = K⁻¹·(that homography) holds R₁, R₃ and t in its columns.
    /// </summary>
    private static (Matrix4x4 Rotation, Vector3 Translation)? Initial(Vector3[] world, Vector2[] pixels, CameraIntrinsics k)
    {
        // Normalized image coordinates, so the homography is K⁻¹·H directly. h33 = 1: the mat centre is in front of the camera.
        var normal = new double[8, 8];
        var rhs = new double[8];
        for (int i = 0; i < world.Length; i++)
        {
            double x = world[i].X, z = world[i].Z;
            double u = (pixels[i].X - k.Cx) / k.Fx, v = (pixels[i].Y - k.Cy) / k.Fy;
            Span<double> a = [x, z, 1, 0, 0, 0, -u * x, -u * z];
            Span<double> b = [0, 0, 0, x, z, 1, -v * x, -v * z];
            for (int p = 0; p < 8; p++)
            {
                rhs[p] += a[p] * u + b[p] * v;
                for (int q = 0; q <= p; q++) normal[p, q] += a[p] * a[q] + b[p] * b[q];
            }
        }
        double[] h;
        try
        {
            h = DenseCholesky.Solve(normal, rhs);
        }
        catch (ArgumentException)
        {
            return null;
        }
        var c1 = new Vector3((float)h[0], (float)h[3], (float)h[6]);
        var c2 = new Vector3((float)h[1], (float)h[4], (float)h[7]);
        var c3 = new Vector3((float)h[2], (float)h[5], 1);
        float scale = 2 / (c1.Length() + c2.Length());
        if (c3.Z * scale < 0) scale = -scale; // the mat is in front of the camera
        var r1 = Vector3.Normalize(c1 * scale);
        var r3 = Vector3.Normalize(c2 * scale - Vector3.Dot(c2 * scale, r1) * r1);
        var r2 = Vector3.Cross(r3, r1); // e_z × e_x = e_y
        var rotation = new Matrix4x4(
            r1.X, r2.X, r3.X, 0,
            r1.Y, r2.Y, r3.Y, 0,
            r1.Z, r2.Z, r3.Z, 0,
            0, 0, 0, 1);
        return (rotation, c3 * scale);
    }
}
