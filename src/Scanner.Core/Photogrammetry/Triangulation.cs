using System.Numerics;
using Scanner.Capture;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// The 3D point of a track from its observations and the camera poses: the starting points of bundle adjustment,
/// and the test that throws out tracks no single point explains. Poses and points are floats (metres), but the solve
/// runs in doubles: normal equations square their inputs, and in floats that would eat the sub-millimetre detail.
/// </summary>
public static class Triangulation
{
    // As in Pinhole.Project: a point closer than this to the camera plane counts as behind it.
    private const double MinDepth = 1e-6;
    private const int RefinementSteps = 10;

    /// <summary>
    /// The point that best explains every observation. A linear solve (DLT on normalized image coordinates, all
    /// observations at once) gives a start without any initial guess; a few Gauss-Newton steps then minimise the
    /// reprojection error in pixels, which is the error the observations actually carry, rather than the algebraic
    /// error of the linear solve, which weights each photo by the point's depth in it.
    /// </summary>
    /// <returns>Null when the observations cannot give a trustworthy point: the widest angle between two observing
    /// rays is below <paramref name="minAngleDegrees"/> (depth then hangs on a fraction of a pixel), the point lies
    /// behind a camera that saw it, or the RMS reprojection error exceeds <paramref name="maxErrorPx"/> (a wrong match
    /// or a wrong pose).</returns>
    public static Vector3? Triangulate(IReadOnlyList<PhotoView> views, IReadOnlyList<Observation> observations,
        float maxErrorPx = 2f, float minAngleDegrees = 1f)
    {
        ArgumentNullException.ThrowIfNull(views);
        ArgumentNullException.ThrowIfNull(observations);
        int n = observations.Count;
        if (n < 2) return null;

        var sights = new Sight[n];
        for (int i = 0; i < n; i++) sights[i] = new Sight(views[observations[i].View], observations[i]);

        if (!HasAngle(sights, Math.Cos(minAngleDegrees * Math.PI / 180))) return null;
        if (Linear(sights) is not { } start || double.IsPositiveInfinity(SquaredError(sights, start))) return null;

        var point = Refine(sights, start);
        double rms = Math.Sqrt(SquaredError(sights, point) / n);
        if (!(rms <= maxErrorPx)) return null;
        return new Vector3((float)point.X, (float)point.Y, (float)point.Z);
    }

    /// <summary>Pixel distance between the observation and the projection of the point into the view (the
    /// observation's own view index is not consulted); <see cref="float.PositiveInfinity"/> when the point is behind
    /// the camera, where no pixel shows it.</summary>
    public static float ReprojectionError(PhotoView view, Vector3 point, Observation observation)
    {
        ArgumentNullException.ThrowIfNull(view);
        var sight = new Sight(view, observation);
        return sight.Residual(new Vec(point.X, point.Y, point.Z), out double du, out double dv)
            ? (float)Math.Sqrt(du * du + dv * dv)
            : float.PositiveInfinity;
    }

    // True when some pair of observing rays is at least the minimum angle apart (cosine at most cosMin).
    private static bool HasAngle(Sight[] sights, double cosMin)
    {
        var rays = sights.Select(s => s.Ray()).ToArray();
        for (int i = 0; i < rays.Length; i++)
        for (int j = i + 1; j < rays.Length; j++)
            if (Vec.Dot(rays[i], rays[j]) <= cosMin) return true;
        return false;
    }

    /// <summary>
    /// DLT: each observation (x, y) in normalized coordinates of a camera with projection P = [R | t] gives the rows
    /// x·P₃ − P₁ and y·P₃ − P₂, and the homogeneous point is the smallest eigenvector of the 4x4 normal matrix of all
    /// rows. The origin is moved to the mean camera centre and lengths are scaled by the cameras' spread around it, so
    /// the translation column is as large as the rotation columns whatever the session's world origin and units.
    /// </summary>
    private static Vec? Linear(Sight[] sights)
    {
        var origin = new Vec(0, 0, 0);
        foreach (var s in sights) origin += s.Centre;
        origin *= 1.0 / sights.Length;
        double spread = 0;
        foreach (var s in sights) spread += Vec.Dot(s.Centre - origin, s.Centre - origin);
        spread = Math.Sqrt(spread / sights.Length);
        double scale = spread > 1e-9 ? 1 / spread : 1;

        var normal = new double[4, 4];
        foreach (var s in sights)
        {
            var (x, y) = s.Normalized();
            var t = s.Rotate(origin - s.Centre) * scale;
            AddRow(normal, s.ZAxis * x - s.XAxis, x * t.Z - t.X);
            AddRow(normal, s.ZAxis * y - s.YAxis, y * t.Z - t.Y);
        }
        // A non-finite pixel or pose entry spreads through the whole matrix: there is no point to find, and the
        // eigen solver must not see it.
        foreach (double value in normal)
            if (!double.IsFinite(value)) return null;

        var h = SmallestEigenvector(normal);
        // A point at infinity: only possible when the rays are (nearly) parallel.
        if (Math.Abs(h[3]) < 1e-12) return null;
        return origin + new Vec(h[0], h[1], h[2]) * (1 / (h[3] * scale));
    }

    /// <summary>Gauss-Newton on the pixel residuals, keeping only steps that lower the error: from the DLT start it
    /// converges in two or three steps, and a step that does not help means it already has.</summary>
    private static Vec Refine(Sight[] sights, Vec point)
    {
        double error = SquaredError(sights, point);
        var jtj = new double[3, 3];
        var jtr = new double[3];
        for (int step = 0; step < RefinementSteps; step++)
        {
            Array.Clear(jtj);
            Array.Clear(jtr);
            foreach (var s in sights)
            {
                s.Residual(point, out double du, out double dv);
                var c = s.ToCamera(point);
                double iz = 1 / c.Z;
                // d(u, v)/d(world) = d(u, v)/d(camera) · R, with du/dcamera = fx/z (1, 0, -x/z) and likewise for v.
                var ju = (s.XAxis - s.ZAxis * (c.X * iz)) * (s.Fx * iz);
                var jv = (s.YAxis - s.ZAxis * (c.Y * iz)) * (s.Fy * iz);
                AddResidual(jtj, jtr, ju, du);
                AddResidual(jtj, jtr, jv, dv);
            }
            if (!Linear3.TrySolve(jtj, [-jtr[0], -jtr[1], -jtr[2]], out var delta)) break;
            var candidate = point + new Vec(delta[0], delta[1], delta[2]);
            double candidateError = SquaredError(sights, candidate);
            if (!(candidateError < error)) break;
            point = candidate;
            error = candidateError;
            // A nanometre: far below anything a pixel can resolve.
            if (Math.Abs(delta[0]) + Math.Abs(delta[1]) + Math.Abs(delta[2]) < 1e-9) break;
        }
        return point;
    }

    // Adds the DLT row (r, w) to the normal matrix.
    private static void AddRow(double[,] normal, Vec r, double w)
    {
        ReadOnlySpan<double> row = [r.X, r.Y, r.Z, w];
        for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            normal[i, j] += row[i] * row[j];
    }

    // Adds one residual and its gradient to the normal equations JᵀJ and Jᵀr.
    private static void AddResidual(double[,] jtj, double[] jtr, Vec gradient, double residual)
    {
        ReadOnlySpan<double> row = [gradient.X, gradient.Y, gradient.Z];
        for (int i = 0; i < 3; i++)
        {
            jtr[i] += row[i] * residual;
            for (int j = 0; j < 3; j++) jtj[i, j] += row[i] * row[j];
        }
    }

    // Sum of squared pixel residuals; infinite when the point is behind any of the cameras.
    private static double SquaredError(Sight[] sights, Vec point)
    {
        double sum = 0;
        foreach (var s in sights)
        {
            if (!s.Residual(point, out double du, out double dv)) return double.PositiveInfinity;
            sum += du * du + dv * dv;
        }
        return sum;
    }

    /// <summary>Unit eigenvector of the smallest eigenvalue of a symmetric 4x4 matrix, by cyclic Jacobi rotations:
    /// accurate to working precision even when the two smallest eigenvalues are close, as with a short baseline.</summary>
    private static double[] SmallestEigenvector(double[,] a)
    {
        const int n = 4;
        var v = new double[n, n];
        for (int i = 0; i < n; i++) v[i, i] = 1;

        double norm = 0;
        foreach (double value in a) norm += value * value;
        for (int sweep = 0; sweep < 50; sweep++)
        {
            double off = 0;
            for (int p = 0; p < n; p++)
            for (int q = p + 1; q < n; q++)
                off += a[p, q] * a[p, q];
            if (off <= 1e-32 * norm) break;

            for (int p = 0; p < n - 1; p++)
            for (int q = p + 1; q < n; q++)
            {
                if (a[p, q] == 0) continue;
                double theta = (a[q, q] - a[p, p]) / (2 * a[p, q]);
                double t = theta == 0 ? 1 : Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                double c = 1 / Math.Sqrt(t * t + 1);
                Rotate(a, v, p, q, c, t * c);
                // Zero in exact arithmetic; left to rounding it would keep the convergence test from ever passing.
                a[p, q] = a[q, p] = 0;
            }
        }

        int smallest = 0;
        for (int i = 1; i < n; i++)
            if (a[i, i] < a[smallest, smallest]) smallest = i;
        return [v[0, smallest], v[1, smallest], v[2, smallest], v[3, smallest]];
    }

    // A ← Pᵀ·A·P, V ← V·P with P a Jacobi rotation in the (p, q) plane.
    private static void Rotate(double[,] a, double[,] v, int p, int q, double c, double s)
    {
        int n = a.GetLength(0);
        for (int k = 0; k < n; k++)
        {
            double akp = a[k, p], akq = a[k, q];
            a[k, p] = c * akp - s * akq;
            a[k, q] = s * akp + c * akq;
        }
        for (int k = 0; k < n; k++)
        {
            double apk = a[p, k], aqk = a[q, k];
            a[p, k] = c * apk - s * aqk;
            a[q, k] = s * apk + c * aqk;
        }
        for (int k = 0; k < n; k++)
        {
            double vkp = v[k, p], vkq = v[k, q];
            v[k, p] = c * vkp - s * vkq;
            v[k, q] = s * vkp + c * vkq;
        }
    }

    /// <summary>One observation with its camera, in doubles: the camera's axes in world coordinates (the rows of
    /// the world→camera rotation), its centre, its intrinsics and the observed pixel.</summary>
    private readonly struct Sight
    {
        public readonly Vec XAxis, YAxis, ZAxis, Centre;
        public readonly double Fx, Fy;
        private readonly double _cx, _cy, _u, _v;

        public Sight(PhotoView view, Observation observation)
        {
            var m = view.CameraToWorld;
            XAxis = new Vec(m.M11, m.M12, m.M13);
            YAxis = new Vec(m.M21, m.M22, m.M23);
            ZAxis = new Vec(m.M31, m.M32, m.M33);
            Centre = new Vec(m.M41, m.M42, m.M43);
            var k = view.Intrinsics;
            (Fx, Fy, _cx, _cy) = (k.Fx, k.Fy, k.Cx, k.Cy);
            (_u, _v) = (observation.X, observation.Y);
        }

        public Vec Rotate(Vec d) => new(Vec.Dot(XAxis, d), Vec.Dot(YAxis, d), Vec.Dot(ZAxis, d));

        public Vec ToCamera(Vec world) => Rotate(world - Centre);

        /// <summary>The observed pixel in normalized image coordinates (the ray's x/z and y/z in the camera).</summary>
        public (double X, double Y) Normalized() => ((_u - _cx) / Fx, (_v - _cy) / Fy);

        /// <summary>World direction of the ray through the observed pixel, unit length.</summary>
        public Vec Ray()
        {
            var (x, y) = Normalized();
            var d = XAxis * x + YAxis * y + ZAxis;
            return d * (1 / Math.Sqrt(Vec.Dot(d, d)));
        }

        /// <summary>Projection minus observation, in pixels; false when the point is behind the camera.</summary>
        public bool Residual(Vec world, out double du, out double dv)
        {
            var c = ToCamera(world);
            if (!(c.Z > MinDepth))
            {
                du = dv = double.PositiveInfinity;
                return false;
            }
            du = Fx * c.X / c.Z + _cx - _u;
            dv = Fy * c.Y / c.Z + _cy - _v;
            return true;
        }
    }

    private readonly record struct Vec(double X, double Y, double Z)
    {
        public static Vec operator +(Vec a, Vec b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec operator -(Vec a, Vec b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec operator *(Vec a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public static double Dot(Vec a, Vec b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    }
}
