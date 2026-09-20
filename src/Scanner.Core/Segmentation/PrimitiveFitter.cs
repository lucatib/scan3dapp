using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

public static class PrimitiveFitter
{
    public static Primitive? Refine(Primitive shape, PointCloud cloud, IReadOnlyList<int> indices) => shape switch
    {
        PlanePrimitive plane => FitPlane(cloud, indices, plane.Normal),
        CylinderPrimitive cylinder => FitCylinder(cloud, indices, cylinder.Axis),
        _ => null,
    };

    /// <summary>Least-squares plane (minimum eigenvector of the covariance), oriented like <paramref name="orientation"/>.</summary>
    public static PlanePrimitive? FitPlane(PointCloud cloud, IReadOnlyList<int> indices, Vector3 orientation)
    {
        if (indices.Count < 3) return null;
        var centroid = Centroid(cloud.Points, indices);
        var covariance = new double[3, 3];
        foreach (int i in indices) AddOuter(covariance, cloud.Points[i] - centroid);

        var normal = SymmetricEigen3.Solve(covariance).Vectors[0];
        if (Vector3.Dot(normal, orientation) < 0) normal = -normal;
        return new PlanePrimitive(normal, Vector3.Dot(normal, centroid));
    }

    /// <summary>Axis = direction orthogonal to all normals; cross-section = Kåsa circle fit on the projected points.</summary>
    public static CylinderPrimitive? FitCylinder(PointCloud cloud, IReadOnlyList<int> indices, Vector3 axisHint)
    {
        if (indices.Count < 6) return null;
        var normalMoments = new double[3, 3];
        foreach (int i in indices) AddOuter(normalMoments, cloud.Normals[i]);
        var (values, vectors) = SymmetricEigen3.Solve(normalMoments);
        // A cylinder's normals sweep the plane orthogonal to the axis, so the moment matrix is rank 2: the
        // smallest eigenvalue (along the axis) is far below the second, and the second is comparable to the
        // largest. A planar inlier set is rank 1 instead and Vectors[0] is an arbitrary direction. Rank 1 has
        // to be excluded on both counts, because either condition alone has a blind spot.
        //
        // The gap test rejects a plane whose normal error is isotropic, where the two smallest eigenvalues come
        // out equal. Measured l0/l1: 0.92 for a patch with isotropic 3 deg normal jitter, against 2.5e-3 for a
        // genuine cylinder with 5 deg jitter and 9.9e-3 at 10 deg.
        //
        // The scale test rejects what the gap test cannot. Anisotropic normal error - routine when normals come
        // from a depth map, where the error along the scan lines differs from the error across them - shrinks
        // l0 without making the patch any less flat, so l0/l1 slides under the gap threshold (measured 0.067 at
        // 4:1) while the patch is still rank 1. It is also the sign-safe half: at exact degeneracy Jacobi
        // returns l0 and l1 at +-1e-13 with arbitrary signs, and "negative >= positive" would accept the set.
        // For normals spread over an arc of width w, l1/l2 is about w^2/12, so 1e-2 rejects anything sweeping
        // less than ~20 deg; a flat patch measures 4.7e-4, while a cylinder wide enough to clear the detector's
        // 90 deg coverage gate measures 0.23 - a 23x margin - and a full circle measures 1.0.
        if (values[0] >= 0.1 * values[1] || values[1] <= 1e-2 * values[2]) return null;
        var axis = vectors[0];
        if (Vector3.Dot(axis, axisHint) < 0) axis = -axis;

        var (u, v) = Basis.Orthonormal(axis);
        var centroid = Centroid(cloud.Points, indices);
        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
        foreach (int i in indices)
        {
            var d = cloud.Points[i] - centroid;
            double x = Vector3.Dot(d, u), y = Vector3.Dot(d, v), z = x * x + y * y;
            sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
            sxz += x * z; syz += y * z; sz += z;
        }
        var a = new double[,] { { sxx, sxy, sx }, { sxy, syy, sy }, { sx, sy, indices.Count } };
        if (!Linear3.TrySolve(a, new[] { -sxz, -syz, -sz }, out var s)) return null;

        double cx = -s[0] / 2, cy = -s[1] / 2, r2 = cx * cx + cy * cy - s[2];
        if (r2 <= 0) return null;

        var cylinder = new CylinderPrimitive(centroid + u * (float)cx + v * (float)cy, axis, (float)Math.Sqrt(r2), IsHole: false);
        int inward = indices.Count(i => Vector3.Dot(cloud.Normals[i], cylinder.RadialVector(cloud.Points[i])) < 0);
        return cylinder with { IsHole = inward * 2 > indices.Count };
    }

    private static Vector3 Centroid(Vector3[] points, IReadOnlyList<int> indices)
    {
        var sum = Vector3.Zero;
        foreach (int i in indices) sum += points[i];
        return sum / indices.Count;
    }

    private static void AddOuter(double[,] m, Vector3 d)
    {
        double[] a = { d.X, d.Y, d.Z };
        for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m[r, c] += a[r] * a[c];
    }
}
