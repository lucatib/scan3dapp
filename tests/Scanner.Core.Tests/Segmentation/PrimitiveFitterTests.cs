using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Segmentation;

namespace Scanner.Core.Tests.Segmentation;

public class PrimitiveFitterTests
{
    /// <summary>
    /// A planar inlier set has (near-)parallel normals, so the normals' second-moment matrix is effectively rank 1
    /// and its two smallest eigenvalues are degenerate: every direction in the plane is an equally valid "axis".
    /// The fit must refuse to invent one instead of returning an arbitrary axis with a bogus radius.
    /// </summary>
    [Fact]
    public void FitCylinder_returns_null_for_a_planar_inlier_set()
    {
        var cloud = PlanarPatch(normalNoiseDegrees: 3f, anisotropy: 1f, depthNoise: 0.0005f, seed: 11);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>
    /// The same patch with mathematically exact points and normals must be refused too. This one documents
    /// intent rather than isolating the gate: the bogus in-plane axis also collapses the Kåsa system, which
    /// <see cref="Linear3.TrySolve"/> rejects on its own. The eigenvalue assertion pins down which condition
    /// of the gate is supposed to fire.
    /// </summary>
    [Fact]
    public void FitCylinder_returns_null_for_an_exactly_planar_inlier_set()
    {
        var cloud = PlanarPatch(normalNoiseDegrees: 0f, anisotropy: 1f, depthNoise: 0f, seed: 11);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        var values = NormalMomentEigenvalues(cloud, indices);
        Assert.True(values[1] <= 1e-2 * values[2], $"lambda1/lambda2 = {values[1] / values[2]:E3} must be negligible");
        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>
    /// A perfectly flat patch whose normal is not axis-aligned makes Jacobi actually rotate, and the two
    /// smallest eigenvalues come back at ±1e-13 with arbitrary signs. Comparing them as lambda0 >= 0.1 * lambda1
    /// is then a coin toss - "negative >= positive" is false and the degenerate set would be accepted with an
    /// invented axis - so rejection has to rest on lambda1 being negligible against lambda2 instead.
    /// </summary>
    [Theory]
    [InlineData(1f, 2f, 3f)]
    [InlineData(3f, 1f, 2f)]
    [InlineData(2f, 3f, 1f)]
    [InlineData(1f, 1f, 1f)]
    public void FitCylinder_returns_null_for_an_exactly_planar_set_with_an_off_axis_normal(float nx, float ny, float nz)
    {
        var normal = Vector3.Normalize(new Vector3(nx, ny, nz));
        var (u, v) = Basis.Orthonormal(normal);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        for (int i = 0; i < 21; i++)
        for (int j = 0; j < 21; j++)
        {
            points.Add(normal * 0.05f + u * (0.002f * i) + v * (0.002f * j));
            normals.Add(normal);
        }
        var cloud = new PointCloud(points.ToArray(), normals.ToArray());
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        var values = NormalMomentEigenvalues(cloud, indices);
        Assert.True(values[1] <= 1e-2 * values[2], $"lambda1/lambda2 = {values[1] / values[2]:E3} must be negligible");
        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>
    /// A flat face whose normal error is anisotropic - the routine case for normals estimated from a depth map,
    /// where the error along the scan lines differs from the error across them - shrinks the smallest eigenvalue
    /// without making the set any less planar. The lambda0/lambda1 ratio then slips under its threshold while the
    /// set is still rank 1, so only the lambda1/lambda2 scale guard can reject it.
    /// </summary>
    [Theory]
    [InlineData(3f, 3f)]
    [InlineData(3f, 4f)]
    [InlineData(3f, 6f)]
    [InlineData(10f, 4f)]
    public void FitCylinder_returns_null_for_a_plane_with_anisotropic_normal_noise(float normalNoiseDegrees, float anisotropy)
    {
        var cloud = PlanarPatch(normalNoiseDegrees, anisotropy, depthNoise: 0.0005f, seed: 11);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>A genuine cylinder sweeps its normals around the axis, so the gap test must not reject it.</summary>
    [Fact]
    public void FitCylinder_recovers_a_genuine_cylinder()
    {
        var cloud = CylinderBand(normalNoiseDegrees: 0f, seed: 1);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        var cylinder = PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitZ);

        Assert.NotNull(cylinder);
        Assert.InRange(cylinder.Radius, 0.0199f, 0.0201f);
        Assert.True(MathF.Abs(cylinder.Axis.Z) > 0.999f);
        Assert.False(cylinder.IsHole);
    }

    /// <summary>
    /// Exact normals make the smallest eigenvalue identically zero, which is the one case where the gap test
    /// cannot fail. With real normal noise the axial eigenvalue is nonzero, and the gate still has to let a
    /// genuine cylinder through.
    /// </summary>
    [Theory]
    [InlineData(5f)]
    [InlineData(8f)]
    [InlineData(12f)]
    public void FitCylinder_recovers_a_cylinder_with_noisy_normals(float normalNoiseDegrees)
    {
        var cloud = CylinderBand(normalNoiseDegrees, seed: 1);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        var cylinder = PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitZ);

        Assert.NotNull(cylinder);
        Assert.InRange(cylinder.Radius, 0.0199f, 0.0201f);
        Assert.True(MathF.Abs(cylinder.Axis.Z) > 0.99f);
        Assert.False(cylinder.IsHole);
    }

    /// <summary>A full 20 mm cylindrical band, 60 angular samples by 8 rings, with optional normal jitter.</summary>
    private static PointCloud CylinderBand(float normalNoiseDegrees, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        for (int i = 0; i < 60; i++)
        {
            float theta = 2 * MathF.PI * i / 60;
            var radial = new Vector3(MathF.Cos(theta), MathF.Sin(theta), 0);
            for (int k = 0; k < 8; k++)
            {
                points.Add(radial * 0.02f + new Vector3(0, 0, 0.004f * k));
                normals.Add(SampleClouds.Tilt(radial, normalNoiseDegrees, rng));
            }
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    /// <summary>
    /// A 40 mm square patch of the plane z = 0.05. Normals are tilted by up to <paramref name="normalNoiseDegrees"/>
    /// along one in-plane direction and by <paramref name="anisotropy"/> times less along the other, so the tilt
    /// magnitude stays the same while the error ellipse gets narrower.
    /// </summary>
    private static PointCloud PlanarPatch(float normalNoiseDegrees, float anisotropy, float depthNoise, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        var (u, v) = Basis.Orthonormal(Vector3.UnitZ);
        float major = normalNoiseDegrees * MathF.PI / 180f;
        float minor = major / anisotropy;
        for (int i = 0; i < 21; i++)
        for (int j = 0; j < 21; j++)
        {
            points.Add(new Vector3(0.002f * i, 0.002f * j, 0.05f + depthNoise * SampleClouds.Gaussian(rng)));
            float phi = (float)(2 * Math.PI * rng.NextDouble());
            float scale = (float)rng.NextDouble();
            float tu = major * scale * MathF.Cos(phi), tv = minor * scale * MathF.Sin(phi);
            float tilt = MathF.Sqrt(tu * tu + tv * tv);
            if (tilt < 1e-9f)
            {
                normals.Add(Vector3.UnitZ);
                continue;
            }
            var tangent = (u * tu + v * tv) / tilt;
            normals.Add(Vector3.Normalize(Vector3.UnitZ * MathF.Cos(tilt) + tangent * MathF.Sin(tilt)));
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    private static double[] NormalMomentEigenvalues(PointCloud cloud, IReadOnlyList<int> indices)
    {
        var moments = new double[3, 3];
        foreach (int i in indices)
        {
            double[] a = { cloud.Normals[i].X, cloud.Normals[i].Y, cloud.Normals[i].Z };
            for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                moments[r, c] += a[r] * a[c];
        }
        return SymmetricEigen3.Solve(moments).Values;
    }
}
