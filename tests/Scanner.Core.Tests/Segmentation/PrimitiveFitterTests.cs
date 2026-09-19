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
        var cloud = PlanarPatch(normalNoiseDegrees: 3f, depthNoise: 0.0005f, seed: 11);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>The same patch with mathematically exact points and normals must be refused too.</summary>
    [Fact]
    public void FitCylinder_returns_null_for_an_exactly_planar_inlier_set()
    {
        var cloud = PlanarPatch(normalNoiseDegrees: 0f, depthNoise: 0f, seed: 11);
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        Assert.Null(PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitX));
    }

    /// <summary>A genuine cylinder sweeps its normals around the axis, so the gap test must not reject it.</summary>
    [Fact]
    public void FitCylinder_recovers_a_genuine_cylinder()
    {
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        for (int i = 0; i < 60; i++)
        {
            float theta = 2 * MathF.PI * i / 60;
            var radial = new Vector3(MathF.Cos(theta), MathF.Sin(theta), 0);
            for (int k = 0; k < 8; k++)
            {
                points.Add(radial * 0.02f + new Vector3(0, 0, 0.004f * k));
                normals.Add(radial);
            }
        }
        var cloud = new PointCloud(points.ToArray(), normals.ToArray());
        var indices = Enumerable.Range(0, cloud.Count).ToList();

        var cylinder = PrimitiveFitter.FitCylinder(cloud, indices, Vector3.UnitZ);

        Assert.NotNull(cylinder);
        Assert.InRange(cylinder.Radius, 0.0199f, 0.0201f);
        Assert.True(MathF.Abs(cylinder.Axis.Z) > 0.999f);
        Assert.False(cylinder.IsHole);
    }

    /// <summary>A 40 mm square patch of the plane z = 0.05, with outward normals jittered by up to the given angle.</summary>
    private static PointCloud PlanarPatch(float normalNoiseDegrees, float depthNoise, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        var (u, v) = Basis.Orthonormal(Vector3.UnitZ);
        float jitter = normalNoiseDegrees * MathF.PI / 180f;
        for (int i = 0; i < 21; i++)
        for (int j = 0; j < 21; j++)
        {
            points.Add(new Vector3(0.002f * i, 0.002f * j, 0.05f + depthNoise * Gaussian(rng)));
            float phi = (float)(2 * Math.PI * rng.NextDouble());
            float tilt = jitter * (float)rng.NextDouble();
            var tangent = u * MathF.Cos(phi) + v * MathF.Sin(phi);
            normals.Add(Vector3.Normalize(Vector3.UnitZ * MathF.Cos(tilt) + tangent * MathF.Sin(tilt)));
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    private static float Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
