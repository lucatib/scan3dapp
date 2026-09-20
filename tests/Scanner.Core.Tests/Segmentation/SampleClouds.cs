using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Segmentation;

namespace Scanner.Core.Tests.Segmentation;

/// <summary>Point clouds sampled directly on known shapes, with exact outward normals unless noise is asked for.</summary>
internal static class SampleClouds
{
    public static PointCloud Cube(float half, float spacing, float noise, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
        foreach (float sign in new[] { -1f, 1f })
        {
            var n = axis * sign;
            var (u, v) = Basis.Orthonormal(n);
            for (float a = -half; a <= half; a += spacing)
            for (float b = -half; b <= half; b += spacing)
            {
                points.Add(n * half + u * a + v * b + n * (noise * Gaussian(rng)));
                normals.Add(n);
            }
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    public static PointCloud Tube(float outer, float inner, float height, float spacing, float noise, int seed, float normalNoiseDegrees = 0f)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        foreach (var (radius, sign) in new[] { (outer, 1f), (inner, -1f) })
        {
            int steps = (int)(2 * MathF.PI * radius / spacing);
            for (int i = 0; i < steps; i++)
            {
                float theta = 2 * MathF.PI * i / steps;
                var radial = new Vector3(MathF.Cos(theta), MathF.Sin(theta), 0);
                for (float z = -height / 2; z <= height / 2; z += spacing)
                {
                    points.Add(radial * (radius + noise * Gaussian(rng)) + new Vector3(0, 0, z));
                    normals.Add(Tilt(radial * sign, normalNoiseDegrees, rng));
                }
            }
        }
        foreach (float side in new[] { -1f, 1f })
        for (float x = -outer; x <= outer; x += spacing)
        for (float y = -outer; y <= outer; y += spacing)
        {
            float r = MathF.Sqrt(x * x + y * y);
            if (r < inner || r > outer) continue;
            points.Add(new Vector3(x, y, side * height / 2 + noise * Gaussian(rng)));
            normals.Add(Tilt(new Vector3(0, 0, side), normalNoiseDegrees, rng));
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    /// <summary>
    /// A cylindrical band that sweeps only <paramref name="arcDegrees"/> of its circle, axis along Z. With
    /// noisy normals the two-point candidate a RANSAC round draws lands slightly off the true axis, so its
    /// measured angular span differs from the span the least-squares refinement ends up with.
    /// </summary>
    public static PointCloud PartialCylinder(float radius, float arcDegrees, float height, float spacing, float noise, int seed, float normalNoiseDegrees = 0f)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        int steps = (int)(arcDegrees * MathF.PI / 180f * radius / spacing);
        for (int i = 0; i <= steps; i++)
        {
            float theta = arcDegrees * MathF.PI / 180f * i / steps;
            var radial = new Vector3(MathF.Cos(theta), MathF.Sin(theta), 0);
            for (float z = -height / 2; z <= height / 2; z += spacing)
            {
                points.Add(radial * (radius + noise * Gaussian(rng)) + new Vector3(0, 0, z));
                normals.Add(Tilt(radial, normalNoiseDegrees, rng));
            }
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    public static PointCloud Translate(PointCloud cloud, Vector3 delta) =>
        new(cloud.Points.Select(p => p + delta).ToArray(), cloud.Normals.ToArray());

    public static PointCloud Concat(PointCloud a, PointCloud b) =>
        new(a.Points.Concat(b.Points).ToArray(), a.Normals.Concat(b.Normals).ToArray());

    /// <summary>Tilts a unit normal by a uniformly random angle in [0, degrees] about a uniformly random direction.</summary>
    internal static Vector3 Tilt(Vector3 n, float degrees, Random rng)
    {
        if (degrees <= 0) return n;
        var (u, v) = Basis.Orthonormal(n);
        float phi = (float)(2 * Math.PI * rng.NextDouble());
        float tilt = degrees * MathF.PI / 180f * (float)rng.NextDouble();
        return Vector3.Normalize(n * MathF.Cos(tilt) + (u * MathF.Cos(phi) + v * MathF.Sin(phi)) * MathF.Sin(tilt));
    }

    internal static float Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
