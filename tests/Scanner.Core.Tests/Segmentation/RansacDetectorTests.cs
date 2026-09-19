using System.Numerics;
using Scanner.Core.Segmentation;
using Scanner.Core.Shapes;

namespace Scanner.Core.Tests.Segmentation;

public class RansacDetectorTests
{
    private static readonly RansacOptions Options = new(DistanceThreshold: 0.001f, NormalThresholdDegrees: 5f, MinInliers: 200);

    [Fact]
    public void Cube_yields_six_axis_aligned_planes()
    {
        var cloud = SampleClouds.Cube(0.02f, 0.001f, 0.0002f, seed: 3);

        var shapes = RansacDetector.Detect(cloud, Options);

        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        Assert.Equal(6, shapes.Count);
        Assert.Equal(6, planes.Count);
        foreach (var plane in planes)
        {
            float maxComponent = MathF.Max(MathF.Abs(plane.Normal.X), MathF.Max(MathF.Abs(plane.Normal.Y), MathF.Abs(plane.Normal.Z)));
            Assert.True(maxComponent > MathF.Cos(2f * MathF.PI / 180f));
            Assert.InRange(plane.D, 0.0197f, 0.0203f);
        }
    }

    [Fact]
    public void Tube_yields_outer_cylinder_hole_and_two_caps()
    {
        var cloud = SampleClouds.Tube(0.02f, 0.01f, 0.03f, 0.001f, 0.0002f, seed: 5);

        var shapes = RansacDetector.Detect(cloud, Options);

        var cylinders = shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>().ToList();
        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        Assert.Equal(2, cylinders.Count);
        Assert.Equal(2, planes.Count);

        var outer = Assert.Single(cylinders, c => !c.IsHole);
        var hole = Assert.Single(cylinders, c => c.IsHole);
        Assert.InRange(outer.Radius, 0.0197f, 0.0203f);
        Assert.InRange(hole.Radius, 0.0097f, 0.0103f);
        Assert.True(MathF.Abs(outer.Axis.Z) > 0.999f);
        Assert.True(outer.RadialVector(Vector3.Zero).Length() < 0.0003f);
        Assert.All(planes, p => Assert.InRange(p.D, 0.0147f, 0.0153f));
    }
}
