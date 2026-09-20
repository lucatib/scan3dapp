using System.Numerics;
using Scanner.Core.Segmentation;
using Scanner.Core.Shapes;

namespace Scanner.Core.Tests.Segmentation;

public class RansacDetectorTests
{
    private static readonly RansacOptions Options = new(DistanceThreshold: 0.001f, NormalThresholdDegrees: 20f, MinInliers: 200);

    [Fact]
    public void Cube_yields_six_axis_aligned_planes()
    {
        var cloud = SampleClouds.Cube(0.02f, 0.001f, 0.0002f, seed: 3);

        var shapes = RansacDetector.Detect(cloud, Options);

        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        Assert.Equal(6, shapes.Count);
        Assert.Equal(6, planes.Count);
        Assert.All(shapes, s => Assert.Equal(1600, s.InlierIndices.Length));
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

    /// <summary>
    /// A cylinder whose inliers sit just above the coverage gate can dip below it once least squares moves the
    /// axis. That must cost only the refinement of that one shape: the detector has to fall back to the
    /// candidate that did pass the gate and keep going, never abandon the rest of the scan. Here the 75° band
    /// outscores every cube face, so it is picked in the very first round - before the fix the whole detection
    /// returned an empty list and all six cube faces were silently lost.
    ///
    /// The band itself comes back as the candidate the two sampled normals produced, not as the least-squares
    /// fit, so its radius is only roughly right (measured 0.0182 against a true 0.0200). That is the whole
    /// price of the fallback, and it is bounded by the 1 mm inlier threshold the candidate had to satisfy.
    /// </summary>
    [Fact]
    public void A_refined_shape_that_loses_angular_coverage_does_not_truncate_the_detection()
    {
        var band = SampleClouds.PartialCylinder(0.02f, 75f, 0.08f, 0.001f, 0.0002f, seed: 1, normalNoiseDegrees: 4f);
        var cube = SampleClouds.Translate(SampleClouds.Cube(0.02f, 0.001f, 0.0002f, seed: 1), new Vector3(0.2f, 0, 0));
        var cloud = SampleClouds.Concat(band, cube);

        var shapes = RansacDetector.Detect(cloud, Options);

        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        var cylinder = Assert.Single(shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>());
        Assert.Equal(6, planes.Count);
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
        foreach (float sign in new[] { -1f, 1f })
            Assert.Contains(planes, p => Vector3.Dot(p.Normal, axis * sign) > MathF.Cos(2f * MathF.PI / 180f));

        // Pinned to the FALLBACK's radius, not to a range wide enough to admit the least-squares fit as well.
        // The band's refined span clears the default gate by a single bin, so a future change could stop
        // exercising the fallback while every assertion above still passed - the test would go quietly vacuous
        // while still claiming to cover the fallback. 0.0182 is the unrefined candidate; 0.0200 is the fit.
        Assert.InRange(cylinder.Radius, 0.017f, 0.019f);
    }

    /// <summary>
    /// A 4 mm bore sampled at 1 mm spacing has only 25 distinct angles around its circle, so at 5° bins two
    /// out of every three bins are empty. A fixed one-bin bridging tolerance collapses its longest span to
    /// 3 bins and rejects it outright; the density-derived tolerance has to keep it.
    /// </summary>
    [Fact]
    public void A_small_bore_is_not_rejected_by_the_coverage_gate()
    {
        var cloud = SampleClouds.Tube(0.02f, 0.004f, 0.03f, 0.001f, 0.0002f, seed: 5);

        var shapes = RansacDetector.Detect(cloud, Options);

        var cylinders = shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>().ToList();
        var bore = Assert.Single(cylinders, c => c.IsHole);
        Assert.InRange(bore.Radius, 0.0038f, 0.0042f);
        Assert.True(MathF.Abs(bore.Axis.Z) > 0.999f);
        Assert.Contains(cylinders, c => !c.IsHole && c.Radius is > 0.0197f and < 0.0203f);
    }

    /// <summary>
    /// A cylinder candidate is built from two sampled normals and is discarded when they disagree on whether
    /// the surface is a boss or a bore. Exact synthetic normals can never disagree on the same surface, so
    /// only noisy normals exercise that rule - and it must not starve either cylinder of candidates.
    /// </summary>
    [Fact]
    public void Tube_with_noisy_normals_still_yields_both_cylinders()
    {
        var cloud = SampleClouds.Tube(0.02f, 0.01f, 0.03f, 0.001f, 0.0002f, seed: 5, normalNoiseDegrees: 8f);

        var shapes = RansacDetector.Detect(cloud, Options);

        var cylinders = shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>().ToList();
        var outer = Assert.Single(cylinders, c => !c.IsHole);
        var hole = Assert.Single(cylinders, c => c.IsHole);
        Assert.InRange(outer.Radius, 0.0197f, 0.0203f);
        Assert.InRange(hole.Radius, 0.0097f, 0.0103f);
    }

    /// <summary>
    /// The coverage gate is a tunable of <see cref="RansacOptions"/>, so Task 9 and the Android app can trade
    /// false positives against small bores without a code change. Asking for more than a full circle is
    /// unsatisfiable, so no cylinder survives - and the detector keeps going rather than stopping, carving the
    /// curved surfaces into tangent planes instead.
    /// </summary>
    [Fact]
    public void Raising_MinCoverageBins_past_a_full_circle_rejects_every_cylinder()
    {
        var cloud = SampleClouds.Tube(0.02f, 0.01f, 0.03f, 0.001f, 0.0002f, seed: 5);

        var shapes = RansacDetector.Detect(cloud, Options with { MinCoverageBins = 73, MaxShapes = 4 });

        Assert.Empty(shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>());
        Assert.NotEmpty(shapes.Select(s => s.Primitive).OfType<PlanePrimitive>());
    }
}
