using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;

namespace Scanner.Brep.Tests.Builders;

public class TubeBuilderTests
{
    private static readonly Vector3 AxisPoint = Vector3.Zero;
    private static readonly Vector3 Axis = Vector3.UnitZ;

    private const float OuterRadius = 0.02f;
    private const float InnerRadius = 0.01f;
    private const float ZBottom = -0.015f;
    private const float ZTop = 0.015f;

    /// <summary>Probe offset: small compared to the tube, large compared to float noise.</summary>
    private const float Step = 1e-3f;

    /// <summary>Safety margin required between a probe point and the boundary of the material.</summary>
    private const float Margin = 1e-4f;

    private const float Tolerance = 1e-6f;

    private static BrepSolid BuildTube() =>
        TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, ZBottom, ZTop);

    [Fact]
    public void Tube_has_four_faces_and_valid_genus_one_topology()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        Assert.Equal(4, solid.Faces.Count);
        Assert.Equal(4, solid.DistinctEdges().Count);
        Assert.Equal(4, solid.DistinctVertices().Count);
        Assert.Equal(2, solid.Faces.Count(f => f.Surface is CylinderSurface));
        Assert.Single(solid.Faces, f => f.Surface is CylinderSurface && !f.SameSense);
        Assert.All(solid.Faces.Where(f => f.Surface is PlaneSurface), f => Assert.Equal(2, f.Loops.Count));
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Solid_cylinder_without_hole_is_valid()
    {
        var solid = TubeBuilder.Build(new Vector3(0.01f, 0, 0), Vector3.UnitY, 0.02f, null, 0f, 0.05f);

        Assert.Equal(3, solid.Faces.Count);
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Cap_planes_face_away_from_each_other_along_axis()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        var caps = solid.Faces.Select(f => f.Surface).OfType<PlaneSurface>().ToList();
        Assert.Contains(caps, p => p.Normal == -Vector3.UnitZ && MathF.Abs(p.Origin.Z + 0.015f) < 1e-6f);
        Assert.Contains(caps, p => p.Normal == Vector3.UnitZ && MathF.Abs(p.Origin.Z - 0.015f) < 1e-6f);
    }

    /// <summary>
    /// The cylinder surface normal always points away from the axis, so the outer wall keeps it
    /// (SameSense = true) and the hole reverses it (SameSense = false): both then point out of the
    /// material. The faces are selected by radius, so reordering them cannot hide an inversion.
    /// </summary>
    [Fact]
    public void Cylinder_senses_keyed_on_radius_point_both_normals_out_of_the_material()
    {
        var solid = BuildTube();

        Assert.True(CylinderFace(solid, OuterRadius).SameSense,
            "The outer wall must keep the surface normal, which points away from the axis and out of the material.");
        Assert.False(CylinderFace(solid, InnerRadius).SameSense,
            "The hole must reverse the surface normal so that it points at the axis, i.e. out of the material.");
    }

    /// <summary>
    /// The full orientation table documented on <see cref="TubeBuilder"/>. Faces are identified by
    /// geometry (radius for the walls, normal for the caps) and edges by centre and radius, and every
    /// loop is checked to carry the very same shared <see cref="BrepEdge"/> instance the builder
    /// creates once per circle.
    /// </summary>
    [Fact]
    public void Face_loops_carry_the_documented_circles_with_the_documented_senses()
    {
        var solid = BuildTube();

        var outerBottom = CircleEdge(solid, ZBottom, OuterRadius);
        var outerTop = CircleEdge(solid, ZTop, OuterRadius);
        var innerBottom = CircleEdge(solid, ZBottom, InnerRadius);
        var innerTop = CircleEdge(solid, ZTop, InnerRadius);

        var outerWall = CylinderFace(solid, OuterRadius);
        AssertLoop(outerWall, loopIndex: 0, outerBottom, sameSense: true);
        AssertLoop(outerWall, loopIndex: 1, outerTop, sameSense: false);

        var hole = CylinderFace(solid, InnerRadius);
        AssertLoop(hole, loopIndex: 0, innerBottom, sameSense: false);
        AssertLoop(hole, loopIndex: 1, innerTop, sameSense: true);

        var bottomCap = CapFace(solid, -Axis);
        AssertLoop(bottomCap, loopIndex: 0, outerBottom, sameSense: false);
        AssertLoop(bottomCap, loopIndex: 1, innerBottom, sameSense: true);

        var topCap = CapFace(solid, Axis);
        AssertLoop(topCap, loopIndex: 0, outerTop, sameSense: true);
        AssertLoop(topCap, loopIndex: 1, innerTop, sameSense: false);
    }

    /// <summary>
    /// The invariant behind the whole table: for every loop of every face, the interior direction
    /// N x T (N = effective face normal, T = oriented loop tangent) points into the face patch, and
    /// the material lies on the -N side. Probing one step along N x T and then one step against N
    /// must therefore land strictly inside the wall of the tube. A flipped face normal or a flipped
    /// loop sense sends the probe out of the material.
    /// </summary>
    [Fact]
    public void Every_loop_puts_the_material_on_the_correct_side_of_its_face()
    {
        var solid = BuildTube();

        foreach (var face in solid.Faces)
        {
            for (int l = 0; l < face.Loops.Count; l++)
            {
                var oriented = Assert.Single(face.Loops[l].Edges);
                var circle = Assert.IsType<CircleCurve>(oriented.Edge.Curve);

                var start = circle.Center + Vector3.Normalize(circle.RefDirection) * circle.Radius;
                var tangent = Vector3.Normalize(Vector3.Cross(circle.Axis, circle.RefDirection));
                if (!oriented.SameSense) tangent = -tangent;

                var normal = EffectiveNormal(face, start);
                var intoFace = Vector3.Normalize(Vector3.Cross(normal, tangent));
                var probe = start + intoFace * Step - normal * Step;

                Assert.True(IsInsideMaterial(probe),
                    $"{Describe(face)}, loop {l}: probing into the face and under the surface reaches " +
                    $"{probe}, which is outside the material (axial {Axial(probe)}, radial {Radial(probe)}).");
            }
        }
    }

    [Fact]
    public void Build_rejects_a_non_finite_axis_point()
    {
        // The caller derives the axis point from the same RANSAC fit as the axial extent, so it is
        // no less likely to arrive non-finite — and it flows into every circle centre and surface origin.
        Assert.Equal("axisPoint", RejectedParameter(() =>
            TubeBuilder.Build(new Vector3(float.NaN, 0f, 0f), Axis, OuterRadius, InnerRadius, ZBottom, ZTop)));
        Assert.Equal("axisPoint", RejectedParameter(() =>
            TubeBuilder.Build(new Vector3(0f, float.PositiveInfinity, 0f), Axis, OuterRadius, InnerRadius, ZBottom, ZTop)));
    }

    [Fact]
    public void Build_rejects_a_degenerate_or_non_finite_axis()
    {
        Assert.Equal("axis", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Vector3.Zero, OuterRadius, InnerRadius, ZBottom, ZTop)));
        Assert.Equal("axis", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, new Vector3(float.NaN, 0f, 1f), OuterRadius, InnerRadius, ZBottom, ZTop)));
        Assert.Equal("axis", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, new Vector3(0f, 0f, float.PositiveInfinity), OuterRadius, InnerRadius, ZBottom, ZTop)));
    }

    [Fact]
    public void Build_rejects_a_non_positive_or_non_finite_outer_radius()
    {
        Assert.Equal("outerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, 0f, null, ZBottom, ZTop)));
        Assert.Equal("outerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, -OuterRadius, null, ZBottom, ZTop)));
        Assert.Equal("outerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, float.NaN, null, ZBottom, ZTop)));
    }

    [Fact]
    public void Build_rejects_a_hole_radius_outside_the_open_interval_up_to_the_outer_radius()
    {
        Assert.Equal("innerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, float.NaN, ZBottom, ZTop)));
        Assert.Equal("innerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, 0f, ZBottom, ZTop)));
        Assert.Equal("innerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, -InnerRadius, ZBottom, ZTop)));
        Assert.Equal("innerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, OuterRadius, ZBottom, ZTop)));
        Assert.Equal("innerRadius", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, OuterRadius + InnerRadius, ZBottom, ZTop)));
    }

    [Fact]
    public void Build_rejects_a_non_finite_or_inverted_axial_extent()
    {
        Assert.Equal("zBottom", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, float.NaN, ZTop)));
        Assert.Equal("zTop", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, ZBottom, float.NaN)));
        Assert.Equal("zTop", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, ZBottom, ZBottom)));
        Assert.Equal("zTop", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, ZTop, ZBottom)));
        // An empty inlier set in the caller yields 0/0 for both bounds; neither may slip through.
        Assert.Equal("zBottom", RejectedParameter(() =>
            TubeBuilder.Build(AxisPoint, Axis, OuterRadius, InnerRadius, float.NaN, float.NaN)));
    }

    private static string? RejectedParameter(Action build) =>
        Assert.Throws<ArgumentOutOfRangeException>(build).ParamName;

    private static BrepFace CylinderFace(BrepSolid solid, float radius) =>
        Assert.Single(solid.Faces, f => f.Surface is CylinderSurface c && MathF.Abs(c.Radius - radius) < Tolerance);

    private static BrepFace CapFace(BrepSolid solid, Vector3 normal) =>
        Assert.Single(solid.Faces, f => f.Surface is PlaneSurface p && (p.Normal - normal).Length() < Tolerance);

    private static BrepEdge CircleEdge(BrepSolid solid, float z, float radius) =>
        Assert.Single(solid.DistinctEdges(), e => e.Curve is CircleCurve c
            && MathF.Abs(c.Radius - radius) < Tolerance
            && MathF.Abs(Axial(c.Center) - z) < Tolerance);

    private static void AssertLoop(BrepFace face, int loopIndex, BrepEdge expected, bool sameSense)
    {
        var oriented = Assert.Single(face.Loops[loopIndex].Edges);
        Assert.Same(expected, oriented.Edge);
        Assert.Equal(sameSense, oriented.SameSense);
    }

    private static Vector3 EffectiveNormal(BrepFace face, Vector3 point)
    {
        var normal = face.Surface switch
        {
            PlaneSurface plane => Vector3.Normalize(plane.Normal),
            CylinderSurface cylinder => RadialDirection(cylinder, point),
            _ => throw new NotSupportedException($"Unexpected surface {face.Surface.GetType().Name}."),
        };
        return face.SameSense ? normal : -normal;
    }

    /// <summary>Outward normal of a cylindrical surface at <paramref name="point"/>.</summary>
    private static Vector3 RadialDirection(CylinderSurface cylinder, Vector3 point)
    {
        var axis = Vector3.Normalize(cylinder.Axis);
        var offset = point - cylinder.Origin;
        return Vector3.Normalize(offset - axis * Vector3.Dot(offset, axis));
    }

    private static bool IsInsideMaterial(Vector3 point) =>
        Axial(point) > ZBottom + Margin && Axial(point) < ZTop - Margin
        && Radial(point) > InnerRadius + Margin && Radial(point) < OuterRadius - Margin;

    private static float Axial(Vector3 point) => Vector3.Dot(point - AxisPoint, Axis);

    private static float Radial(Vector3 point)
    {
        var offset = point - AxisPoint;
        return (offset - Axis * Vector3.Dot(offset, Axis)).Length();
    }

    private static string Describe(BrepFace face) => face.Surface switch
    {
        CylinderSurface c => $"cylinder of radius {c.Radius} (SameSense={face.SameSense})",
        PlaneSurface p => $"cap with normal {p.Normal} (SameSense={face.SameSense})",
        _ => face.Surface.GetType().Name,
    };
}
