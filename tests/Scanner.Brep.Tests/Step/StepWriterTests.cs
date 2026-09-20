using System.Numerics;
using System.Text.RegularExpressions;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;
using Scanner.Brep.Step;
using Scanner.Brep.Tests.Builders;

namespace Scanner.Brep.Tests.Step;

public class StepWriterTests
{
    private static readonly DateTime Timestamp = new(2026, 9, 19, 12, 0, 0, DateTimeKind.Utc);

    private static readonly Regex SameSensePattern = new(@",(\.[TF]\.)\)$");

    private static BrepSolid Tube() => TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

    private static BrepSolid Cube() => ConvexPolyhedronBuilder.Build(ConvexPolyhedronBuilderTests.CubePlanes(0.02f), 1e-4f);

    [Fact]
    public void Cube_step_is_well_formed_with_expected_entities()
    {
        var solid = Cube();

        var step = StepWriter.Write(solid, "cube", Timestamp);

        Assert.Empty(StepSyntaxChecker.Check(step));
        Assert.Equal(1, StepSyntaxChecker.Count(step, "MANIFOLD_SOLID_BREP"));
        Assert.Equal(1, StepSyntaxChecker.Count(step, "CLOSED_SHELL"));
        Assert.Equal(6, StepSyntaxChecker.Count(step, "ADVANCED_FACE"));
        Assert.Equal(6, StepSyntaxChecker.Count(step, "PLANE"));
        Assert.Equal(12, StepSyntaxChecker.Count(step, "EDGE_CURVE"));
        Assert.Equal(12, StepSyntaxChecker.Count(step, "LINE"));
        Assert.Equal(8, StepSyntaxChecker.Count(step, "VERTEX_POINT"));
        Assert.Contains("CARTESIAN_POINT('',(20.0,20.0,20.0))", step);
        Assert.Contains("SI_UNIT(.MILLI.,.METRE.)", step);
    }

    [Fact]
    public void Tube_step_uses_cylinders_and_circles()
    {
        var solid = Tube();

        var step = StepWriter.Write(solid, "tube", Timestamp);

        Assert.Empty(StepSyntaxChecker.Check(step));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "ADVANCED_FACE"));
        Assert.Equal(2, StepSyntaxChecker.Count(step, "CYLINDRICAL_SURFACE"));
        Assert.Equal(2, StepSyntaxChecker.Count(step, "PLANE"));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "CIRCLE"));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "EDGE_CURVE"));
        Assert.Equal(0, StepSyntaxChecker.Count(step, "LINE"));
        Assert.Contains("CYLINDRICAL_SURFACE('',#", step);
        Assert.Contains(",20.0)", step);
        Assert.Contains(",10.0)", step);
    }

    [Fact]
    public void Output_is_deterministic()
    {
        var solid = Tube();

        Assert.Equal(StepWriter.Write(solid, "tube", Timestamp), StepWriter.Write(solid, "tube", Timestamp));
    }

    [Fact]
    public void Product_name_quotes_are_escaped()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, null, 0f, 0.01f);

        var step = StepWriter.Write(solid, "user's ring", Timestamp);

        Assert.Contains("PRODUCT('user''s ring','user''s ring'", step);
    }

    /// <summary>A backslash introduces the control directives of an ISO 10303-21 string, so it is doubled too.</summary>
    [Fact]
    public void Product_name_backslashes_are_escaped()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, null, 0f, 0.01f);

        var step = StepWriter.Write(solid, @"ring\2", Timestamp);

        Assert.Contains(@"PRODUCT('ring\\2','ring\\2'", step);
        Assert.Contains(@"FILE_NAME('ring\\2.stp'", step);
    }

    /// <summary>
    /// H9. <c>BrepFace.SameSense</c> is the only signal that a hole's surface normal is inverted, so the
    /// flag on ADVANCED_FACE must be read from the model. Both expectations are pinned: hardcoding
    /// either <c>.T.</c> or <c>.F.</c> breaks one of them.
    /// </summary>
    [Fact]
    public void Advanced_face_same_sense_is_read_from_the_model()
    {
        var step = StepWriter.Write(Tube(), "tube", Timestamp);
        var entities = StepSyntaxChecker.Entities(step);

        Assert.Equal(".T.", SameSenseOfFaceOn(entities, "CYLINDRICAL_SURFACE", "20.0"));
        Assert.Equal(".F.", SameSenseOfFaceOn(entities, "CYLINDRICAL_SURFACE", "10.0"));
    }

    /// <summary>
    /// H10. The enclosing loop of a cap is FACE_OUTER_BOUND and the bore is FACE_BOUND, selected by
    /// area and not by position in <c>Loops</c>: reversing the loops of a cap must not change which
    /// bound is the outer one.
    /// </summary>
    [Fact]
    public void Cap_outer_bound_is_the_enclosing_loop_whatever_the_loop_order()
    {
        var solid = Tube();
        var reordered = new BrepSolid(solid.Faces
            .Select(f => new BrepFace(f.Surface, f.Loops.Reverse().ToList(), f.SameSense))
            .ToList());

        foreach (var step in new[] { StepWriter.Write(solid, "tube", Timestamp), StepWriter.Write(reordered, "tube", Timestamp) })
        {
            var entities = StepSyntaxChecker.Entities(step);
            foreach (var face in FacesOn(entities, "PLANE"))
            {
                Assert.Equal("20.0", BoundRadius(entities, face, "FACE_OUTER_BOUND"));
                Assert.Equal("10.0", BoundRadius(entities, face, "FACE_BOUND"));
            }
        }
    }

    /// <summary>
    /// H11. Pins the representation decision documented on <see cref="StepWriter"/>: every circle is
    /// emitted as a single closed edge, i.e. one EDGE_CURVE whose start and end vertex are the same
    /// instance. If a CAD kernel refuses the file, this is the first thing to change.
    /// </summary>
    [Fact]
    public void Every_circular_edge_is_emitted_as_a_single_closed_edge()
    {
        var step = StepWriter.Write(Tube(), "tube", Timestamp);
        var entities = StepSyntaxChecker.Entities(step);

        var edges = entities.Values.Where(e => StepSyntaxChecker.Name(e) == "EDGE_CURVE").ToList();
        Assert.Equal(4, edges.Count);
        foreach (var edge in edges)
        {
            var references = StepSyntaxChecker.References(edge);
            Assert.Equal(references[0], references[1]);
            Assert.Equal("CIRCLE", StepSyntaxChecker.Name(entities[references[2]]));
        }
    }

    /// <summary>H1. A shell whose faces are all inside-out is not emitted, even though its topology is sound.</summary>
    [Fact]
    public void Write_rejects_an_inside_out_shell()
    {
        var cube = Cube();
        var flipped = new BrepSolid(cube.Faces.Select(f => new BrepFace(f.Surface, f.Loops, !f.SameSense)).ToList());

        var error = Assert.Throws<ArgumentException>(() => StepWriter.Write(flipped, "cube", Timestamp));

        Assert.Equal("solid", error.ParamName);
        Assert.Contains("counterclockwise", error.Message, StringComparison.Ordinal);
    }

    /// <summary>H1. A face whose loops do not lie on its own plane is not emitted.</summary>
    [Fact]
    public void Write_rejects_a_face_whose_loops_are_off_its_plane()
    {
        var cube = Cube();
        var broken = cube.Faces.Select((f, i) =>
        {
            var plane = (PlaneSurface)f.Surface;
            return i == 0
                ? new BrepFace(plane with { Origin = plane.Origin + plane.Normal * 0.05f }, f.Loops, f.SameSense)
                : f;
        }).ToList();

        var error = Assert.Throws<ArgumentException>(() => StepWriter.Write(new BrepSolid(broken), "cube", Timestamp));

        Assert.Equal("solid", error.ParamName);
        Assert.Contains("does not lie on", error.Message, StringComparison.Ordinal);
    }

    /// <summary>
    /// H12. The degenerate sliver loop [e forward, e backward] balances the edge-use counts and keeps the
    /// Euler-Poincare characteristic at 2, so only an explicit rule rejects it.
    /// </summary>
    [Fact]
    public void Write_rejects_a_loop_that_uses_the_same_edge_twice()
    {
        var start = new BrepVertex(Vector3.Zero);
        var end = new BrepVertex(new Vector3(0.01f, 0f, 0f));
        var edge = new BrepEdge(start, end, new LineCurve(start.Position, Vector3.UnitX));
        var face = new BrepFace(
            new PlaneSurface(Vector3.Zero, Vector3.UnitZ, Vector3.UnitX),
            [new BrepLoop([new OrientedEdge(edge, true), new OrientedEdge(edge, false)])],
            sameSense: true);

        var error = Assert.Throws<ArgumentException>(() => StepWriter.Write(new BrepSolid([face]), "sliver", Timestamp));

        Assert.Equal("solid", error.ParamName);
        Assert.Contains("same edge twice", error.Message, StringComparison.Ordinal);
    }

    /// <summary>
    /// H15. A radius below the resolution of the emitted numbers would be written as a zero-radius
    /// CIRCLE, a degenerate solid that no earlier validation rejects. The writer owns that floor.
    /// </summary>
    [Fact]
    public void Write_rejects_a_radius_that_would_be_emitted_as_zero()
    {
        var speck = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 1e-30f, null, 0f, 0.01f);

        var error = Assert.Throws<ArgumentOutOfRangeException>(() => StepWriter.Write(speck, "speck", Timestamp));

        Assert.Equal("solid", error.ParamName);
        Assert.Contains("radius", error.Message, StringComparison.Ordinal);
    }

    /// <summary>
    /// H15. The same floor seen from the other side: an extent below the emitted resolution collapses
    /// two distinct vertices onto the same coordinates, which no radius check would notice.
    /// </summary>
    [Fact]
    public void Write_rejects_vertices_that_collapse_at_the_emitted_resolution()
    {
        var wafer = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, null, 0f, 1e-30f);

        var error = Assert.Throws<ArgumentException>(() => StepWriter.Write(wafer, "wafer", Timestamp));

        Assert.Equal("solid", error.ParamName);
        Assert.Contains("same coordinates", error.Message, StringComparison.Ordinal);
    }

    private static IEnumerable<string> FacesOn(IReadOnlyDictionary<int, string> entities, string surfaceName) =>
        entities.Values
            .Where(e => StepSyntaxChecker.Name(e) == "ADVANCED_FACE")
            .Where(e => StepSyntaxChecker.Name(entities[StepSyntaxChecker.References(e)[^1]]) == surfaceName);

    private static string SameSenseOfFaceOn(IReadOnlyDictionary<int, string> entities, string surfaceName, string surfaceRadius)
    {
        var face = Assert.Single(
            FacesOn(entities, surfaceName),
            e => LastArgument(entities[StepSyntaxChecker.References(e)[^1]]) == surfaceRadius);
        return SameSensePattern.Match(face).Groups[1].Value;
    }

    /// <summary>Radius of the circle carried by the single bound of <paramref name="face"/> of kind <paramref name="boundKind"/>.</summary>
    private static string BoundRadius(IReadOnlyDictionary<int, string> entities, string face, string boundKind)
    {
        var references = StepSyntaxChecker.References(face);
        var bound = Assert.Single(
            references[..^1].Select(id => entities[id]),
            e => StepSyntaxChecker.Name(e) == boundKind);
        var loop = entities[StepSyntaxChecker.References(bound)[0]];
        var oriented = entities[StepSyntaxChecker.References(loop)[0]];
        var edge = entities[StepSyntaxChecker.References(oriented)[0]];
        var curve = entities[StepSyntaxChecker.References(edge)[^1]];
        return LastArgument(curve);
    }

    private static string LastArgument(string entity) =>
        entity[(entity.LastIndexOf(',') + 1)..].TrimEnd(')');
}
