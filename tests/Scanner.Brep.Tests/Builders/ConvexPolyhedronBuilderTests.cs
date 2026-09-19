using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Tests.Builders;

public class ConvexPolyhedronBuilderTests
{
    internal static List<PlanePrimitive> CubePlanes(float half) =>
    [
        new(Vector3.UnitX, half), new(-Vector3.UnitX, half),
        new(Vector3.UnitY, half), new(-Vector3.UnitY, half),
        new(Vector3.UnitZ, half), new(-Vector3.UnitZ, half),
    ];

    [Fact]
    public void Cube_has_expected_topology_and_is_valid()
    {
        var solid = ConvexPolyhedronBuilder.Build(CubePlanes(0.02f), 1e-4f);

        Assert.Equal(6, solid.Faces.Count);
        Assert.Equal(12, solid.DistinctEdges().Count);
        Assert.Equal(8, solid.DistinctVertices().Count);
        Assert.All(solid.Faces, f => Assert.Equal(4, Assert.Single(f.Loops).Edges.Count));
        Assert.All(solid.DistinctVertices(), v =>
            Assert.True(Vector3.Distance(Vector3.Abs(v.Position), new Vector3(0.02f)) < 1e-6f));
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Face_loops_run_counterclockwise_around_outward_normal()
    {
        var solid = ConvexPolyhedronBuilder.Build(CubePlanes(0.02f), 1e-4f);

        foreach (var face in solid.Faces)
        {
            var normal = ((PlaneSurface)face.Surface).Normal;
            var loop = face.Loops[0].Edges;
            var a = loop[0].StartVertex.Position;
            var b = loop[1].StartVertex.Position;
            var c = loop[2].StartVertex.Position;
            Assert.True(Vector3.Dot(Vector3.Cross(b - a, c - b), normal) > 0);
        }
    }

    [Fact]
    public void Near_duplicate_planes_are_merged()
    {
        var planes = CubePlanes(0.02f);
        planes.Add(new PlanePrimitive(Vector3.Normalize(new Vector3(1f, 0.01f, 0)), 0.02001f));

        var solid = ConvexPolyhedronBuilder.Build(planes, 1e-4f);

        Assert.Equal(6, solid.Faces.Count);
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Too_few_planes_throw()
    {
        Assert.Throws<InvalidOperationException>(() =>
            ConvexPolyhedronBuilder.Build(CubePlanes(0.02f).Take(3).ToList(), 1e-4f));
    }
}
