using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;

namespace Scanner.Brep.Tests.Builders;

public class TubeBuilderTests
{
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
}
