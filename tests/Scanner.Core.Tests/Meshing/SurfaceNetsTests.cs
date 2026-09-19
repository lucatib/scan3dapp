using System.Numerics;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Tests.Meshing;

public class SurfaceNetsTests
{
    private const float Radius = 0.02f;

    [Fact]
    public void Sphere_vertices_lie_on_surface_with_outward_normals()
    {
        var mesh = SurfaceNets.Extract(SphereVolume());

        Assert.True(mesh.Positions.Count > 500);
        Assert.Equal(mesh.Positions.Count, mesh.Normals.Count);
        for (int i = 0; i < mesh.Positions.Count; i++)
        {
            var p = mesh.Positions[i];
            Assert.InRange(p.Length(), Radius - 0.0005f, Radius + 0.0005f);
            Assert.True(Vector3.Dot(mesh.Normals[i], Vector3.Normalize(p)) > 0.95f);
        }
    }

    [Fact]
    public void Sphere_triangles_are_wound_outward()
    {
        var mesh = SurfaceNets.Extract(SphereVolume());

        int outward = 0;
        for (int t = 0; t < mesh.TriangleCount; t++)
        {
            var centroid = (mesh.Positions[mesh.Indices[3 * t]] + mesh.Positions[mesh.Indices[3 * t + 1]]
                            + mesh.Positions[mesh.Indices[3 * t + 2]]) / 3f;
            if (Vector3.Dot(mesh.FaceNormal(t), centroid) > 0) outward++;
        }

        Assert.True(mesh.TriangleCount > 1000);
        Assert.True(outward >= mesh.TriangleCount * 0.99);
    }

    private static TsdfVolume SphereVolume()
    {
        var volume = new TsdfVolume(0.002f, 0.006f);
        for (int z = -15; z <= 15; z++)
        for (int y = -15; y <= 15; y++)
        for (int x = -15; x <= 15; x++)
        {
            float sdf = volume.VoxelToWorld(x, y, z).Length() - Radius;
            volume.Set(x, y, z, Math.Clamp(sdf / volume.TruncationDistance, -1f, 1f), 1f);
        }
        return volume;
    }
}
