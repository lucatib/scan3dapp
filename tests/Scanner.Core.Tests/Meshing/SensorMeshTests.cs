using System.Numerics;
using Scanner.Core.Meshing;

namespace Scanner.Core.Tests.Meshing;

public class SensorMeshTests
{
    private static readonly Vector3 BoxCentre = new(0, 0.03f, 0);

    [Fact]
    public void Cuts_the_box_from_the_table_and_the_far_wall()
    {
        var chunks = new[] { Grid(-1, 1, 0.02f, 0), Box(BoxCentre, 0.03f), Wall(x: 0.8f) };

        var piece = SensorMesh.Cut(chunks, target: BoxCentre with { Y = 0.06f });

        Assert.NotNull(piece.Plane);
        Assert.InRange(piece.Plane!.Value.HeightAt(0, 0), -0.002f, 0.002f);
        Assert.True(piece.Mesh.TriangleCount > 0);
        Assert.Equal(piece.Mesh.Positions.Count, piece.Mesh.Normals.Count);
        foreach (var p in piece.Mesh.Positions)
        {
            Assert.InRange(p.X, -0.031f, 0.031f); // no table around the box, nothing of the wall
            Assert.InRange(p.Z, -0.031f, 0.031f);
        }
        Assert.Contains(piece.Mesh.Positions, p => p.Y > 0.059f); // the top is kept
    }

    [Fact]
    public void Falls_back_to_the_session_table_height_when_nothing_fits_a_plane()
    {
        var piece = SensorMesh.Cut([Box(BoxCentre, 0.03f)], BoxCentre with { Y = 0.06f }, tableHeight: 0);

        Assert.Equal(0, piece.Plane!.Value.HeightAt(0, 0));
        Assert.All(piece.Mesh.Positions, p => Assert.True(p.Y >= 0));
    }

    /// <summary>A flat table at height <paramref name="y"/>, as the triangles of a square grid.</summary>
    private static MeshChunk Grid(float from, float to, float step, float y)
    {
        int n = (int)MathF.Round((to - from) / step) + 1;
        var positions = new List<Vector3>();
        var indices = new List<int>();
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
            positions.Add(new Vector3(from + i * step, y, from + j * step));
        for (int j = 0; j + 1 < n; j++)
        for (int i = 0; i + 1 < n; i++)
        {
            int a = j * n + i;
            indices.AddRange([a, a + n, a + 1, a + 1, a + n, a + n + 1]);
        }
        return new MeshChunk([.. positions], [.. indices]);
    }

    /// <summary>A cube of half size <paramref name="half"/>, each face split into a 6x6 grid like a sensor mesh.</summary>
    private static MeshChunk Box(Vector3 centre, float half)
    {
        const int n = 7;
        var positions = new List<Vector3>();
        var indices = new List<int>();
        for (int axis = 0; axis < 3; axis++)
        for (int sign = -1; sign <= 1; sign += 2)
        {
            int start = positions.Count;
            for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                float u = -half + 2 * half * i / (n - 1), v = -half + 2 * half * j / (n - 1), w = sign * half;
                var local = axis switch { 0 => new Vector3(w, u, v), 1 => new Vector3(u, w, v), _ => new Vector3(u, v, w) };
                positions.Add(centre + local);
            }
            for (int j = 0; j + 1 < n; j++)
            for (int i = 0; i + 1 < n; i++)
            {
                int a = start + j * n + i;
                indices.AddRange([a, a + 1, a + n, a + 1, a + n + 1, a + n]);
            }
        }
        return new MeshChunk([.. positions], [.. indices]);
    }

    private static MeshChunk Wall(float x)
    {
        var grid = Grid(-0.5f, 0.5f, 0.05f, 0);
        return grid with { Positions = grid.Positions.Select(p => new Vector3(x, p.X + 0.5f, p.Z)).ToArray() };
    }
}
