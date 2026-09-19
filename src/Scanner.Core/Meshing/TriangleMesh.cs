using System.Numerics;

namespace Scanner.Core.Meshing;

public sealed class TriangleMesh
{
    public List<Vector3> Positions { get; } = new();
    /// <summary>Per-vertex normals, pointing out of the solid.</summary>
    public List<Vector3> Normals { get; } = new();
    public List<int> Indices { get; } = new();

    public int TriangleCount => Indices.Count / 3;

    public Vector3 FaceNormal(int triangle)
    {
        var a = Positions[Indices[3 * triangle]];
        var b = Positions[Indices[3 * triangle + 1]];
        var c = Positions[Indices[3 * triangle + 2]];
        return Vector3.Normalize(Vector3.Cross(b - a, c - a));
    }

    /// <summary>Adds the quad a-b-c-d as two triangles with the same winding.</summary>
    public void AddQuad(int a, int b, int c, int d)
    {
        Indices.AddRange([a, b, c, a, c, d]);
    }
}
