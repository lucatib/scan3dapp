using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Core.Meshing;

/// <summary>One piece of a depth sensor's own surface mesh (an ARKit mesh anchor), already in world metres.</summary>
public sealed record MeshChunk(Vector3[] Positions, int[] Indices);

/// <summary>The piece cut out of a sensor mesh, and the table it stood on.</summary>
public sealed record SensorMeshPiece(TriangleMesh Mesh, FittedSupportPlane? Plane);

/// <summary>
/// Cuts the scanned piece out of the surface mesh a LiDAR phone builds of the whole room: the triangles near the
/// target, above the table, in the parts big enough to be the piece. The mesh is measured, so unlike the photo
/// reconstruction there is nothing to fuse or carve; it is open underneath, where the table hid it.
/// </summary>
public static class SensorMesh
{
    /// <summary>How far around the target, horizontally, the piece may reach.</summary>
    public const float Radius = 0.30f;

    /// <summary>How far above the target the piece may reach (the target is usually on its side or top).</summary>
    public const float Height = 0.50f;

    /// <summary>ARKit's mesh is about a centimetre thick on a table: closer to it than this is table, not piece.</summary>
    private const float MinTableMargin = 0.008f;

    /// <param name="chunks">The sensor's mesh, in world metres, +Y up.</param>
    /// <param name="target">The point picked on the piece.</param>
    /// <param name="tablePoints">Depth points of the scan, to fit the table from (they are far denser than the mesh
    /// on a flat table); the mesh's own vertices are used when these give no plane.</param>
    /// <param name="tableHeight">The table level the AR session found, for when nothing fits a plane.</param>
    public static SensorMeshPiece Cut(IReadOnlyList<MeshChunk> chunks, Vector3 target,
        IReadOnlyList<Vector3>? tablePoints = null, float? tableHeight = null)
    {
        var near = Crop(chunks, target);
        var plane = (tablePoints is { Count: > 0 } ? SupportPlaneFinder.Fit(tablePoints) : null)
                    ?? SupportPlaneFinder.Fit(near.Positions)
                    ?? (tableHeight is { } h ? new FittedSupportPlane(0, 0, h) : null);
        if (plane is { } table) plane = table with { Margin = MathF.Max(table.Margin, MinTableMargin) };

        var piece = MeshCleanup.Piece(near, target, plane, linkDistance: 0.01f);
        if (plane is { } support) MeshCleanup.FlattenOnto(piece, support, band: 0.004f);
        return new SensorMeshPiece(piece, plane);
    }

    /// <summary>The triangles with every corner inside the cylinder around the target, merged into one mesh with
    /// area-weighted vertex normals.</summary>
    internal static TriangleMesh Crop(IReadOnlyList<MeshChunk> chunks, Vector3 target)
    {
        var mesh = new TriangleMesh();
        float radiusSquared = Radius * Radius;
        bool Inside(Vector3 p)
        {
            float dx = p.X - target.X, dz = p.Z - target.Z;
            return dx * dx + dz * dz <= radiusSquared && p.Y >= target.Y - Height && p.Y <= target.Y + Height;
        }

        foreach (var chunk in chunks)
        {
            var map = new int[chunk.Positions.Length];
            for (int i = 0; i < map.Length; i++)
            {
                map[i] = -1;
                if (!Inside(chunk.Positions[i])) continue;
                map[i] = mesh.Positions.Count;
                mesh.Positions.Add(chunk.Positions[i]);
                mesh.Normals.Add(Vector3.Zero);
            }
            for (int t = 0; t + 2 < chunk.Indices.Length; t += 3)
            {
                int a = map[chunk.Indices[t]], b = map[chunk.Indices[t + 1]], c = map[chunk.Indices[t + 2]];
                if (a >= 0 && b >= 0 && c >= 0) mesh.Indices.AddRange([a, b, c]);
            }
        }

        for (int t = 0; t < mesh.TriangleCount; t++)
        {
            // Unnormalized: the cross product's length is twice the area, which weights the vertex normals.
            int a = mesh.Indices[3 * t], b = mesh.Indices[3 * t + 1], c = mesh.Indices[3 * t + 2];
            var n = Vector3.Cross(mesh.Positions[b] - mesh.Positions[a], mesh.Positions[c] - mesh.Positions[a]);
            mesh.Normals[a] += n;
            mesh.Normals[b] += n;
            mesh.Normals[c] += n;
        }
        for (int i = 0; i < mesh.Normals.Count; i++)
            mesh.Normals[i] = mesh.Normals[i] == Vector3.Zero ? Vector3.UnitY : Vector3.Normalize(mesh.Normals[i]);
        return mesh;
    }
}
