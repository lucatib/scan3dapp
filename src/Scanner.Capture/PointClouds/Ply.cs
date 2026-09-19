using System.Buffers.Binary;
using System.Numerics;
using System.Text;

namespace Scanner.Capture.PointClouds;

/// <summary>Writes point clouds as binary little-endian PLY (float x, y, z in metres).</summary>
public static class PlyWriter
{
    public static void Write(Stream stream, IReadOnlyList<Vector3> points)
    {
        var header = "ply\n"
                     + "format binary_little_endian 1.0\n"
                     + "comment Scan3D point cloud, units: metres\n"
                     + $"element vertex {points.Count}\n"
                     + "property float x\n"
                     + "property float y\n"
                     + "property float z\n"
                     + "end_header\n";
        stream.Write(Encoding.ASCII.GetBytes(header));

        Span<byte> vertex = stackalloc byte[12];
        foreach (var p in points)
        {
            BinaryPrimitives.WriteSingleLittleEndian(vertex[..4], p.X);
            BinaryPrimitives.WriteSingleLittleEndian(vertex[4..8], p.Y);
            BinaryPrimitives.WriteSingleLittleEndian(vertex[8..], p.Z);
            stream.Write(vertex);
        }
    }
}

/// <summary>Reads the PLY files produced by <see cref="PlyWriter"/> (binary little-endian, float x, y, z only).</summary>
public static class PlyReader
{
    public static Vector3[] Read(Stream stream)
    {
        int count = -1;
        if (ReadLine(stream) != "ply") throw new InvalidDataException("Not a PLY file.");
        if (ReadLine(stream) != "format binary_little_endian 1.0") throw new InvalidDataException("Only binary little-endian PLY is supported.");
        while (true)
        {
            var line = ReadLine(stream) ?? throw new InvalidDataException("Unexpected end of PLY header.");
            if (line == "end_header") break;
            if (line.StartsWith("element vertex ", StringComparison.Ordinal)) count = int.Parse(line["element vertex ".Length..]);
        }
        if (count < 0) throw new InvalidDataException("PLY header has no vertex element.");

        var points = new Vector3[count];
        Span<byte> vertex = stackalloc byte[12];
        for (int i = 0; i < count; i++)
        {
            stream.ReadExactly(vertex);
            points[i] = new Vector3(
                BinaryPrimitives.ReadSingleLittleEndian(vertex[..4]),
                BinaryPrimitives.ReadSingleLittleEndian(vertex[4..8]),
                BinaryPrimitives.ReadSingleLittleEndian(vertex[8..]));
        }
        return points;
    }

    private static string? ReadLine(Stream stream)
    {
        var builder = new StringBuilder();
        while (true)
        {
            int b = stream.ReadByte();
            if (b < 0) return builder.Length == 0 ? null : builder.ToString();
            if (b == '\n') return builder.ToString();
            builder.Append((char)b);
            if (builder.Length > 256) throw new InvalidDataException("PLY header line too long.");
        }
    }
}
