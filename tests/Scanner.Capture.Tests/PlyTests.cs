using System.Numerics;
using System.Text;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class PlyTests
{
    [Fact]
    public void Writes_binary_little_endian_header_and_vertices()
    {
        using var stream = new MemoryStream();

        PlyWriter.Write(stream, [new Vector3(1, 2, 3), new Vector3(-1, 0.5f, 0)]);

        var bytes = stream.ToArray();
        var text = Encoding.ASCII.GetString(bytes);
        int headerEnd = text.IndexOf("end_header\n", StringComparison.Ordinal) + "end_header\n".Length;
        Assert.StartsWith("ply\nformat binary_little_endian 1.0\n", text);
        Assert.Contains("element vertex 2\n", text);
        Assert.Equal(headerEnd + 2 * 12, bytes.Length);
        Assert.Equal(1f, BitConverter.ToSingle(bytes, headerEnd));
        Assert.Equal(0.5f, BitConverter.ToSingle(bytes, headerEnd + 16));
    }

    [Fact]
    public void Round_trips_through_reader()
    {
        Vector3[] points = [new(1, 2, 3), new(-0.25f, 0.125f, 9)];
        using var stream = new MemoryStream();
        PlyWriter.Write(stream, points);
        stream.Position = 0;

        var read = PlyReader.Read(stream);

        Assert.Equal(points, read);
    }

    [Fact]
    public void Reader_rejects_non_ply()
    {
        using var stream = new MemoryStream(Encoding.ASCII.GetBytes("hello"));

        Assert.Throws<InvalidDataException>(() => PlyReader.Read(stream));
    }
}
