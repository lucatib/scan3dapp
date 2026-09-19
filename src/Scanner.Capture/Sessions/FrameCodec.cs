using System.Numerics;
using System.Text;

namespace Scanner.Capture.Sessions;

/// <summary>
/// Binary depth-frame file: magic "S3DF", version, width, height, fx, fy, cx, cy, timestamp,
/// camera→world matrix (16 floats, row-major M11..M44), confidence flag, depth as uint16 millimetres,
/// optional confidence bytes. Little-endian.
/// </summary>
public static class FrameCodec
{
    private const int Version = 1;
    private static readonly byte[] Magic = Encoding.ASCII.GetBytes("S3DF");

    public static void Write(Stream stream, DepthFrame frame, byte[]? confidence)
    {
        var k = frame.Intrinsics;
        if (confidence is not null && confidence.Length != frame.Depth.Length)
            throw new ArgumentException("Confidence buffer size does not match the depth buffer.", nameof(confidence));

        using var writer = new BinaryWriter(stream, Encoding.ASCII, leaveOpen: true);
        writer.Write(Magic);
        writer.Write(Version);
        writer.Write(k.Width);
        writer.Write(k.Height);
        writer.Write(k.Fx);
        writer.Write(k.Fy);
        writer.Write(k.Cx);
        writer.Write(k.Cy);
        writer.Write(frame.TimestampSeconds);
        foreach (float value in Elements(frame.CameraToWorld)) writer.Write(value);
        writer.Write(confidence is not null);
        foreach (float d in frame.Depth)
            writer.Write((ushort)Math.Clamp(MathF.Round(d * 1000f), 0f, ushort.MaxValue));
        if (confidence is not null) writer.Write(confidence);
    }

    public static (DepthFrame Frame, byte[]? Confidence) Read(Stream stream)
    {
        using var reader = new BinaryReader(stream, Encoding.ASCII, leaveOpen: true);
        try
        {
            if (!reader.ReadBytes(4).AsSpan().SequenceEqual(Magic)) throw new InvalidDataException("Not a Scan3D frame file.");
            int version = reader.ReadInt32();
            if (version != Version) throw new InvalidDataException($"Unsupported frame version {version}.");

            var k = new CameraIntrinsics(reader.ReadInt32(), reader.ReadInt32(),
                reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            double timestamp = reader.ReadDouble();
            var m = new float[16];
            for (int i = 0; i < 16; i++) m[i] = reader.ReadSingle();
            bool hasConfidence = reader.ReadBoolean();

            int count = checked(k.Width * k.Height);
            var depth = new float[count];
            for (int i = 0; i < count; i++) depth[i] = reader.ReadUInt16() * 0.001f;
            byte[]? confidence = hasConfidence ? reader.ReadBytes(count) : null;
            if (confidence is not null && confidence.Length != count) throw new InvalidDataException("Truncated confidence data.");

            var pose = new Matrix4x4(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
            return (new DepthFrame(k, depth, pose, timestamp), confidence);
        }
        catch (EndOfStreamException ex)
        {
            throw new InvalidDataException("Truncated frame file.", ex);
        }
    }

    private static IEnumerable<float> Elements(Matrix4x4 m) =>
    [
        m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
        m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44,
    ];
}
