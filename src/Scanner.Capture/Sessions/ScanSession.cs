using System.IO.Compression;
using System.Numerics;
using System.Text.Json;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Sessions;

/// <summary>Session metadata stored as manifest.json. Target is the world point the user aimed at (x, y, z).</summary>
public sealed record ScanManifest(
    int Version,
    string Id,
    DateTimeOffset CreatedUtc,
    string Device,
    int FrameCount,
    float[]? Target,
    float? SupportPlaneHeight,
    int PointCount);

internal static class SessionPaths
{
    public const string Manifest = "manifest.json";
    public const string Points = "points.ply";
    public const string Frames = "frames";

    public static string Frame(string directory, int index) => Path.Combine(directory, Frames, $"{index:D6}.frame");

    public static readonly JsonSerializerOptions Json = new() { WriteIndented = true };
}

/// <summary>Writes a scan session folder: frames as they arrive, then points and manifest on completion.</summary>
public sealed class ScanSessionWriter
{
    private readonly string _directory;
    private readonly string _id;
    private readonly string _device;
    private readonly DateTimeOffset _created = DateTimeOffset.UtcNow;

    public ScanSessionWriter(string directory, string id, string device)
    {
        _directory = directory;
        _id = id;
        _device = device;
        Directory.CreateDirectory(Path.Combine(directory, SessionPaths.Frames));
    }

    public int FrameCount { get; private set; }

    public void AppendFrame(DepthFrame frame, byte[]? confidence)
    {
        using var file = File.Create(SessionPaths.Frame(_directory, FrameCount + 1));
        FrameCodec.Write(file, frame, confidence);
        FrameCount++;
    }

    public void Complete(Vector3? target, float? supportPlaneHeight, IReadOnlyList<Vector3> points)
    {
        using (var file = File.Create(Path.Combine(_directory, SessionPaths.Points)))
            PlyWriter.Write(file, points);

        var manifest = new ScanManifest(1, _id, _created, _device, FrameCount,
            target is { } t ? [t.X, t.Y, t.Z] : null, supportPlaneHeight, points.Count);
        File.WriteAllText(Path.Combine(_directory, SessionPaths.Manifest), JsonSerializer.Serialize(manifest, SessionPaths.Json));
    }
}

public static class ScanSessionReader
{
    public static ScanManifest ReadManifest(string directory) =>
        JsonSerializer.Deserialize<ScanManifest>(File.ReadAllText(Path.Combine(directory, SessionPaths.Manifest)), SessionPaths.Json)
        ?? throw new InvalidDataException("Empty manifest.");

    public static Vector3[] ReadPoints(string directory)
    {
        using var file = File.OpenRead(Path.Combine(directory, SessionPaths.Points));
        return PlyReader.Read(file);
    }

    public static IEnumerable<(DepthFrame Frame, byte[]? Confidence)> ReadFrames(string directory)
    {
        var files = Directory.GetFiles(Path.Combine(directory, SessionPaths.Frames), "*.frame").Order(StringComparer.Ordinal);
        foreach (var path in files)
        {
            using var file = File.OpenRead(path);
            yield return FrameCodec.Read(file);
        }
    }
}

public static class ScanArchive
{
    /// <summary>Zips a session folder into a portable .scan file (overwrites an existing file).</summary>
    public static void Export(string sessionDirectory, string scanFilePath)
    {
        if (File.Exists(scanFilePath)) File.Delete(scanFilePath);
        ZipFile.CreateFromDirectory(sessionDirectory, scanFilePath, CompressionLevel.Fastest, includeBaseDirectory: false);
    }
}
