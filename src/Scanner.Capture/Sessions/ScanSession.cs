using System.IO.Compression;
using System.Numerics;
using System.Text.Json;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Sessions;

/// <summary>Session metadata stored as manifest.json. Target is the world point the user aimed at (x, y, z).</summary>
/// <param name="Version">2 since photos were added. A version 1 session simply has none, which reads back as
/// <see cref="PhotoCount"/> 0, so sessions recorded before this still open.</param>
public sealed record ScanManifest(
    int Version,
    string Id,
    DateTimeOffset CreatedUtc,
    string Device,
    int FrameCount,
    float[]? Target,
    float? SupportPlaneHeight,
    int PointCount,
    int PhotoCount = 0);

/// <summary>What the platform layer has to supply about a photo; the writer turns it into a <see cref="ScanPhoto"/>.</summary>
public readonly record struct ScanPhotoData(
    CameraIntrinsics Intrinsics,
    Matrix4x4 CameraToWorld,
    double TimestampSeconds,
    int RotationDegrees);

internal static class SessionPaths
{
    public const string Manifest = "manifest.json";
    public const string Points = "points.ply";
    public const string Frames = "frames";
    public const string Photos = "photos";

    public static string Frame(string directory, int index) => Path.Combine(directory, Frames, $"{index:D6}.frame");

    public static string PhotoImage(string directory, int index) => Path.Combine(directory, Photos, $"{index:D6}.jpg");

    public static string PhotoMetadata(string directory, int index) => Path.Combine(directory, Photos, $"{index:D6}.json");

    public static readonly JsonSerializerOptions Json = new() { WriteIndented = true };
}

/// <summary>Writes a scan session folder: frames and photos as they arrive, then points and manifest on completion.</summary>
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
        Directory.CreateDirectory(Path.Combine(directory, SessionPaths.Photos));
    }

    public int FrameCount { get; private set; }

    public int PhotoCount { get; private set; }

    public void AppendFrame(DepthFrame frame, byte[]? confidence)
    {
        using var file = File.Create(SessionPaths.Frame(_directory, FrameCount + 1));
        FrameCodec.Write(file, frame, confidence);
        FrameCount++;
    }

    /// <summary>Writes one camera photo and the metadata describing it. The picture goes first, so a process killed
    /// between the two writes leaves a JPEG the reader skips rather than metadata pointing at nothing.</summary>
    public void AppendPhoto(byte[] jpeg, ScanPhotoData photo)
    {
        int index = PhotoCount + 1;
        File.WriteAllBytes(SessionPaths.PhotoImage(_directory, index), jpeg);
        var record = new ScanPhoto(index, photo.TimestampSeconds, photo.Intrinsics,
            ScanPhoto.Elements(photo.CameraToWorld), photo.RotationDegrees);
        File.WriteAllText(SessionPaths.PhotoMetadata(_directory, index), JsonSerializer.Serialize(record, SessionPaths.Json));
        PhotoCount++;
    }

    public void Complete(Vector3? target, float? supportPlaneHeight, IReadOnlyList<Vector3> points)
    {
        using (var file = File.Create(Path.Combine(_directory, SessionPaths.Points)))
            PlyWriter.Write(file, points);

        var manifest = new ScanManifest(2, _id, _created, _device, FrameCount,
            target is { } t ? [t.X, t.Y, t.Z] : null, supportPlaneHeight, points.Count, PhotoCount);
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

    /// <summary>The session's photos in capture order, each with the path of its JPEG. A photo whose picture or
    /// metadata is missing is skipped: a pair half-written when the app was killed must not make the session
    /// unopenable, and the scan it belongs to is otherwise intact.</summary>
    public static IReadOnlyList<(ScanPhoto Photo, string ImagePath)> ReadPhotos(string directory)
    {
        string folder = Path.Combine(directory, SessionPaths.Photos);
        if (!Directory.Exists(folder)) return [];

        var photos = new List<(ScanPhoto Photo, string ImagePath)>();
        foreach (string path in Directory.GetFiles(folder, "*.json"))
        {
            string image = Path.ChangeExtension(path, ".jpg");
            if (!File.Exists(image)) continue;
            ScanPhoto? photo;
            try
            {
                photo = JsonSerializer.Deserialize<ScanPhoto>(File.ReadAllText(path), SessionPaths.Json);
            }
            catch (JsonException)
            {
                continue;
            }
            if (photo is not null) photos.Add((photo, image));
        }
        return photos.OrderBy(p => p.Photo.Index).ToList();
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
