using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Sessions;
using Scanner.Core.Synthetic;
using StbImageWriteSharp;

namespace Scanner.Desktop.Tests;

public sealed class SessionProcessorTests : IDisposable
{
    private static readonly Vector3 Target = new(0, 0.04f, 0);
    private static readonly CameraIntrinsics PhotoK = new(320, 240, 290, 290, 159.5f, 119.5f);
    private static readonly CameraIntrinsics DepthK = new(80, 60, 72.5f, 72.5f, 39.5f, 29.5f);

    private readonly string _root = Path.Combine(Path.GetTempPath(), "scan3d-desktop-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_root)) Directory.Delete(_root, recursive: true);
    }

    private static ISdf Scene() => new UnionSdf([
        new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.3f, 0.01f, 0.3f)),
        new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
    ]);

    /// <summary>A session as the phone writes it: a walk around the box with photos stored upright (the rotation
    /// recorded, intrinsics and pose left as the sensor's), depth frames, and a manifest with the target.</summary>
    private string WriteSession(int rotationDegrees)
    {
        string directory = Path.Combine(_root, "session");
        var writer = new ScanSessionWriter(directory, "synthetic", "test");
        var scene = Scene();
        for (int i = 0; i < 24; i++)
        {
            float azimuth = i * 15f * MathF.PI / 180f;
            var pose = CameraPoses.LookAt(Target + new Vector3(MathF.Sin(azimuth) * 0.35f, 0.3f, -MathF.Cos(azimuth) * 0.35f), Target);
            var image = Rotate(SyntheticPhotoRenderer.Render(scene, PhotoK, pose).Image, rotationDegrees);
            writer.AppendPhoto(Png(image), new ScanPhotoData(PhotoK, pose, i * 0.5, rotationDegrees));
            if (i % 3 == 0) writer.AppendFrame(SyntheticDepthRenderer.Render(scene, DepthK, pose, timestampSeconds: i * 0.5), null);
        }
        writer.Complete(Target, null, []);
        return directory;
    }

    [Fact]
    public void Processing_a_session_isolates_the_box_from_photos_and_depth()
    {
        string session = WriteSession(rotationDegrees: 90);
        string output = Path.Combine(_root, "out");

        var result = SessionProcessor.Process(SessionLoader.Load(session, downscale: 1), output,
            new ProcessOptions(ReferenceViews: 12, DepthSamples: 96), TextWriter.Null);

        Assert.True(result.Isolated);
        Assert.True(result.PhotoPoints > 1000, $"only {result.PhotoPoints} photo points");
        var box = new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f));
        var piece = ReadPly(Path.Combine(output, "piece.ply"));
        Assert.Equal(result.PiecePoints, piece.Length);
        int onBox = piece.Count(p => MathF.Abs(box.Distance(p)) <= 0.004f);
        Assert.True(onBox >= piece.Length * 9 / 10, $"{piece.Length - onBox} of {piece.Length} piece points are off the box");
        Assert.True(File.Exists(Path.Combine(output, "merged.ply")));
        Assert.True(File.Exists(Path.Combine(output, "report.txt")));
    }

    [Fact]
    public void A_scan_archive_opens_like_its_folder()
    {
        string session = WriteSession(rotationDegrees: 0);
        string archive = Path.Combine(_root, "session.scan");
        ScanArchive.Export(session, archive);

        var loaded = SessionLoader.Load(SessionLoader.Resolve(archive, Path.Combine(_root, "unzipped")), downscale: 2);

        Assert.Equal(24, loaded.Photos.Count);
        Assert.Equal(Target, loaded.Target);
        Assert.Equal(PhotoK.Width / 2, loaded.Photos[0].Image.Width);
    }

    // The upright picture and the recorded rotation must describe the same camera: the loader turns the intrinsics and
    // pose with the pixels, so a point projects onto the upright pixel that shows it. The point is off the optical axis,
    // which a turn about that axis would not move.
    [Fact]
    public void Loaded_photos_project_points_where_the_upright_picture_shows_them()
    {
        string session = WriteSession(rotationDegrees: 90);
        var corner = new Vector3(0.04f, 0.08f, -0.04f);
        var sensorPose = CameraPoses.LookAt(Target + new Vector3(0, 0.3f, -0.35f), Target); // photo 1 of the walk
        Scanner.Core.Photogrammetry.Pinhole.Project(PhotoK,
            Scanner.Core.Photogrammetry.Pinhole.ToCamera(sensorPose, corner), out float su, out float sv);

        var photo = SessionLoader.Load(session, downscale: 1).Photos[0];

        Assert.True(Scanner.Core.Photogrammetry.Pinhole.Project(photo.Intrinsics,
            Scanner.Core.Photogrammetry.Pinhole.ToCamera(photo.CameraToWorld, corner), out float u, out float v));
        Assert.Equal(PhotoK.Height, photo.Image.Width);
        // A clockwise quarter turn moves sensor pixel (x, y) to (height - 1 - y, x).
        Assert.Equal(PhotoK.Height - 1 - sv, u, 3);
        Assert.Equal(su, v, 3);
    }

    // Clockwise quarter turns, the mapping PhotoOrientation documents: (x, y) -> (height - 1 - y, x).
    private static GrayImage Rotate(GrayImage image, int degreesClockwise)
    {
        var current = image;
        for (int turn = 0; turn < degreesClockwise / 90; turn++)
        {
            int w = current.Width, h = current.Height;
            var pixels = new byte[w * h];
            for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                pixels[x * h + (h - 1 - y)] = current[x, y];
            current = new GrayImage(h, w, pixels);
        }
        return current;
    }

    // PNG bytes under the .jpg name: the loader decodes by content, and PNG keeps the test lossless.
    private static byte[] Png(GrayImage image)
    {
        using var stream = new MemoryStream();
        new ImageWriter().WritePng(image.Pixels, image.Width, image.Height, ColorComponents.Grey, stream);
        return stream.ToArray();
    }

    private static Vector3[] ReadPly(string path)
    {
        using var file = File.OpenRead(path);
        return Scanner.Capture.PointClouds.PlyReader.Read(file);
    }
}
