using System.IO.Compression;
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Tests;

public sealed class ScanSessionTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "scan3d-tests-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_root)) Directory.Delete(_root, recursive: true);
    }

    private static DepthFrame Frame(float depth, double time) =>
        new(new CameraIntrinsics(3, 2, 100f, 101f, 1f, 0.5f), [depth, 0f, 0.5f, 1.2346f, 70f, depth],
            Matrix4x4.CreateTranslation(1, 2, 3), time);

    [Fact]
    public void Frame_codec_round_trips_with_millimetre_precision()
    {
        using var stream = new MemoryStream();
        var frame = Frame(0.8f, 1.25);
        byte[] confidence = [1, 2, 3, 4, 5, 6];

        FrameCodec.Write(stream, frame, confidence);
        stream.Position = 0;
        var (read, readConfidence) = FrameCodec.Read(stream);

        Assert.Equal(frame.Intrinsics, read.Intrinsics);
        Assert.Equal(frame.CameraToWorld, read.CameraToWorld);
        Assert.Equal(1.25, read.TimestampSeconds);
        Assert.Equal(1.235f, read.Depth[3], 4);
        Assert.Equal(65.535f, read.Depth[4], 3); // clamped to uint16 millimetres
        Assert.Equal(0f, read.Depth[1]);
        Assert.Equal(confidence, readConfidence);
    }

    [Fact]
    public void Frame_codec_supports_missing_confidence_and_rejects_garbage()
    {
        using var stream = new MemoryStream();
        FrameCodec.Write(stream, Frame(1f, 0), null);
        stream.Position = 0;

        Assert.Null(FrameCodec.Read(stream).Confidence);
        Assert.Throws<InvalidDataException>(() => FrameCodec.Read(new MemoryStream([1, 2, 3, 4, 5, 6, 7, 8])));
    }

    [Fact]
    public void Session_writes_frames_points_and_manifest()
    {
        var dir = Path.Combine(_root, "s1");
        var writer = new ScanSessionWriter(dir, "s1", "Pixel 8");
        writer.AppendFrame(Frame(0.5f, 0), null);
        writer.AppendFrame(Frame(0.6f, 0.2), [9, 9, 9, 9, 9, 9]);

        writer.Complete(new Vector3(0.1f, 0.2f, 0.3f), 0.05f, [Vector3.One, Vector3.Zero]);

        var manifest = ScanSessionReader.ReadManifest(dir);
        Assert.Equal(2, manifest.Version);
        Assert.Equal("s1", manifest.Id);
        Assert.Equal("Pixel 8", manifest.Device);
        Assert.Equal(2, manifest.FrameCount);
        Assert.Equal(2, manifest.PointCount);
        Assert.Equal([0.1f, 0.2f, 0.3f], manifest.Target!);
        Assert.Equal(0.05f, manifest.SupportPlaneHeight);
        Assert.Equal([Vector3.One, Vector3.Zero], ScanSessionReader.ReadPoints(dir));
        var frames = ScanSessionReader.ReadFrames(dir).ToList();
        Assert.Equal(2, frames.Count);
        Assert.Equal(0.2, frames[1].Frame.TimestampSeconds);
        Assert.NotNull(frames[1].Confidence);
        Assert.Equal(0, manifest.PhotoCount);
        Assert.Empty(ScanSessionReader.ReadPhotos(dir));
    }

    [Fact]
    public void Session_round_trips_photos_with_their_pose_and_intrinsics()
    {
        var dir = Path.Combine(_root, "s3");
        var writer = new ScanSessionWriter(dir, "s3", "test");
        var intrinsics = new CameraIntrinsics(640, 480, 500f, 520f, 310f, 250f);
        var pose = Matrix4x4.CreateRotationY(0.4f) * Matrix4x4.CreateTranslation(0.1f, 0.2f, 0.3f);

        writer.AppendPhoto([1, 2, 3], new ScanPhotoData(intrinsics, pose, 1.5, 90));
        writer.AppendPhoto([4, 5], new ScanPhotoData(intrinsics, Matrix4x4.Identity, 2.5, 0));
        writer.Complete(null, null, [Vector3.One]);

        Assert.Equal(2, ScanSessionReader.ReadManifest(dir).PhotoCount);
        var photos = ScanSessionReader.ReadPhotos(dir);
        Assert.Equal(2, photos.Count);
        Assert.Equal([1, 2, 3], File.ReadAllBytes(photos[0].ImagePath));
        Assert.Equal(1, photos[0].Photo.Index);
        Assert.Equal(1.5, photos[0].Photo.TimestampSeconds);
        Assert.Equal(90, photos[0].Photo.RotationDegrees);
        Assert.Equal(intrinsics, photos[0].Photo.Intrinsics);
        Assert.Equal(pose, photos[0].Photo.ToPose());
        Assert.Equal(2, photos[1].Photo.Index);
    }

    [Fact]
    public void A_photo_whose_picture_is_missing_is_skipped_rather_than_failing_the_session()
    {
        var dir = Path.Combine(_root, "s4");
        var writer = new ScanSessionWriter(dir, "s4", "test");
        writer.AppendPhoto([1], new ScanPhotoData(default, Matrix4x4.Identity, 0, 0));
        writer.AppendPhoto([2], new ScanPhotoData(default, Matrix4x4.Identity, 1, 0));

        // What a process killed between the two writes leaves behind, and what a stray file in the folder looks like.
        File.Delete(Path.Combine(dir, "photos", "000001.jpg"));
        File.WriteAllText(Path.Combine(dir, "photos", "000003.json"), "{ not json");
        File.WriteAllBytes(Path.Combine(dir, "photos", "000003.jpg"), [3]);

        var photos = ScanSessionReader.ReadPhotos(dir);
        Assert.Equal(2, Assert.Single(photos).Photo.Index);
    }

    [Fact]
    public void Archive_contains_the_whole_session()
    {
        var dir = Path.Combine(_root, "s2");
        var writer = new ScanSessionWriter(dir, "s2", "test");
        writer.AppendFrame(Frame(0.5f, 0), null);
        writer.AppendPhoto([7, 7], new ScanPhotoData(default, Matrix4x4.Identity, 0, 270));
        writer.Complete(null, null, [Vector3.One]);
        var archive = Path.Combine(_root, "s2.scan");

        ScanArchive.Export(dir, archive);

        using var zip = ZipFile.OpenRead(archive);
        var names = zip.Entries.Select(e => e.FullName.Replace('\\', '/')).ToList();
        Assert.Contains("manifest.json", names);
        Assert.Contains("points.ply", names);
        Assert.Contains("frames/000001.frame", names);
        Assert.Contains("photos/000001.jpg", names);
        Assert.Contains("photos/000001.json", names);
    }
}
