using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Live;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Tests;

public sealed class LiveScanSessionTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "scan3d-live-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, recursive: true);
    }

    private LiveScanSession NewSession(LiveScanOptions? options = null) =>
        new(new ScanSessionWriter(_dir, "live", "test"), options);

    // 20x20 pixels, every pixel 1 m away on the camera's optical axis direction; identity pose.
    private static DepthFrame FlatFrame(double time)
    {
        var k = new CameraIntrinsics(20, 20, 20f, 20f, 9.5f, 9.5f);
        return new DepthFrame(k, Enumerable.Repeat(1f, 400).ToArray(), Matrix4x4.Identity, time);
    }

    [Fact]
    public void Start_waits_for_target_then_records_with_throttling()
    {
        var session = NewSession();

        session.RequestStart();
        Assert.Equal(LiveScanState.WaitingForTarget, session.State);
        Assert.False(session.ShouldIntegrate(0));

        session.SetTarget(new Vector3(0, 0, 1), 0.5f);
        Assert.Equal(LiveScanState.Recording, session.State);
        Assert.Equal(new Vector3(0, 0, 1), session.Target);
        Assert.Equal(0.5f, session.SupportPlaneHeight);
        Assert.True(session.ShouldIntegrate(10.0));
        Assert.False(session.ShouldIntegrate(10.1));
        Assert.True(session.ShouldIntegrate(10.25));
    }

    [Fact]
    public void Pause_and_resume_keep_the_target()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(Vector3.Zero, null);

        session.Pause();
        Assert.Equal(LiveScanState.Paused, session.State);
        Assert.False(session.ShouldIntegrate(1));

        session.RequestStart();
        Assert.Equal(LiveScanState.Recording, session.State);
        Assert.Equal(Vector3.Zero, session.Target);
    }

    [Fact]
    public void Pause_while_waiting_for_target_returns_to_idle()
    {
        var session = NewSession();
        session.RequestStart();

        session.Pause();

        Assert.Equal(LiveScanState.Idle, session.State);
    }

    [Fact]
    public void SetTarget_outside_waiting_state_throws()
    {
        Assert.Throws<InvalidOperationException>(() => NewSession().SetTarget(Vector3.Zero, null));
    }

    [Fact]
    public void Integrate_keeps_points_inside_the_region_and_records_the_frame()
    {
        var session = NewSession(new LiveScanOptions(RegionRadius: 0.2f));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);

        int added = session.Integrate(FlatFrame(0), null);

        Assert.True(added > 0 && added < 400);
        Assert.Equal(1, session.FrameCount);
        Assert.All(session.SnapshotPoints(), p => Assert.True(Vector3.Distance(p, new Vector3(0, 0, 1)) <= 0.2f + 0.005f));
        Assert.Equal(session.SnapshotPoints().Length, session.PointCount);
    }

    [Fact]
    public void Complete_writes_the_session_and_isolates_the_piece()
    {
        var session = NewSession(new LiveScanOptions(MinIsolatedPoints: 1));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Integrate(FlatFrame(0), null);

        var result = session.Complete();

        Assert.Equal(LiveScanState.Completed, session.State);
        Assert.True(result.Isolated);
        Assert.NotEmpty(result.PiecePoints);
        Assert.Equal(result.PiecePoints.Length, ScanSessionReader.ReadPoints(_dir).Length);
        Assert.Equal(1, ScanSessionReader.ReadManifest(_dir).FrameCount);
        Assert.Throws<InvalidOperationException>(() => session.Complete());
        Assert.Throws<InvalidOperationException>(() => session.RequestStart());
    }

    [Fact]
    public void Complete_without_enough_isolated_points_falls_back_to_all_points()
    {
        var session = NewSession(new LiveScanOptions(MinIsolatedPoints: 1_000_000));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Integrate(FlatFrame(0), null);

        var result = session.Complete();

        Assert.False(result.Isolated);
        Assert.Equal(result.AllPoints.Length, result.PiecePoints.Length);
    }

    [Fact]
    public void Photos_are_throttled_by_interval_and_capped()
    {
        var session = NewSession(new LiveScanOptions(PhotoIntervalSeconds: 0.5, MaxPhotos: 3));
        Assert.False(session.ShouldCapturePhoto(0)); // not recording yet
        session.RequestStart();
        session.SetTarget(Vector3.Zero, null);

        Assert.True(session.ShouldCapturePhoto(10.0));
        Assert.False(session.ShouldCapturePhoto(10.3));
        Assert.True(session.ShouldCapturePhoto(10.6));
        Assert.True(session.ShouldCapturePhoto(11.2));
        // The cap counts what was promised, not what was written: none of the three above got as far as AddPhoto.
        Assert.False(session.ShouldCapturePhoto(20.0));

        session.Pause();
        Assert.False(session.ShouldCapturePhoto(30.0));
    }

    [Fact]
    public void AddPhoto_records_the_photo_and_the_manifest_counts_it()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(Vector3.Zero, null);
        session.Integrate(FlatFrame(0), null);

        Assert.True(session.AddPhoto([1, 2, 3], new ScanPhotoData(default, Matrix4x4.Identity, 0.5, 90)));
        Assert.Equal(1, session.PhotoCount);

        session.Complete();
        Assert.Equal(1, ScanSessionReader.ReadManifest(_dir).PhotoCount);
        var photo = Assert.Single(ScanSessionReader.ReadPhotos(_dir));
        Assert.Equal(90, photo.Photo.RotationDegrees);
        Assert.Equal([1, 2, 3], File.ReadAllBytes(photo.ImagePath));
    }

    // The JPEG is encoded off the render thread, so one can still be in flight when Finish is pressed. It must be
    // dropped rather than land in a folder whose manifest is already written and no longer counts it.
    [Fact]
    public void AddPhoto_after_completion_is_ignored()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(Vector3.Zero, null);
        session.Complete();

        Assert.False(session.AddPhoto([1], new ScanPhotoData(default, Matrix4x4.Identity, 0, 0)));
        Assert.Equal(0, session.PhotoCount);
        Assert.Empty(ScanSessionReader.ReadPhotos(_dir));
    }

    [Fact]
    public void Integrate_after_completion_is_ignored()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Complete();

        Assert.Equal(0, session.Integrate(FlatFrame(1), null));
        Assert.Equal(0, session.FrameCount);
    }

    // The frame write happens outside the state lock, so Complete can land while a worker is between the state
    // check and its write. Every frame on disk must still be counted by the manifest, and no frame may be written
    // after it. Several runs, because each one only hits the window with some probability.
    [Fact]
    public async Task Complete_during_concurrent_integration_counts_every_frame_on_disk()
    {
        for (int run = 0; run < 10; run++)
        {
            string directory = Path.Combine(_dir, $"run{run}");
            var session = new LiveScanSession(new ScanSessionWriter(directory, $"live{run}", "test"),
                new LiveScanOptions(RegionRadius: 0.2f));
            session.RequestStart();
            session.SetTarget(new Vector3(0, 0, 1), null);

            var worker = Task.Run(() =>
            {
                for (int i = 0; i < 100 && session.State != LiveScanState.Completed; i++)
                    session.Integrate(WideFrame(i * 0.05), null);
            });

            // Complete as soon as a frame lands: the worker is then most likely inside the next back-projection.
            var spin = new SpinWait();
            while (session.FrameCount < 2) spin.SpinOnce();
            session.Complete();
            await worker;

            int counted = ScanSessionReader.ReadManifest(directory).FrameCount;
            string[] written = Directory.GetFiles(Path.Combine(directory, "frames"), "*.frame")
                .Select(Path.GetFileName).OfType<string>().Order(StringComparer.Ordinal).ToArray();
            Assert.Equal(counted, session.FrameCount);
            Assert.Equal(Enumerable.Range(1, counted).Select(i => $"{i:D6}.frame"), written);
        }
    }

    // 64x64 pixels, all 1 m away: enough back-projection work to widen the window Complete has to race.
    private static DepthFrame WideFrame(double time)
    {
        var k = new CameraIntrinsics(64, 64, 64f, 64f, 31.5f, 31.5f);
        return new DepthFrame(k, Enumerable.Repeat(1f, 64 * 64).ToArray(), Matrix4x4.Identity, time);
    }
}
