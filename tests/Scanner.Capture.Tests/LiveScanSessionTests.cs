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
    public void Integrate_after_completion_is_ignored()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Complete();

        Assert.Equal(0, session.Integrate(FlatFrame(1), null));
        Assert.Equal(0, session.FrameCount);
    }
}
