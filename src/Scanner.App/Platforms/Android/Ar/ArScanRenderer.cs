using System.Diagnostics;
using System.Numerics;
using Android.Opengl;
using Google.AR.Core;
using Google.AR.Core.Exceptions;
using Javax.Microedition.Khronos.Opengles;
using Scanner.App.Controls;
using Scanner.App.Droid.Rendering;
using Scanner.Capture.Live;
using ArFrame = Google.AR.Core.Frame;
using ArPlane = Google.AR.Core.Plane;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Ar;

/// <summary>
/// GL-thread renderer for the scan screen: updates ARCore, draws the camera background and the live points,
/// picks the target when a scan starts, and hands depth frames to a worker thread for integration.
/// </summary>
internal sealed class ArScanRenderer : Java.Lang.Object, GLSurfaceView.IRenderer
{
    private const float PointSizePixels = 7f;
    private static readonly TimeSpan PointUploadInterval = TimeSpan.FromMilliseconds(300);
    private static readonly TimeSpan StatusInterval = TimeSpan.FromMilliseconds(250);

    private readonly Action<ArScanStatus> _reportStatus;
    private readonly CameraBackgroundRenderer _background = new();
    private readonly PointCloudRenderer _points = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly float[] _view = new float[16];
    private readonly float[] _projection = new float[16];
    private readonly float[] _viewProjection = new float[16];

    private volatile Session? _session;
    private volatile LiveScanSession? _scan;
    private volatile bool _geometryChanged;
    private volatile string? _message;
    private LiveScanSession? _uploadedScan;
    private int _width;
    private int _height;
    private int _integrating;
    private TimeSpan _lastUpload;
    private TimeSpan _lastStatus;

    public ArScanRenderer(Action<ArScanStatus> reportStatus) => _reportStatus = reportStatus;

    /// <summary>Sets the ARCore session to drive. Call only while the GL thread is paused (GLSurfaceView.OnPause) or before it starts.</summary>
    public void AttachSession(Session? session)
    {
        _session = session;
        _geometryChanged = true; // a new session needs SetDisplayGeometry before its first Update
    }

    public void SetScan(LiveScanSession? scan) => _scan = scan;

    public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
    {
        GLES30.GlClearColor(0f, 0f, 0f, 1f);
        _background.Initialize();
        _points.Initialize();
        _uploadedScan = null;
        _lastUpload = TimeSpan.Zero;
    }

    public void OnSurfaceChanged(IGL10? gl, int width, int height)
    {
        GLES30.GlViewport(0, 0, width, height);
        _width = width;
        _height = height;
        _geometryChanged = true;
    }

    public void OnDrawFrame(IGL10? gl)
    {
        GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
        var session = _session;
        if (session is null || _width == 0 || _height == 0) return;

        var scan = _scan;
        string tracking = "Starting";
        try
        {
            if (_geometryChanged)
            {
                _geometryChanged = false;
                session.SetDisplayGeometry(0, _width, _height); // portrait-locked activity: rotation 0
                _background.InvalidateTextureCoordinates();
            }
            session.SetCameraTextureName(_background.TextureId); // ARCore needs the texture before Update
            ArFrame frame = session.Update()!;
            Camera camera = frame.Camera!;
            _background.Draw(frame);

            bool isTracking = camera.TrackingState!.Equals(TrackingState.Tracking!);
            tracking = isTracking ? "Tracking" : "Move the phone slowly";

            if (!ReferenceEquals(scan, _uploadedScan) || _clock.Elapsed - _lastUpload >= PointUploadInterval)
            {
                _points.Upload(scan is null ? [] : scan.SnapshotPoints());
                _uploadedScan = scan;
                _lastUpload = _clock.Elapsed;
            }

            if (isTracking && scan is not null)
            {
                var state = scan.State;
                if (state == LiveScanState.WaitingForTarget) TryPickTarget(session, frame, scan);
                else if (state == LiveScanState.Recording && Volatile.Read(ref _integrating) == 0) TryIntegrate(frame, camera, scan);

                camera.GetViewMatrix(_view, 0);
                camera.GetProjectionMatrix(_projection, 0, 0.05f, 20f);
                Android.Opengl.Matrix.MultiplyMM(_viewProjection, 0, _projection, 0, _view, 0);
                _points.Draw(_viewProjection, PointSizePixels);
            }
        }
        catch (Exception ex)
        {
            _message = ex.Message;
        }

        if (_clock.Elapsed - _lastStatus >= StatusInterval)
        {
            _lastStatus = _clock.Elapsed;
            _reportStatus(new ArScanStatus(tracking, scan?.State ?? LiveScanState.Idle,
                scan?.PointCount ?? 0, scan?.FrameCount ?? 0, _message));
            _message = null;
        }
    }

    // The target is the depth/plane hit under the screen centre (the crosshair); the support plane is the highest
    // tracked upward-facing horizontal plane below it.
    private void TryPickTarget(Session session, ArFrame frame, LiveScanSession scan)
    {
        var hit = frame.HitTest(_width / 2f, _height / 2f)?.FirstOrDefault();
        if (hit is null)
        {
            _message = "Aim the crosshair at the piece";
            return;
        }
        var pose = hit.HitPose!;
        var target = new Vector3(pose.Tx(), pose.Ty(), pose.Tz());

        float? planeHeight = null;
        foreach (var trackable in session.GetAllTrackables(Java.Lang.Class.FromType(typeof(ArPlane)))!)
        {
            if (trackable is not ArPlane plane) continue;
            if (plane.SubsumedBy is not null) continue; // merged into another plane, which is visited instead
            if (!plane.TrackingState!.Equals(TrackingState.Tracking!)) continue;
            // The binding exposes Java getType() as a GetType() that hides object.GetType(); the typed local proves
            // at compile time that this is ARCore's Plane.Type and not System.Type (whose Equals would always be false).
            ArPlane.Type planeType = plane.GetType()!;
            if (!planeType.Equals(ArPlane.Type.HorizontalUpwardFacing!)) continue;
            float y = plane.CenterPose!.Ty();
            if (y < target.Y - 0.005f && (planeHeight is null || y > planeHeight)) planeHeight = y;
        }

        if (scan.State == LiveScanState.WaitingForTarget) scan.SetTarget(target, planeHeight);
    }

    private void TryIntegrate(ArFrame frame, Camera camera, LiveScanSession scan)
    {
        if (Interlocked.CompareExchange(ref _integrating, 1, 0) != 0) return; // previous frame still integrating

        (Scanner.Capture.DepthFrame Frame, byte[] Confidence)? data;
        try
        {
            // Throttling happens only for fresh depth, so a stale depth frame never consumes an integration slot.
            data = DepthFrameReader.TryRead(frame, camera, scan.ShouldIntegrate);
        }
        catch (NotYetAvailableException)
        {
            data = null; // normal while ARCore warms up its depth estimate
        }
        catch
        {
            Volatile.Write(ref _integrating, 0);
            throw;
        }

        if (data is not { } depth)
        {
            Volatile.Write(ref _integrating, 0);
            return;
        }

        // Never integrate on the GL thread: back-projection and the frame write run on the thread pool.
        Task.Run(() =>
        {
            try
            {
                scan.Integrate(depth.Frame, depth.Confidence);
            }
            catch (Exception ex)
            {
                _message = $"Integration failed: {ex.Message}";
            }
            finally
            {
                Volatile.Write(ref _integrating, 0);
            }
        });
    }
}
