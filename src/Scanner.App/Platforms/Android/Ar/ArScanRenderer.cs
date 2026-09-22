using System.Diagnostics;
using System.Numerics;
using Android.Opengl;
using Google.AR.Core;
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

    /// <summary>
    /// Consecutive draws without usable depth before the UI is told. The draw loop runs at the display rate, so
    /// 30 draws are 0.5 s at 60 fps and 1 s at 30 fps. It has to stay well above the gap a slow but working depth
    /// stream leaves: 5 Hz depth against a 60 fps draw loop is about 12 depthless draws between images.
    /// </summary>
    private const int WaitingForDepthFrames = 30;

    private const string WaitingForDepthMessage = "Waiting for depth from the camera…";

    private static readonly TimeSpan PointUploadInterval = TimeSpan.FromMilliseconds(300);
    private static readonly TimeSpan StatusInterval = TimeSpan.FromMilliseconds(250);

    /// <summary>ARCore's Plane trackable class; rebuilding it per draw would allocate a JNI reference every frame.</summary>
    private static readonly Java.Lang.Class PlaneClass = Java.Lang.Class.FromType(typeof(ArPlane))!;

    private readonly Action<ArScanStatus> _reportStatus;
    private readonly Func<int> _readDisplayRotation;
    private readonly CameraBackgroundRenderer _background = new();
    private readonly PointCloudRenderer _points = new();
    private readonly DepthFrameReader _depth = new();
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
    private int _displayRotation;
    private int _depthlessFrames; // GL thread only
    private bool _waitingForDepth; // GL thread only
    private int _integrating;
    private int _capturingPhoto;
    private TimeSpan _lastUpload;
    private TimeSpan _lastStatus;

    /// <param name="reportStatus">Called a few times a second with the state to show; may hop to the UI thread.</param>
    /// <param name="readDisplayRotation">Surface rotation (0-3) of the display showing the view; called on the GL thread.</param>
    public ArScanRenderer(Action<ArScanStatus> reportStatus, Func<int> readDisplayRotation)
    {
        _reportStatus = reportStatus;
        _readDisplayRotation = readDisplayRotation;
    }

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
        // The activity is portrait-locked, but that is rotation 0 only on a device whose natural orientation is
        // portrait; on a landscape-natural tablet both the camera background and the hit test would be rotated.
        _displayRotation = _readDisplayRotation();
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
                session.SetDisplayGeometry(_displayRotation, _width, _height);
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

            var state = scan?.State ?? LiveScanState.Idle;
            if (isTracking && scan is not null)
            {
                if (state == LiveScanState.WaitingForTarget) TryPickTarget(session, frame, scan);
                else if (state == LiveScanState.Recording)
                {
                    if (Volatile.Read(ref _integrating) == 0) TryIntegrate(frame, camera, scan);
                    TryCapturePhoto(frame, camera, scan);
                }

                camera.GetViewMatrix(_view, 0);
                camera.GetProjectionMatrix(_projection, 0, 0.05f, 20f);
                Android.Opengl.Matrix.MultiplyMM(_viewProjection, 0, _projection, 0, _view, 0);
                _points.Draw(_viewProjection, PointSizePixels);
            }
            // Only recording reads depth, so the "waiting for depth" state cannot outlive it.
            if (!isTracking || state != LiveScanState.Recording) ResetDepthWait();
        }
        catch (Exception ex)
        {
            _message = ex.Message;
        }

        if (_clock.Elapsed - _lastStatus >= StatusInterval)
        {
            _lastStatus = _clock.Elapsed;
            string? message = _message ?? (_waitingForDepth ? WaitingForDepthMessage : null);
            _reportStatus(new ArScanStatus(tracking, scan?.State ?? LiveScanState.Idle,
                scan?.PointCount ?? 0, scan?.FrameCount ?? 0, scan?.PhotoCount ?? 0, message));
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
        foreach (var trackable in session.GetAllTrackables(PlaneClass)!)
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

        DepthRead read;
        try
        {
            // Throttling happens only for fresh depth, so a stale depth frame never consumes an integration slot.
            read = _depth.Read(frame, camera, scan.ShouldIntegrate);
        }
        catch
        {
            Volatile.Write(ref _integrating, 0);
            throw;
        }

        NoteDepthOutcome(read.Outcome);
        if (read is not { Outcome: DepthReadOutcome.Read, Frame: { } depthFrame })
        {
            Volatile.Write(ref _integrating, 0);
            return;
        }
        byte[]? confidence = read.Confidence;

        // Never integrate on the GL thread: back-projection and the frame write run on the thread pool.
        Task.Run(() =>
        {
            try
            {
                scan.Integrate(depthFrame, confidence);
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

    /// <summary>
    /// Takes a camera photo when the scan asks for one: the copy off ARCore's pool happens here, the JPEG on the
    /// thread pool. Failures are reported and swallowed - the depth scan is the deliverable and must not be
    /// disturbed by a picture, which is why this does not share the integration slot either.
    /// </summary>
    private void TryCapturePhoto(ArFrame frame, Camera camera, LiveScanSession scan)
    {
        if (Interlocked.CompareExchange(ref _capturingPhoto, 1, 0) != 0) return; // previous photo still encoding

        CameraImage? image;
        try
        {
            // The camera frame's own timestamp, the same clock the depth frames are stamped with, so a photo can
            // be matched to the frames around it later.
            double seconds = frame.Timestamp / 1e9;
            image = scan.ShouldCapturePhoto(seconds) ? CameraImageReader.TryRead(frame, camera, seconds) : null;
        }
        catch (Exception ex)
        {
            _message = $"Could not take a photo: {ex.Message}";
            image = null;
        }

        if (image is null)
        {
            Volatile.Write(ref _capturingPhoto, 0);
            return;
        }

        Task.Run(() =>
        {
            try
            {
                scan.AddPhoto(CameraImageReader.EncodeJpeg(image), image.Metadata);
            }
            catch (Exception ex)
            {
                _message = $"Could not save a photo: {ex.Message}";
            }
            finally
            {
                Volatile.Write(ref _capturingPhoto, 0);
            }
        });
    }

    /// <summary>Counts consecutive draws without usable depth, so the UI can say why nothing is being recorded.</summary>
    private void NoteDepthOutcome(DepthReadOutcome outcome)
    {
        // Declined means depth is arriving and only the integration throttle turned it down: that clears the wait.
        if (outcome is not (DepthReadOutcome.Stale or DepthReadOutcome.NotAvailable))
        {
            ResetDepthWait();
            return;
        }
        if (_depthlessFrames < WaitingForDepthFrames) _depthlessFrames++;
        if (_depthlessFrames >= WaitingForDepthFrames) _waitingForDepth = true;
    }

    private void ResetDepthWait()
    {
        _depthlessFrames = 0;
        _waitingForDepth = false;
    }
}
