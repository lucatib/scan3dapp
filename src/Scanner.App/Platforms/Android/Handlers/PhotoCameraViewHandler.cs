using System.Diagnostics;
using Android.Opengl;
using Google.AR.Core;
using Javax.Microedition.Khronos.Opengles;
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.App.Droid.Ar;
using Scanner.App.Droid.Rendering;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Handlers;

/// <summary>
/// The camera for the printed-mat turntable: ARCore only as the camera (its CPU image comes with intrinsics, at up to
/// 1920×1080), its tracking unused since the phone stands still. Draws the picture and offers a photo every
/// <see cref="Renderer.PhotoInterval"/>, one at a time.
/// </summary>
public sealed class PhotoCameraViewHandler : ViewHandler<PhotoCameraView, GLSurfaceView>
{
    public static readonly IPropertyMapper<PhotoCameraView, PhotoCameraViewHandler> PropertyMapper =
        new PropertyMapper<PhotoCameraView, PhotoCameraViewHandler>(ViewMapper);

    public static readonly CommandMapper<PhotoCameraView, PhotoCameraViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(PhotoCameraView.Start)] = (handler, _, _) => handler.StartCamera(),
            [nameof(PhotoCameraView.Stop)] = (handler, _, _) => handler.StopCamera(),
        };

    private Renderer? _renderer;
    private Session? _session;
    private bool _running;

    public PhotoCameraViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    protected override GLSurfaceView CreatePlatformView()
    {
        var view = new GLSurfaceView(Context) { PreserveEGLContextOnPause = true };
        view.SetEGLContextClientVersion(3);
        view.SetEGLConfigChooser(8, 8, 8, 8, 16, 0);
        _renderer = new Renderer(photo => (((IElementHandler)this).VirtualView as PhotoCameraView)?.Raise(photo),
            () => view.Display is { } display ? (int)display.Rotation : 0);
        view.SetRenderer(_renderer);
        view.RenderMode = Rendermode.Continuously;
        view.OnPause();
        return view;
    }

    protected override void ConnectHandler(GLSurfaceView platformView)
    {
        base.ConnectHandler(platformView);
        if (VirtualView.IsRunRequested) StartCamera();
    }

    protected override void DisconnectHandler(GLSurfaceView platformView)
    {
        StopCamera();
        _renderer?.AttachSession(null);
        _session?.Close();
        _session = null;
        base.DisconnectHandler(platformView);
    }

    private void StartCamera()
    {
        if (_running || ((IElementHandler)this).PlatformView is not GLSurfaceView view) return;
        try
        {
            if (_session is null)
            {
                _session = new Session(Context);
                SelectLargestImage(_session);
                var config = new Config(_session);
                config.SetFocusMode(Config.FocusMode.Auto!);
                _session.Configure(config);
                _renderer?.AttachSession(_session);
            }
            _session.Resume();
            view.OnResume();
            _running = true;
        }
        catch (Exception ex)
        {
            Android.Util.Log.Warn("Scan3D", $"Camera failed: {ex}");
        }
    }

    private void StopCamera()
    {
        (((IElementHandler)this).PlatformView as GLSurfaceView)?.OnPause();
        if (!_running) return;
        _running = false;
        _session?.Pause();
    }

    private static void SelectLargestImage(Session session)
    {
        using var filter = new CameraConfigFilter(session);
        filter.SetFacingDirection(CameraConfig.FacingDirection.Back!);
        var best = session.GetSupportedCameraConfigs(filter)!
            .Where(c => c.ImageSize!.Width <= 1920 && c.ImageSize!.Height <= 1080)
            .MaxBy(c => (long)c.ImageSize!.Width * c.ImageSize!.Height);
        if (best is not null) session.CameraConfig = best;
    }

    private sealed class Renderer(Action<PhotoCapture> offer, Func<int> readDisplayRotation)
        : Java.Lang.Object, GLSurfaceView.IRenderer
    {
        public static readonly TimeSpan PhotoInterval = TimeSpan.FromMilliseconds(250);

        private readonly CameraBackgroundRenderer _background = new();
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private volatile Session? _session;
        private volatile bool _geometryChanged;
        private int _width, _height, _rotation;
        private int _busy;
        private TimeSpan _lastPhoto;

        public void AttachSession(Session? session)
        {
            _session = session;
            _geometryChanged = true;
        }

        public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
        {
            GLES30.GlClearColor(0, 0, 0, 1);
            _background.Initialize();
        }

        public void OnSurfaceChanged(IGL10? gl, int width, int height)
        {
            GLES30.GlViewport(0, 0, width, height);
            (_width, _height, _rotation) = (width, height, readDisplayRotation());
            _geometryChanged = true;
        }

        public void OnDrawFrame(IGL10? gl)
        {
            GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
            var session = _session;
            if (session is null || _width == 0) return;
            try
            {
                if (_geometryChanged)
                {
                    _geometryChanged = false;
                    session.SetDisplayGeometry(_rotation, _width, _height);
                    _background.InvalidateTextureCoordinates();
                }
                session.SetCameraTextureName(_background.TextureId);
                var frame = session.Update()!;
                _background.Draw(frame);
                if (_clock.Elapsed - _lastPhoto < PhotoInterval || Interlocked.CompareExchange(ref _busy, 1, 0) != 0) return;
                var image = CameraImageReader.TryRead(frame, frame.Camera!, frame.Timestamp / 1e9);
                if (image is null)
                {
                    Volatile.Write(ref _busy, 0);
                    return;
                }
                _lastPhoto = _clock.Elapsed;
                Task.Run(() =>
                {
                    try
                    {
                        offer(new PhotoCapture(CameraImageReader.Preview(image), image.Metadata,
                            () => CameraImageReader.EncodeJpeg(image)));
                    }
                    catch (Exception ex)
                    {
                        Android.Util.Log.Warn("Scan3D", $"Photo failed: {ex.Message}");
                    }
                    finally
                    {
                        Volatile.Write(ref _busy, 0);
                    }
                });
            }
            catch (Exception ex)
            {
                Android.Util.Log.Warn("Scan3D", $"Camera frame failed: {ex.Message}");
            }
        }
    }
}
