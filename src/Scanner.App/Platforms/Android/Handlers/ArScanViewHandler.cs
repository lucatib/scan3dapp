using Android.Opengl;
using Google.AR.Core;
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.App.Droid.Ar;
using Scanner.Capture.Live;

namespace Scanner.App.Droid.Handlers;

public sealed class ArScanViewHandler : ViewHandler<ArScanView, GLSurfaceView>
{
    public static readonly IPropertyMapper<ArScanView, ArScanViewHandler> PropertyMapper =
        new PropertyMapper<ArScanView, ArScanViewHandler>(ViewMapper)
        {
            [nameof(ArScanView.Session)] = (handler, view) => handler._renderer?.SetScan(view.Session),
        };

    public static readonly CommandMapper<ArScanView, ArScanViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(ArScanView.Resume)] = (handler, _, _) => handler.ResumeAr(),
            [nameof(ArScanView.Pause)] = (handler, _, _) => handler.PauseAr(),
        };

    private ArScanRenderer? _renderer;
    private Session? _session;
    private bool _running;

    public ArScanViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    // The typed VirtualView/PlatformView properties throw once the handler is disconnected; these do not.
    private ArScanView? CurrentView => ((IElementHandler)this).VirtualView as ArScanView;

    private GLSurfaceView? CurrentPlatformView => ((IElementHandler)this).PlatformView as GLSurfaceView;

    protected override GLSurfaceView CreatePlatformView()
    {
        var view = new GLSurfaceView(Context) { PreserveEGLContextOnPause = true };
        view.SetEGLContextClientVersion(3);
        view.SetEGLConfigChooser(8, 8, 8, 8, 16, 0);
        _renderer = new ArScanRenderer(
            status => MainThread.BeginInvokeOnMainThread(() => CurrentView?.ReportStatus(status)),
            // The display the view is attached to, read on the GL thread when the surface changes, as ARCore's own
            // sample does: a portrait-locked activity is rotation 0 only on a portrait-natural device.
            () => view.Display is { } display ? (int)display.Rotation : 0);
        view.SetRenderer(_renderer);
        view.RenderMode = Rendermode.Continuously;
        // GLSurfaceView starts its GL thread when attached to the window; keep it paused until Resume is requested.
        view.OnPause();
        return view;
    }

    protected override void ConnectHandler(GLSurfaceView platformView)
    {
        base.ConnectHandler(platformView);
        _renderer?.SetScan(VirtualView.Session);
        if (VirtualView.IsResumeRequested) ResumeAr(); // Resume() was called before this handler existed
    }

    protected override void DisconnectHandler(GLSurfaceView platformView)
    {
        PauseAr();
        _renderer?.AttachSession(null);
        _renderer?.SetScan(null);
        _session?.Close();
        _session = null;
        base.DisconnectHandler(platformView);
    }

    private void ResumeAr()
    {
        if (_running) return;
        var view = CurrentPlatformView;
        if (view is null) return;
        try
        {
            if (_session is null)
            {
                _session = CreateSession();
                _renderer?.AttachSession(_session); // GL thread is paused here, so this cannot race Update
            }
            _session.Resume(); // before the GL thread resumes: Update on a paused session throws
            view.OnResume();
            _running = true;
        }
        catch (Exception ex)
        {
            var virtualView = CurrentView;
            virtualView?.ReportStatus(new ArScanStatus("Unavailable", virtualView.Session?.State ?? LiveScanState.Idle,
                0, 0, ex.Message));
        }
    }

    private void PauseAr()
    {
        // OnPause is idempotent and blocks until the GL thread has left OnDrawFrame, so it runs even when AR was
        // never resumed: DisconnectHandler closes the session right after, and a GL thread started by a detach and
        // re-attach of the platform view would otherwise call Update on a closed session (a native crash).
        CurrentPlatformView?.OnPause();
        if (!_running) return;
        _running = false;
        _session?.Pause();
    }

    private Session CreateSession()
    {
        var session = new Session(Context);
        try
        {
            if (!session.IsDepthModeSupported(Config.DepthMode.Automatic!))
                throw new NotSupportedException("This device does not support ARCore depth, which scanning requires.");
            var config = new Config(session);
            config.SetDepthMode(Config.DepthMode.Automatic!);
            config.SetFocusMode(Config.FocusMode.Auto!);
            config.SetPlaneFindingMode(Config.PlaneFindingMode.Horizontal!);
            session.Configure(config);
            return session;
        }
        catch
        {
            session.Close();
            throw;
        }
    }
}
