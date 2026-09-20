using Android.Opengl;
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.App.Droid.Viewer;

namespace Scanner.App.Droid.Handlers;

public sealed class PointCloudViewHandler : ViewHandler<PointCloudView, PointCloudGlView>
{
    public static readonly IPropertyMapper<PointCloudView, PointCloudViewHandler> PropertyMapper =
        new PropertyMapper<PointCloudView, PointCloudViewHandler>(ViewMapper)
        {
            [nameof(PointCloudView.Points)] = (handler, view) => handler.PlatformView.SetPoints(view.Points),
        };

    public static readonly CommandMapper<PointCloudView, PointCloudViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(PointCloudView.Resume)] = (handler, _, _) => handler.ResumeRendering(),
            [nameof(PointCloudView.Pause)] = (handler, _, _) => handler.PauseRendering(),
        };

    public PointCloudViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    // The typed PlatformView property throws once the handler is disconnected; this one does not.
    private GLSurfaceView? CurrentPlatformView => ((IElementHandler)this).PlatformView as GLSurfaceView;

    protected override PointCloudGlView CreatePlatformView() => new(Context);

    protected override void ConnectHandler(PointCloudGlView platformView)
    {
        base.ConnectHandler(platformView);
        if (VirtualView.IsResumeRequested) ResumeRendering(); // Resume() was called before this handler existed
    }

    protected override void DisconnectHandler(PointCloudGlView platformView)
    {
        // A GLSurfaceView that is never paused keeps its render thread and EGL context alive after the view is gone.
        PauseRendering();
        base.DisconnectHandler(platformView);
    }

    private void ResumeRendering() => CurrentPlatformView?.OnResume();

    private void PauseRendering() => CurrentPlatformView?.OnPause(); // blocks until the render thread is idle
}
