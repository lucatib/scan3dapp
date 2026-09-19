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

    public PointCloudViewHandler() : base(PropertyMapper)
    {
    }

    protected override PointCloudGlView CreatePlatformView() => new(Context);
}
