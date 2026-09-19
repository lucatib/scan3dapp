using Microsoft.Maui.Handlers;
using Microsoft.UI.Xaml.Controls;
using Scanner.App.Controls;

namespace Scanner.App.WinUI.Handlers;

/// <summary>Shows a short message where an Android-only view would be.</summary>
public sealed class ArScanViewHandler() : ViewHandler<ArScanView, TextBlock>(ViewMapper)
{
    protected override TextBlock CreatePlatformView() =>
        new() { Text = "AR scanning is available on Android only.", Margin = new Microsoft.UI.Xaml.Thickness(16) };
}

/// <inheritdoc cref="ArScanViewHandler"/>
public sealed class PointCloudViewHandler() : ViewHandler<PointCloudView, TextBlock>(ViewMapper)
{
    protected override TextBlock CreatePlatformView() =>
        new() { Text = "The 3D preview is available on Android only for now.", Margin = new Microsoft.UI.Xaml.Thickness(16) };
}
