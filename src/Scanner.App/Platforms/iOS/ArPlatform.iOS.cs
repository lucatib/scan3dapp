using ARKit;

namespace Scanner.App.Services;

public static partial class ArPlatform
{
    public static partial async Task<string?> PrepareAsync()
    {
        if (await Permissions.RequestAsync<Permissions.Camera>() != PermissionStatus.Granted)
            return "Camera permission is required to scan.";
        if (!ARWorldTrackingConfiguration.IsSupported) return "This device does not support ARKit.";
        if (!ARWorldTrackingConfiguration.SupportsSceneReconstruction(ARSceneReconstruction.Mesh)
            || !ARConfiguration.SupportsFrameSemantics(ARFrameSemantics.SceneDepth))
            return "Scanning on iPhone needs the LiDAR sensor (iPhone 12 Pro or later Pro models).";
        return null;
    }
}
