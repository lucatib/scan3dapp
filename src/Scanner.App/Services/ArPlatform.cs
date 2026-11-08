namespace Scanner.App.Services;

public static partial class ArPlatform
{
    /// <summary>Requests camera permission and checks/installs ARCore.
    /// Returns null when scanning can start, otherwise a user-facing reason.</summary>
    public static partial Task<string?> PrepareAsync();

    /// <summary>True where a depth sensor measures the surface (LiDAR iPhones): its mesh is the geometry, and the
    /// photos only texture it.</summary>
    public static bool HasDepthSensor => OperatingSystem.IsIOS();

    /// <summary>True where the front camera measures depth (TrueDepth on iPhone): the turntable scan is offered.</summary>
    public static bool HasFrontDepthCamera => OperatingSystem.IsIOS();

    /// <summary>True where the rear camera measures depth (LiDAR on iPhone): the turntable scan on the dot-ring platter
    /// with the back of the phone towards it is offered.</summary>
    public static bool HasLidarTurntable => OperatingSystem.IsIOS();

    /// <summary>True where the printed-mat turntable scan is available (Android, its camera through ARCore).</summary>
    public static bool HasMatTurntable => OperatingSystem.IsAndroid();
}
