namespace Scanner.App.Services;

public static partial class ArPlatform
{
    /// <summary>Requests camera permission and checks/installs ARCore.
    /// Returns null when scanning can start, otherwise a user-facing reason.</summary>
    public static partial Task<string?> PrepareAsync();
}
