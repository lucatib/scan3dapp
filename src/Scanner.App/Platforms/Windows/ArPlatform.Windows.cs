namespace Scanner.App.Services;

public static partial class ArPlatform
{
    public static partial Task<string?> PrepareAsync() =>
        Task.FromResult<string?>("Scanning needs an Android phone with ARCore depth support.");
}
