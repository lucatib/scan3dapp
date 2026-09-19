using Google.AR.Core;

namespace Scanner.App.Services;

public static partial class ArPlatform
{
    // ARCore's install pattern: prompt the user once; on the next attempt (after returning from the Play Store)
    // pass false so a declined install fails instead of prompting again.
    private static bool s_installRequested;

    public static partial async Task<string?> PrepareAsync()
    {
        if (await Permissions.RequestAsync<Permissions.Camera>() != PermissionStatus.Granted)
            return "Camera permission is required to scan.";

        var activity = Platform.CurrentActivity;
        if (activity is null) return "The app is not ready yet, try again.";

        var availability = ArCoreApk.Instance!.CheckAvailability(activity)!;
        for (int i = 0; i < 20 && availability.IsTransient; i++)
        {
            await Task.Delay(250);
            availability = ArCoreApk.Instance!.CheckAvailability(activity)!;
        }
        // An unknown availability (e.g. offline) is not a verdict: RequestInstall below decides.
        if (availability.IsUnsupported) return "This device does not support ARCore.";

        try
        {
            var status = ArCoreApk.Instance!.RequestInstall(activity, !s_installRequested)!;
            if (status.Equals(ArCoreApk.InstallStatus.InstallRequested!))
            {
                s_installRequested = true;
                return "Install or update Google Play Services for AR, then come back to the app.";
            }
        }
        catch (Exception ex)
        {
            return $"ARCore is not available: {ex.Message}";
        }
        return null;
    }
}
