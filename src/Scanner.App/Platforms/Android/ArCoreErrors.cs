using Google.AR.Core.Exceptions;

namespace Scanner.App.Droid;

/// <summary>
/// A readable sentence for an ARCore or camera failure. ARCore throws Java exceptions without a message, and a Release
/// build (trimmed resource strings) turns that into "Exception_WasThrown, Java.Lang.RuntimeException" — not something
/// to show the user.
/// </summary>
internal static class ArCoreErrors
{
    public static string Describe(Exception ex) => ex switch
    {
        UnavailableUserDeclinedInstallationException =>
            "Google Play Services for AR is not installed: install it from the Play Store, then open this mode again.",
        UnavailableArcoreNotInstalledException or UnavailableApkTooOldException =>
            "Install or update Google Play Services for AR, then open this mode again.",
        UnavailableSdkTooOldException => "This app's ARCore is too old for this phone: update the app.",
        UnavailableDeviceNotCompatibleException => "This device does not support ARCore.",
        CameraNotAvailableException => "The camera is not available: close other apps that use it and try again.",
        _ when HasRealMessage(ex) => ex.Message,
        _ => $"ARCore could not open the camera ({ex.GetType().Name}).",
    };

    private static bool HasRealMessage(Exception ex) =>
        !string.IsNullOrWhiteSpace(ex.Message) && !ex.Message.StartsWith("Exception_WasThrown", StringComparison.Ordinal);
}
