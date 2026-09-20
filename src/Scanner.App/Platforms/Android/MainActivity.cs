using Android.App;
using Android.Content.PM;
using Android.OS;

namespace Scanner.App;

// Portrait-locked so the AR view never has to handle a rotation mid-scan; the renderer still reads the real
// display rotation, because portrait is not rotation 0 on a landscape-natural device.
[Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, LaunchMode = LaunchMode.SingleTop, ScreenOrientation = ScreenOrientation.Portrait, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density)]
public class MainActivity : MauiAppCompatActivity
{
}
