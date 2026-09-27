using Android.App;
using Android.Runtime;

namespace Scanner.App;

// Debuggable in Release too: scan sessions live in app-private storage and are pulled with adb run-as for analysis.
[Application(Debuggable = true)]
public class MainApplication : MauiApplication
{
	public MainApplication(IntPtr handle, JniHandleOwnership ownership)
		: base(handle, ownership)
	{
	}

	protected override MauiApp CreateMauiApp() => MauiProgram.CreateMauiApp();
}
