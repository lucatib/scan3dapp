using Microsoft.Extensions.Logging;

namespace Scanner.App;

public static class MauiProgram
{
	public static MauiApp CreateMauiApp()
	{
		var builder = MauiApp.CreateBuilder();
		builder
			.UseMauiApp<App>()
			.ConfigureFonts(fonts =>
			{
				fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
				fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
			})
			.ConfigureMauiHandlers(handlers =>
			{
#if ANDROID
				handlers.AddHandler<Scanner.App.Controls.ArScanView, Scanner.App.Droid.Handlers.ArScanViewHandler>();
#elif WINDOWS
				handlers.AddHandler<Scanner.App.Controls.ArScanView, Scanner.App.WinUI.Handlers.ArScanViewHandler>();
#endif
			});

		builder.Services.AddSingleton<Scanner.App.Services.SessionStore>();
		builder.Services.AddTransient<Scanner.App.Pages.ScanPage>();

#if DEBUG
		builder.Logging.AddDebug();
#endif

		return builder.Build();
	}
}
