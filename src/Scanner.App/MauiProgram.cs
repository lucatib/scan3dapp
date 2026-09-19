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
				handlers.AddHandler<Scanner.App.Controls.PointCloudView, Scanner.App.Droid.Handlers.PointCloudViewHandler>();
#elif WINDOWS
				handlers.AddHandler<Scanner.App.Controls.ArScanView, Scanner.App.WinUI.Handlers.ArScanViewHandler>();
				handlers.AddHandler<Scanner.App.Controls.PointCloudView, Scanner.App.WinUI.Handlers.PointCloudViewHandler>();
#endif
			});

		builder.Services.AddSingleton<Scanner.App.Services.SessionStore>();
		builder.Services.AddTransient<Scanner.App.Pages.ScanPage>();
		builder.Services.AddTransient<Scanner.App.Pages.PreviewPage>();

#if DEBUG
		builder.Logging.AddDebug();
#endif

		return builder.Build();
	}
}
