namespace Scanner.App;

public partial class AppShell : Shell
{
	public AppShell()
	{
		InitializeComponent();
		Routing.RegisterRoute("scan", typeof(Scanner.App.Pages.ScanPage));
		Routing.RegisterRoute("matturntable", typeof(Scanner.App.Pages.MatTurntablePage));
		Routing.RegisterRoute("preview", typeof(Scanner.App.Pages.PreviewPage));
		Routing.RegisterRoute("turntable", typeof(Scanner.App.Pages.TurntablePage));
		Routing.RegisterRoute("lidarturntable", typeof(Scanner.App.Pages.LidarTurntablePage));
	}
}
