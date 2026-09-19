namespace Scanner.App;

public partial class AppShell : Shell
{
	public AppShell()
	{
		InitializeComponent();
		Routing.RegisterRoute("preview", typeof(Scanner.App.Pages.PreviewPage));
	}
}
