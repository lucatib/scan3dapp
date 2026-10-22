using Scanner.App.Services;
using Scanner.Core.Turntable;

namespace Scanner.App.Pages;

/// <summary>The start page: each way of scanning is its own mode, offered where the phone can do it.</summary>
public sealed class ModePage : ContentPage
{
    public ModePage()
    {
        Title = "Scan3D";
        var modes = new VerticalStackLayout { Padding = 16, Spacing = 12 };
        modes.Add(Mode(
            ArPlatform.HasDepthSensor ? "Handheld · LiDAR" : "Handheld · AR",
            ArPlatform.HasDepthSensor
                ? "Walk around the piece on a table. The LiDAR mesh is the shape, the photos colour it."
                : "Walk around the piece on a table. ARCore finds it, the photos measure it.",
            "scan"));
        if (ArPlatform.HasFrontDepthCamera)
            modes.Add(Mode("Turntable · TrueDepth",
                "Phone standing still, screen towards the piece about 25 cm away; the piece turns. For small pieces.",
                "turntable"));
        if (ArPlatform.HasMatTurntable)
        {
            modes.Add(Mode("Turntable · printed mat",
                "Phone on a stand, back camera on the mat; the piece turns on the printed mat. Exact poses and scale.",
                "matturntable"));
            var print = new Button { Text = "Get the mat to print (A4, 100 %)" };
            print.Clicked += OnPrintClicked;
            modes.Add(print);
        }
        Content = new ScrollView { Content = modes };
    }

    private static View Mode(string title, string description, string route)
    {
        var button = new Button { Text = title, FontSize = 18 };
        button.Clicked += async (_, _) => await Shell.Current.GoToAsync(route);
        return new VerticalStackLayout
        {
            Spacing = 4,
            Children = { button, new Label { Text = description, FontSize = 13, Opacity = 0.75 } },
        };
    }

    /// <summary>Writes the mat as an SVG and hands it to the share sheet (to send to a computer, or print).</summary>
    private async void OnPrintClicked(object? sender, EventArgs e)
    {
        try
        {
            string path = Path.Combine(FileSystem.Current.CacheDirectory, "scan3d-turntable-mat.svg");
            await File.WriteAllTextAsync(path, TurntableMat.Svg());
            await Share.Default.RequestAsync(new ShareFileRequest("Scan3D turntable mat", new ShareFile(path, "image/svg+xml")));
        }
        catch (Exception ex)
        {
            await DisplayAlertAsync("Mat", $"Could not share the mat: {ex.Message}", "OK");
        }
    }
}
