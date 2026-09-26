using System.Globalization;
using Scanner.Desktop;

const string Usage = """
    Usage: scan3d-process <session folder | file.scan> [options]
      --out <folder>        where to write the results (default: <session>/desktop)
      --downscale <n>       average the 1920x1080 photos down by n before stereo (default 2; 1 = full size)
      --references <n>      photos that get a depth map (default 30)
      --depths <n>          depth hypotheses per depth map (default 128)
    """;

if (args.Length == 0 || args[0] is "-h" or "--help")
{
    Console.WriteLine(Usage);
    return args.Length == 0 ? 1 : 0;
}

try
{
    string? output = null;
    int downscale = 2;
    var options = new ProcessOptions();
    for (int i = 1; i < args.Length; i++)
    {
        string value = i + 1 < args.Length ? args[i + 1] : throw new ArgumentException($"{args[i]} needs a value.");
        switch (args[i])
        {
            case "--out": output = value; break;
            case "--downscale": downscale = int.Parse(value, CultureInfo.InvariantCulture); break;
            case "--references": options = options with { ReferenceViews = int.Parse(value, CultureInfo.InvariantCulture) }; break;
            case "--depths": options = options with { DepthSamples = int.Parse(value, CultureInfo.InvariantCulture) }; break;
            default: throw new ArgumentException($"Unknown option {args[i]}.");
        }
        i++;
    }

    string directory = SessionLoader.Resolve(args[0]);
    var session = SessionLoader.Load(directory, downscale);
    Console.WriteLine($"Loaded {session.Manifest.Id}: {session.Photos.Count} photos, {session.Manifest.FrameCount} depth frames");
    var result = SessionProcessor.Process(session, output ?? Path.Combine(directory, "desktop"), options, Console.Out);
    Console.WriteLine(result.Isolated ? "Done: the piece is in piece.ply." : "Done, but the piece could not be isolated: piece.ply holds every point.");
    return 0;
}
catch (Exception ex) when (ex is ArgumentException or FormatException or IOException or InvalidDataException)
{
    Console.Error.WriteLine(ex.Message);
    Console.Error.WriteLine(Usage);
    return 2;
}
