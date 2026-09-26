using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;
using Scanner.Core.Photogrammetry;

namespace Scanner.Desktop;

/// <param name="ReferenceViews">Photos that get a depth map, spread evenly over the capture.</param>
/// <param name="DepthSamples">Depth hypotheses per depth map.</param>
/// <param name="RegionRadius">Points farther than this from the target are dropped, as on the phone.</param>
public sealed record ProcessOptions(int ReferenceViews = 30, int DepthSamples = 128, float RegionRadius = 0.30f);

public sealed record ProcessResult(int PhotoPoints, int DepthPoints, int MergedPoints, int PiecePoints,
    float? SupportPlaneHeight, bool Isolated);

/// <summary>The full-quality reconstruction of one session, the PC half of the phone's preview: dense photo points,
/// ARCore depth points from the recorded frames, the merge, and the cut from the table.</summary>
public static class SessionProcessor
{
    private const float VoxelSize = 0.005f;
    private const int MinIsolatedPoints = 200;

    public static ProcessResult Process(LoadedSession session, string outputDirectory, ProcessOptions? options, TextWriter log)
    {
        options ??= new ProcessOptions();
        if (session.Target is not { } target)
            throw new InvalidDataException("The session has no target point: it was never started, so there is nothing to reconstruct.");
        Directory.CreateDirectory(outputDirectory);
        var clock = Stopwatch.StartNew();

        var photoPoints = PhotoReconstruction.DensePoints(session.Photos, target, new ReconstructionOptions(
            ReferenceViews: options.ReferenceViews, RegionRadius: options.RegionRadius,
            Stereo: new StereoOptions(DepthSamples: options.DepthSamples)));
        log.WriteLine($"Photo stereo: {photoPoints.Count} points from {Math.Min(options.ReferenceViews, session.Photos.Count)} depth maps, {clock.Elapsed.TotalSeconds:F1} s");

        // ARCore depth, accumulated the way the phone did it.
        var accumulator = new VoxelPointAccumulator(VoxelSize);
        var filter = new DepthFilter(Region: new ScanRegion(target, options.RegionRadius));
        foreach (var (frame, confidence) in ScanSessionReader.ReadFrames(session.Directory))
        {
            var points = new List<Vector3>();
            DepthBackProjector.Project(frame, confidence, filter, points);
            accumulator.AddRange(points);
        }
        var depthPoints = accumulator.Snapshot();

        var merged = SourceMerge.Merge(photoPoints, depthPoints, VoxelSize);
        // The table is found again in this cloud: it is the better one, and the phone's height came from its preview.
        float? plane = SupportPlaneFinder.Find(merged) ?? session.Manifest.SupportPlaneHeight;
        var piece = plane is null ? merged : ObjectIsolator.Isolate(merged, target, plane, 2 * VoxelSize);
        bool isolated = plane is not null && piece.Length >= MinIsolatedPoints;
        if (!isolated) piece = merged;
        log.WriteLine($"Depth points {depthPoints.Length}, merged {merged.Length}, table at {Format(plane)}, piece {piece.Length}{(isolated ? "" : " (not isolated)")}");

        WritePly(Path.Combine(outputDirectory, "photo-points.ply"), photoPoints);
        WritePly(Path.Combine(outputDirectory, "merged.ply"), merged);
        WritePly(Path.Combine(outputDirectory, "piece.ply"), piece);
        var result = new ProcessResult(photoPoints.Count, depthPoints.Length, merged.Length, piece.Length, plane, isolated);
        File.WriteAllLines(Path.Combine(outputDirectory, "report.txt"),
        [
            $"Session: {session.Manifest.Id} ({session.Manifest.Device})",
            $"Photos: {session.Photos.Count} at {session.Photos.FirstOrDefault()?.Image.Width}x{session.Photos.FirstOrDefault()?.Image.Height}",
            $"Options: {options}",
            $"Photo points: {result.PhotoPoints}",
            $"Depth points: {result.DepthPoints}",
            $"Merged points: {result.MergedPoints}",
            $"Table height: {Format(plane)}",
            $"Piece points: {result.PiecePoints} ({(isolated ? "isolated" : "not isolated: all points kept")})",
            $"Time: {clock.Elapsed.TotalSeconds:F1} s",
        ]);
        return result;
    }

    private static string Format(float? height) => height is { } h ? h.ToString("F4", CultureInfo.InvariantCulture) + " m" : "none";

    private static void WritePly(string path, IReadOnlyList<Vector3> points)
    {
        using var file = File.Create(path);
        PlyWriter.Write(file, points);
    }
}
