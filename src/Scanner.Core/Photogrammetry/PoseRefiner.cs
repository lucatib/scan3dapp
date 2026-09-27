using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <param name="MaxCorners">Corners per photo. On a 90-photo book scan, 800 corners with 3 partners gave ~290
/// observations per photo and the same photo-consistency as 3000 with 8, in half the time.</param>
/// <param name="MinResponseRatio">Weakest corner kept, as a fraction of the photo's strongest.</param>
/// <param name="PairAngleDegrees">Two photos are matched when their rays to the target meet within this angle.</param>
/// <param name="PartnersPerPhoto">Each photo is matched with at most this many others, the closest in angle.</param>
/// <param name="CoarseTolerance">Round 1 epipolar tolerance, as a fraction of the focal length: wide enough for
/// ARCore's pose error (about a degree).</param>
/// <param name="FineTolerance">Round 2 epipolar tolerance with the round 1 poses, as a fraction of the focal length.</param>
/// <param name="CoarseHuber">Round 1 Huber threshold, as a fraction of the focal length.</param>
/// <param name="FineHuber">Round 2 Huber threshold, as a fraction of the focal length.</param>
/// <param name="MaxIterations">Bundle adjustment iterations per round; real scans converge in about 30.</param>
/// <param name="MinObservationsPerPhoto">Below this average the refinement is not trusted and ARCore's poses stay.</param>
public sealed record PoseRefinementOptions(int MaxCorners = 800, float MinResponseRatio = 1e-4f,
    float PairAngleDegrees = 15f, int PartnersPerPhoto = 3, float CoarseTolerance = 0.03f, float FineTolerance = 0.005f,
    float CoarseHuber = 0.003f, float FineHuber = 0.0015f, int MaxIterations = 30, int MinObservationsPerPhoto = 30);

/// <param name="Views">The photos with refined poses (ARCore's when <paramref name="Applied"/> is false).</param>
/// <param name="InitialRmsPx">Reprojection RMS of round 2's tracks before its adjustment.</param>
/// <param name="FinalRmsPx">Reprojection RMS after round 2.</param>
/// <param name="Observations">Observations used in round 2.</param>
/// <param name="Applied">False when there were too few observations to trust the result.</param>
public sealed record PoseRefinement(PhotoView[] Views, double InitialRmsPx, double FinalRmsPx, int Observations, bool Applied);

/// <summary>
/// Makes ARCore's poses a starting guess only: photos are matched to each other (corners, guided by the ARCore poses),
/// the matches triangulated, and poses and points adjusted together so the points reproject onto the corners
/// (bundle adjustment, with ARCore's camera centres as soft priors that keep its scale and gravity). Two rounds: the
/// first with a wide epipolar tolerance that covers ARCore's error, the second re-matched tightly with the first
/// round's poses. Tolerances scale with the focal length, so the same options work at any photo resolution.
/// </summary>
public static class PoseRefiner
{
    public static PoseRefinement Refine(IReadOnlyList<PhotoView> views, Vector3 target, PoseRefinementOptions? options = null,
        TextWriter? log = null)
    {
        options ??= new PoseRefinementOptions();
        var original = views.ToArray();
        if (views.Count < 3) return new PoseRefinement(original, 0, 0, 0, false);

        var clock = System.Diagnostics.Stopwatch.StartNew();
        var corners = new CornerOptions(MaxCorners: options.MaxCorners, MinResponseRatio: options.MinResponseRatio);
        var features = new List<Feature>[views.Count];
        Parallel.For(0, views.Count, i => features[i] = CornerDetector.Detect(views[i].Image, corners));
        var pairs = Pairs(views, target, options);
        log?.WriteLine($"  poses: corners {features.Average(c => c.Count):F0} per photo, {clock.ElapsedMilliseconds} ms");
        float f = views[0].Intrinsics.Fx;

        var round1 = Round(original, features, pairs, target, options.CoarseTolerance * f, options.CoarseHuber * f, options);
        if (round1 is null) return Rejected(original, log, "round 1 had too few tracks");
        var (views1, _) = round1.Value;
        log?.WriteLine($"  poses: round 1 done, {clock.ElapsedMilliseconds} ms");

        var round2 = Round(views1, features, pairs, target, options.FineTolerance * f, options.FineHuber * f, options);
        if (round2 is null) return Rejected(original, log, "round 2 had too few tracks");
        var (views2, result) = round2.Value;

        int observations = result.Observations;
        log?.WriteLine($"  poses: {pairs.Count} pairs, {observations} observations ({observations / views.Count} per photo), "
                       + $"RMS {result.Bundle.InitialRmsPx:F2} -> {result.Bundle.FinalRmsPx:F2} px, {result.Bundle.Iterations} iterations, {clock.ElapsedMilliseconds} ms");
        if (observations < options.MinObservationsPerPhoto * views.Count)
            return Rejected(original, log, "too few observations per photo");
        return new PoseRefinement(views2, result.Bundle.InitialRmsPx, result.Bundle.FinalRmsPx, observations, true);
    }

    private sealed record RoundResult(BundleResult Bundle, int Observations);

    private static (PhotoView[] Views, RoundResult Result)? Round(PhotoView[] views, List<Feature>[] features,
        List<(int A, int B)> pairs, Vector3 target, float tolerance, float huber, PoseRefinementOptions options)
    {
        var perPair = new List<FeatureMatch>[pairs.Count];
        Parallel.For(0, pairs.Count, p =>
        {
            var (a, b) = pairs[p];
            float distance = Vector3.Distance(views[a].CameraToWorld.Translation, target);
            perPair[p] = GuidedMatcher.Match(a, views[a], features[a], b, views[b], features[b],
                new MatchOptions(EpipolarTolerancePx: tolerance, MinDepth: 0.05f, MaxDepth: MathF.Max(1f, 3 * distance)));
        });
        var observations = TrackBuilder.Build(features, perPair.SelectMany(m => m));
        var tracks = new Track?[observations.Count];
        Parallel.For(0, observations.Count, t =>
        {
            if (Triangulation.Triangulate(views, observations[t], maxErrorPx: tolerance, minAngleDegrees: 1f) is { } point)
                tracks[t] = new Track(point, observations[t]);
        });
        var kept = tracks.OfType<Track>().ToList();
        if (kept.Count < 50) return null;

        var bundle = BundleAdjuster.Adjust(views, kept, new BundleOptions(MaxIterations: options.MaxIterations, HuberPx: huber));
        var refined = views.Select((v, i) => v with { CameraToWorld = bundle.CameraToWorld[i] }).ToArray();
        return (refined, new RoundResult(bundle, kept.Sum(t => t.Observations.Length)));
    }

    /// <summary>Each photo with its closest partners by the angle between their rays to the target.</summary>
    private static List<(int A, int B)> Pairs(IReadOnlyList<PhotoView> views, Vector3 target, PoseRefinementOptions options)
    {
        var pairs = new HashSet<(int, int)>();
        for (int i = 0; i < views.Count; i++)
        {
            var toI = Vector3.Normalize(target - views[i].CameraToWorld.Translation);
            var partners = Enumerable.Range(0, views.Count).Where(j => j != i)
                .Select(j => (j, angle: MathF.Acos(Math.Clamp(Vector3.Dot(toI,
                    Vector3.Normalize(target - views[j].CameraToWorld.Translation)), -1f, 1f)) * 180f / MathF.PI))
                .Where(p => p.angle <= options.PairAngleDegrees)
                .OrderBy(p => p.angle)
                .Take(options.PartnersPerPhoto);
            foreach (var (j, _) in partners) pairs.Add((Math.Min(i, j), Math.Max(i, j)));
        }
        return pairs.Order().ToList();
    }

    private static PoseRefinement Rejected(PhotoView[] original, TextWriter? log, string reason)
    {
        log?.WriteLine($"  poses: keeping ARCore's, {reason}");
        return new PoseRefinement(original, 0, 0, 0, false);
    }
}
