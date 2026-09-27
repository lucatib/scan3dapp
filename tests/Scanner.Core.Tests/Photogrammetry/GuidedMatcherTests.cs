using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;
using Xunit.Abstractions;

namespace Scanner.Core.Tests.Photogrammetry;

public class GuidedMatcherTests(ITestOutputHelper output)
{
    private const int Radius = 5;
    private static readonly Vector3 Target = new(0, 0.04f, 0);

    // A third of 1080p, with the phone's field of view.
    private static readonly CameraIntrinsics K = new(640, 360, 463, 463, 319.5f, 179.5f);

    private sealed record Photos(PhotoView A, float[] DepthA, PhotoView B, float[] DepthB);

    // An 8 cm box, a ball and a 12 cm post on a 60 cm table whose top is at y = 0, seen from two photos 10 degrees apart
    // on an arc 35 cm from the box and 30 cm above it: the objects hide some of the table from one photo but not the
    // other, and the table's edges leave one photo before the other.
    private static readonly Lazy<Photos> Pair = new(() =>
    {
        ISdf scene = new UnionSdf([
            new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.3f, 0.01f, 0.3f)),
            new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
            new SphereSdf(new Vector3(0.1f, 0.035f, -0.04f), 0.035f),
            new BoxSdf(new Vector3(-0.1f, 0.06f, 0f), new Vector3(0.012f, 0.06f, 0.012f)),
        ]);
        var (a, depthA) = Render(scene, K, -5f, 0.35f, 0.3f);
        var (b, depthB) = Render(scene, K, 5f, 0.35f, 0.3f);
        return new Photos(a, depthA, b, depthB);
    });

    private static (PhotoView View, float[] Depth) Render(ISdf scene, CameraIntrinsics k, float azimuthDegrees, float distance, float height)
    {
        float azimuth = azimuthDegrees * MathF.PI / 180f;
        var eye = Target + new Vector3(MathF.Sin(azimuth) * distance, height, -MathF.Cos(azimuth) * distance);
        var pose = Level(eye, Target);
        var (image, depth) = SyntheticPhotoRenderer.Render(scene, k, pose);
        return (new PhotoView(image, k, pose), depth);
    }

    // Like CameraPoses.LookAt but with the image's horizontal axis level (+Y is up in these scenes), as a phone is held.
    // LookAt's +Z up would roll these two photos 12 degrees apart, which no neighbouring phone photos are.
    private static Matrix4x4 Level(Vector3 eye, Vector3 target)
    {
        var forward = Vector3.Normalize(target - eye);
        var right = Vector3.Normalize(Vector3.Cross(forward, Vector3.UnitY));
        var down = Vector3.Cross(forward, right);
        return new Matrix4x4(
            right.X, right.Y, right.Z, 0,
            down.X, down.Y, down.Z, 0,
            forward.X, forward.Y, forward.Z, 0,
            eye.X, eye.Y, eye.Z, 1);
    }

    private enum Fate { Visible, Hidden, Outside }

    /// <summary>A's features; B's features (the true positions of A's visible points, B's own surface where A's
    /// hidden points would be, and random distractors, shuffled); and for each feature of A its fate in B and the
    /// index of its true match in B (-1 when it has none).</summary>
    private sealed record Setup(Feature[] A, Feature[] B, Fate[] Fates, int[] Truth)
    {
        public int Count(Fate fate) => Fates.Count(f => f == fate);
    }

    private static readonly Lazy<Setup> Features = new(() => Correspondences(Pair.Value));

    private static Setup Correspondences(Photos photos)
    {
        var k = photos.A.Intrinsics;
        var featuresA = new List<Feature>();
        var fates = new List<Fate>();
        var inB = new List<Vector2>();
        foreach (var f in TexturedPixels(photos.A.Image, Radius + 1, 10))
        {
            float depth = photos.DepthA[(int)f.Y * k.Width + (int)f.X];
            if (depth <= 0) continue;
            var camera = Pinhole.ToCamera(photos.B.CameraToWorld, Pinhole.BackProject(photos.A, f.X, f.Y, depth));
            Fate fate;
            if (!Pinhole.Project(k, camera, out float u, out float v)
                || u < Radius || v < Radius || u > k.Width - 1 - Radius || v > k.Height - 1 - Radius)
                fate = Fate.Outside;
            else if (Visibility(photos.DepthB, k.Width, u, v, camera.Z) is { } seen)
                fate = seen;
            else
                continue; // on a depth edge in B: neither clearly seen nor clearly hidden
            featuresA.Add(f);
            fates.Add(fate);
            inB.Add(new Vector2(u, v));
        }

        var truePositions = Enumerable.Range(0, featuresA.Count).Where(i => fates[i] == Fate.Visible).Select(i => inB[i]).ToList();
        // Other features keep 4 px from the true positions, as a detector keeps two features off one corner.
        bool FarFromTruth(Vector2 p) => truePositions.All(t => Vector2.DistanceSquared(t, p) >= 16f);

        var featuresB = new List<(Feature Feature, int Owner)>();
        for (int i = 0; i < featuresA.Count; i++)
        {
            if (fates[i] == Fate.Visible) featuresB.Add((new Feature(inB[i].X, inB[i].Y, 1f), i));
            // A corner on whatever hides the point, right where the point would be: the tempting wrong match.
            else if (fates[i] == Fate.Hidden && FarFromTruth(inB[i])) featuresB.Add((new Feature(inB[i].X, inB[i].Y, 1f), -1));
        }
        var random = new Random(7);
        for (int added = 0; added < truePositions.Count;)
        {
            var p = new Vector2(Radius + random.NextSingle() * (k.Width - 1 - 2 * Radius), Radius + random.NextSingle() * (k.Height - 1 - 2 * Radius));
            if (!FarFromTruth(p)) continue;
            featuresB.Add((new Feature(p.X, p.Y, 1f), -1));
            added++;
        }
        random.Shuffle(CollectionsMarshal.AsSpan(featuresB));

        var truth = Enumerable.Repeat(-1, featuresA.Count).ToArray();
        for (int j = 0; j < featuresB.Count; j++)
            if (featuresB[j].Owner >= 0) truth[featuresB[j].Owner] = j;
        return new Setup(featuresA.ToArray(), featuresB.Select(b => b.Feature).ToArray(), fates.ToArray(), truth);
    }

    // Seen when B's depth at that pixel is the point's own; hidden when B sees something clearly nearer all around it.
    private static Fate? Visibility(float[] depthB, int width, float u, float v, float z)
    {
        float nearest = depthB[(int)MathF.Round(v) * width + (int)MathF.Round(u)];
        if (MathF.Abs(nearest - z) <= 0.003f) return Fate.Visible;
        int x0 = (int)u, y0 = (int)v;
        for (int dy = 0; dy <= 1; dy++)
        for (int dx = 0; dx <= 1; dx++)
        {
            float d = depthB[(y0 + dy) * width + x0 + dx];
            if (d <= 0 || d > z - 0.01f) return null;
        }
        return Fate.Hidden;
    }

    // A stand-in for a corner detector: in each bucket, the pixel whose 5x5 structure tensor has the largest smaller
    // eigenvalue (texture in two directions, so the pixel can be located), kept when that is well above flat ground and
    // is the largest within 3 px, as a detector's non-maximum suppression keeps two features off the same corner.
    private static List<Feature> TexturedPixels(GrayImage image, int margin, int bucket)
    {
        const int Suppression = 3;
        int w = image.Width, h = image.Height;
        var corner = new float[w * h];
        for (int y = margin; y < h - margin; y++)
        for (int x = margin; x < w - margin; x++)
        {
            float sxx = 0, syy = 0, sxy = 0;
            for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++)
            {
                float gx = image[x + dx + 1, y + dy] - image[x + dx - 1, y + dy];
                float gy = image[x + dx, y + dy + 1] - image[x + dx, y + dy - 1];
                sxx += gx * gx;
                syy += gy * gy;
                sxy += gx * gy;
            }
            float half = (sxx - syy) / 2;
            corner[y * w + x] = (sxx + syy) / 2 - MathF.Sqrt(half * half + sxy * sxy);
        }

        var features = new List<Feature>();
        for (int by = margin; by < h - margin; by += bucket)
        for (int bx = margin; bx < w - margin; bx += bucket)
        {
            float best = 0;
            int bestX = -1, bestY = -1;
            for (int y = by; y < Math.Min(by + bucket, h - margin); y++)
            for (int x = bx; x < Math.Min(bx + bucket, w - margin); x++)
                if (corner[y * w + x] > best) { best = corner[y * w + x]; bestX = x; bestY = y; }
            if (best < 1000f) continue;
            bool peak = true;
            for (int y = Math.Max(0, bestY - Suppression); y <= Math.Min(h - 1, bestY + Suppression); y++)
            for (int x = Math.Max(0, bestX - Suppression); x <= Math.Min(w - 1, bestX + Suppression); x++)
                peak &= corner[y * w + x] <= best;
            if (peak) features.Add(new Feature(bestX, bestY, best));
        }
        return features;
    }

    private static Matrix4x4 Perturb(Matrix4x4 pose, Vector3 shift, Vector3 axis, float degrees)
    {
        var eye = pose.Translation;
        return pose * Matrix4x4.CreateTranslation(-eye) * Matrix4x4.CreateFromAxisAngle(Vector3.Normalize(axis), degrees * MathF.PI / 180f)
               * Matrix4x4.CreateTranslation(eye + shift);
    }

    private (int Correct, int Found) Score(Setup setup, List<FeatureMatch> matches, string label)
    {
        int correct = matches.Count(m => setup.Truth[m.FeatureA] == m.FeatureB);
        int visible = setup.Count(Fate.Visible);
        output.WriteLine($"{label}: {setup.A.Length} features in A ({visible} visible in B, {setup.Count(Fate.Hidden)} hidden, "
                         + $"{setup.Count(Fate.Outside)} outside), {setup.B.Length} in B; {matches.Count} matches, {correct} correct: "
                         + $"recall {(float)correct / visible:P1}, precision {(float)correct / Math.Max(1, matches.Count):P1}");
        return (correct, matches.Count);
    }

    [Fact]
    public void GuidedMatcher_finds_true_correspondences_with_exact_poses()
    {
        var photos = Pair.Value;
        var setup = Features.Value;

        var matches = GuidedMatcher.Match(0, photos.A, setup.A, 1, photos.B, setup.B);

        var (correct, found) = Score(setup, matches, "exact poses");
        Assert.True(setup.Count(Fate.Visible) >= 500, $"only {setup.Count(Fate.Visible)} visible features");
        Assert.True(correct >= setup.Count(Fate.Visible) * 0.9, $"found {correct} of {setup.Count(Fate.Visible)} correspondences");
        Assert.True(correct >= found * 0.98, $"{found - correct} of {found} matches are wrong");
        Assert.All(matches, m =>
        {
            Assert.Equal(0, m.ViewA);
            Assert.Equal(1, m.ViewB);
            Assert.InRange(m.Score, 0.8f, 1f);
        });
        Assert.Equal(matches.Select(m => m.FeatureA), matches.Select(m => m.FeatureA).Order());
        Assert.Equal(matches, GuidedMatcher.Match(0, photos.A, setup.A, 1, photos.B, setup.B));
    }

    // ARCore's pose error: B is off by 1 cm along its image's vertical axis and turned 0.5 degrees about its horizontal
    // axis, the two adding up: B's view of the true points moves across the epipolar segments by 14 px on average and
    // 17 px at most. The tolerance covers that.
    [Fact]
    public void GuidedMatcher_stays_precise_with_perturbed_poses_and_a_wider_tolerance()
    {
        var photos = Pair.Value;
        var setup = Features.Value;
        var pose = photos.B.CameraToWorld;
        var b = photos.B with { CameraToWorld = Perturb(pose, 0.01f * new Vector3(pose.M21, pose.M22, pose.M23), new Vector3(pose.M11, pose.M12, pose.M13), -0.5f) };

        var matches = GuidedMatcher.Match(0, photos.A, setup.A, 1, b, setup.B, new MatchOptions(EpipolarTolerancePx: 20f));

        var (correct, found) = Score(setup, matches, "perturbed poses");
        Assert.True(correct >= setup.Count(Fate.Visible) * 0.7, $"found only {correct} of {setup.Count(Fate.Visible)} correspondences");
        Assert.True(correct >= found * 0.95, $"{found - correct} of {found} matches are wrong");
    }

    // With a depth range around the scene's (0.36-0.73 m), as the pipeline will pass it from ARCore's points. Over the
    // default 0.05-3 m each segment crosses the whole photo, and one or two in a hundred of these features find a chance
    // correlation above 0.8 with some unrelated corner on it.
    [Fact]
    public void GuidedMatcher_leaves_points_hidden_in_or_outside_the_other_photo_unmatched()
    {
        var photos = Pair.Value;
        var setup = Features.Value;

        var matches = GuidedMatcher.Match(0, photos.A, setup.A, 1, photos.B, setup.B, new MatchOptions(MinDepth: 0.15f, MaxDepth: 1.5f));

        Assert.True(setup.Count(Fate.Hidden) >= 10, $"only {setup.Count(Fate.Hidden)} hidden features");
        Assert.True(setup.Count(Fate.Outside) >= 10, $"only {setup.Count(Fate.Outside)} features outside B");
        var wrong = matches.Where(m => setup.Fates[m.FeatureA] != Fate.Visible).ToList();
        Assert.True(wrong.Count == 0, $"{wrong.Count} features without a counterpart matched, e.g. A {wrong.FirstOrDefault()}");
    }

    // Two features of A a pixel apart both find the one feature of B on their own; only the true one gets it.
    [Fact]
    public void GuidedMatcher_mutual_check_rejects_a_many_to_one_match()
    {
        var photos = Pair.Value;
        var setup = Features.Value;
        var shift = new Vector2(1f, 0.5f);
        int chosen = Enumerable.Range(0, setup.A.Length).First(i =>
        {
            if (setup.Truth[i] < 0) return false;
            var near = setup.A[i] with { X = setup.A[i].X + shift.X, Y = setup.A[i].Y + shift.Y };
            return GuidedMatcher.Match(0, photos.A, [near], 1, photos.B, [setup.B[setup.Truth[i]]]).Count == 1;
        });
        var a = setup.A[chosen];
        var b = setup.B[setup.Truth[chosen]];

        var matches = GuidedMatcher.Match(0, photos.A, [a with { X = a.X + shift.X, Y = a.Y + shift.Y }, a], 1, photos.B, [b]);

        var match = Assert.Single(matches);
        Assert.Equal(1, match.FeatureA);
        Assert.Equal(0, match.FeatureB);
    }

    // Two features of B a third of a pixel apart correlate almost equally: the best is a coin toss, so no match.
    [Fact]
    public void GuidedMatcher_margin_rejects_two_near_equal_candidates()
    {
        var photos = Pair.Value;
        var setup = Features.Value;
        int chosen = Enumerable.Range(0, setup.A.Length).First(i => setup.Truth[i] >= 0
            && GuidedMatcher.Match(0, photos.A, [setup.A[i]], 1, photos.B, [setup.B[setup.Truth[i]]]).Count == 1);
        var a = setup.A[chosen];
        var b = setup.B[setup.Truth[chosen]];

        Assert.Empty(GuidedMatcher.Match(0, photos.A, [a], 1, photos.B, [b, b with { X = b.X + 0.33f }]));
    }

    // Identical photos: every feature matches itself, except where its patch leaves the image or has no texture.
    [Fact]
    public void GuidedMatcher_skips_features_whose_patch_is_outside_the_image_or_flat()
    {
        var k = new CameraIntrinsics(64, 48, 60, 60, 31.5f, 23.5f);
        var random = new Random(3);
        var pixels = new byte[k.Width * k.Height];
        random.NextBytes(pixels);
        for (int y = 30; y < 45; y++)
        for (int x = 30; x < 45; x++) pixels[y * k.Width + x] = 90;
        var view = new PhotoView(new GrayImage(k.Width, k.Height, pixels), k, CameraPoses.LookAt(new Vector3(0, 0, -0.5f), Vector3.Zero));
        Feature[] features =
        [
            new(12.3f, 20.7f, 1), // inside
            new(4.5f, 20f, 1), // patch half a pixel past the left edge
            new(k.Width - 1 - Radius, 12f, 1), // patch exactly touching the right edge
            new(Radius, Radius, 1), // patch exactly touching the top-left corner
            new(37f, 37f, 1), // flat patch
            new(20f, k.Height - 1 - Radius + 0.01f, 1), // patch a hundredth of a pixel past the bottom edge
        ];

        var matches = GuidedMatcher.Match(0, view, features, 1, view, features);

        Assert.Equal([0, 2, 3], matches.Select(m => m.FeatureA));
        Assert.All(matches, m => Assert.Equal(m.FeatureA, m.FeatureB));
        Assert.All(matches, m => Assert.Equal(1f, m.Score, 3));
    }

    // B stands 0.5 m ahead of A on A's optical axis, looking the same way. The ray of A's pixel 100 px right of centre
    // is at B's pixel 200 px right of centre at depth 1 m, and at 100 px left of centre at depth 0.25 m, which is
    // behind B. A's patch is pasted at both places in B: only the first is a candidate, so the match is clear.
    [Fact]
    public void GuidedMatcher_searches_only_the_part_of_the_ray_in_front_of_the_other_camera()
    {
        var k = new CameraIntrinsics(480, 120, 300, 300, 240, 60);
        var random = new Random(5);
        var pixelsA = new byte[k.Width * k.Height];
        var pixelsB = new byte[k.Width * k.Height];
        random.NextBytes(pixelsA);
        random.NextBytes(pixelsB);
        foreach (int x in new[] { 140, 440 })
            for (int dy = -7; dy <= 7; dy++)
            for (int dx = -7; dx <= 7; dx++)
                pixelsB[(60 + dy) * k.Width + x + dx] = pixelsA[(60 + dy) * k.Width + 340 + dx];
        var a = new PhotoView(new GrayImage(k.Width, k.Height, pixelsA), k, Matrix4x4.Identity);
        var b = new PhotoView(new GrayImage(k.Width, k.Height, pixelsB), k, Matrix4x4.CreateTranslation(0, 0, 0.5f));

        var matches = GuidedMatcher.Match(0, a, [new Feature(340, 60, 1)], 1, b, [new Feature(140, 60, 1), new Feature(440, 60, 1)]);

        var match = Assert.Single(matches);
        Assert.Equal(1, match.FeatureB);
        Assert.Equal(1f, match.Score, 3);
    }

    // B is A moved 10 cm right and 5 cm down, so the epipolar segment of A's pixel p runs from p - (10, 5) px (3 m deep)
    // up and left out of the photo (5 cm deep). A's patch, pasted into B at every pixel near the edge of the tolerance
    // band around that segment, must be found exactly when it lies within the band: the grid misses no candidate.
    [Theory]
    [InlineData(8f)]
    [InlineData(25f)]
    public void GuidedMatcher_candidates_are_exactly_the_features_within_the_tolerance(float tolerance)
    {
        var k = new CameraIntrinsics(320, 240, 300, 300, 159.5f, 119.5f);
        var random = new Random(9);
        var pixelsA = new byte[k.Width * k.Height];
        var background = new byte[k.Width * k.Height];
        random.NextBytes(pixelsA);
        random.NextBytes(background);
        var a = new PhotoView(new GrayImage(k.Width, k.Height, pixelsA), k, Matrix4x4.Identity);
        var pose = Matrix4x4.CreateTranslation(0.1f, 0.05f, 0);
        var p = new Vector2(250, 180);
        var baseline = new Vector2(0.1f, 0.05f) * k.Fx;
        Vector2 near = p - baseline / 3f, far = p - baseline / 0.05f;
        var options = new MatchOptions(EpipolarTolerancePx: tolerance);

        int inside = 0, outside = 0;
        for (int y = Radius + 1; y < k.Height - Radius - 1; y++)
        for (int x = Radius + 1; x < k.Width - Radius - 1; x++)
        {
            float distance = DistanceToSegment(new Vector2(x, y), near, far);
            if (MathF.Abs(distance - tolerance) is > 1.5f or < 0.05f) continue;
            var pixelsB = (byte[])background.Clone();
            for (int dy = -Radius - 1; dy <= Radius + 1; dy++)
            for (int dx = -Radius - 1; dx <= Radius + 1; dx++)
                pixelsB[(y + dy) * k.Width + x + dx] = pixelsA[((int)p.Y + dy) * k.Width + (int)p.X + dx];
            var b = new PhotoView(new GrayImage(k.Width, k.Height, pixelsB), k, pose);

            bool found = GuidedMatcher.Match(0, a, [new Feature(p.X, p.Y, 1)], 1, b, [new Feature(x, y, 1)], options).Count == 1;

            Assert.True(found == distance <= tolerance, $"patch at ({x}, {y}), {distance:F2} px from the segment: found {found}");
            if (found) inside++;
            else outside++;
        }
        Assert.True(inside >= 100 && outside >= 100, $"only {inside} positions inside and {outside} outside tested");
    }

    private static float DistanceToSegment(Vector2 q, Vector2 p0, Vector2 p1)
    {
        var d = p1 - p0;
        float t = Math.Clamp(Vector2.Dot(q - p0, d) / d.LengthSquared(), 0f, 1f);
        return Vector2.Distance(q, p0 + t * d);
    }

    [Fact]
    public void GuidedMatcher_rejects_invalid_options()
    {
        var k = new CameraIntrinsics(16, 16, 20, 20, 7.5f, 7.5f);
        var view = new PhotoView(new GrayImage(k.Width, k.Height, new byte[k.Width * k.Height]), k, Matrix4x4.Identity);
        Feature[] none = [];
        Assert.ThrowsAny<ArgumentException>(() => GuidedMatcher.Match(0, view, none, 1, view, none, new MatchOptions(EpipolarTolerancePx: 0)));
        Assert.ThrowsAny<ArgumentException>(() => GuidedMatcher.Match(0, view, none, 1, view, none, new MatchOptions(PatchRadius: 0)));
        Assert.ThrowsAny<ArgumentException>(() => GuidedMatcher.Match(0, view, none, 1, view, none, new MatchOptions(MinDepth: 0)));
        Assert.ThrowsAny<ArgumentException>(() => GuidedMatcher.Match(0, view, none, 1, view, none, new MatchOptions(MinDepth: 1, MaxDepth: 1)));
    }

    // The real workload: 1080p photos, 2000 features each, a tolerance wide enough for ARCore's poses, and the whole
    // default depth range, so each epipolar segment crosses most of the photo.
    [Fact]
    public void GuidedMatcher_matches_2000_by_2000_features_at_1080p_well_under_a_second()
    {
        var k = new CameraIntrinsics(1920, 1080, 1390, 1390, 959.5f, 539.5f);
        ISdf scene = new UnionSdf([
            new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(1f, 0.01f, 1f)),
            new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
        ]);
        var (a, depthA) = Render(scene, k, -5f, 0.3f, 0.45f);
        var (b, _) = Render(scene, k, 5f, 0.3f, 0.45f);
        var random = new Random(11);
        var featuresA = new Feature[2000];
        var featuresB = new List<Feature>();
        for (int i = 0; i < featuresA.Length; i++)
        {
            featuresA[i] = new Feature(Radius + random.NextSingle() * (k.Width - 1 - 2 * Radius), Radius + random.NextSingle() * (k.Height - 1 - 2 * Radius), 1f);
            float depth = depthA[(int)MathF.Round(featuresA[i].Y) * k.Width + (int)MathF.Round(featuresA[i].X)];
            if (i % 4 == 0 || depth <= 0) continue;
            var camera = Pinhole.ToCamera(b.CameraToWorld, Pinhole.BackProject(a, featuresA[i].X, featuresA[i].Y, depth));
            if (Pinhole.Project(k, camera, out float u, out float v)) featuresB.Add(new Feature(u, v, 1f));
        }
        while (featuresB.Count < 2000)
            featuresB.Add(new Feature(Radius + random.NextSingle() * (k.Width - 1 - 2 * Radius), Radius + random.NextSingle() * (k.Height - 1 - 2 * Radius), 1f));
        var options = new MatchOptions(EpipolarTolerancePx: 30f);

        var matches = GuidedMatcher.Match(0, a, featuresA, 1, b, featuresB, options);
        var times = new List<double>();
        for (int run = 0; run < 3; run++)
        {
            var watch = Stopwatch.StartNew();
            GuidedMatcher.Match(0, a, featuresA, 1, b, featuresB, options);
            times.Add(watch.Elapsed.TotalMilliseconds);
        }

        output.WriteLine($"2000 x {featuresB.Count} features at 1080p, tolerance 30 px: {matches.Count} matches; "
                         + $"runs {string.Join(", ", times.Select(t => $"{t:F0} ms"))}");
        Assert.True(matches.Count >= 500, $"only {matches.Count} matches");
        Assert.True(times.Min() < 1000, $"best of three runs took {times.Min():F0} ms");
    }
}
