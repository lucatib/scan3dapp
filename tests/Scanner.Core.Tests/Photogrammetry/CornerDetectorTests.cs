using System.Diagnostics;
using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;
using Xunit.Abstractions;

namespace Scanner.Core.Tests.Photogrammetry;

public class CornerDetectorTests(ITestOutputHelper output)
{
    private const float Dark = 40, Bright = 200;

    private static readonly Lazy<GrayImage> Textured = new(TexturedPhoto);

    // Pixel centres are at integers and pixel x covers [x - 0.5, x + 0.5], so a square filling pixels 50..109 has its
    // edges at 49.5 and 109.5: its corners lie where four pixels meet, not on any pixel.
    [Fact]
    public void Corners_of_a_square_are_found_to_a_tenth_of_a_pixel()
    {
        var pixels = Enumerable.Repeat((byte)Dark, 160 * 120).ToArray();
        for (int y = 30; y < 90; y++)
        for (int x = 50; x < 110; x++)
            pixels[y * 160 + x] = (byte)Bright;

        var corners = CornerDetector.Detect(new GrayImage(160, 120, pixels));

        AssertCorners(corners, [new(49.5f, 29.5f), new(109.5f, 29.5f), new(49.5f, 89.5f), new(109.5f, 89.5f)], 0.1f);
    }

    // The same with edges anywhere between pixels: an edge pixel is shaded by how much of it the square covers, as a
    // camera pixel integrates the light falling on it. A partly covered pixel pulls the estimate a little: by up to
    // 0.19 px over a sweep of 100 placements.
    [Theory]
    [InlineData(50.3f, 30.8f, 110.65f, 90.15f)]
    [InlineData(50.5f, 30.1f, 110.25f, 90.9f)]
    [InlineData(49.75f, 30.45f, 109.95f, 89.6f)]
    public void Corners_between_pixel_boundaries_are_found_to_a_fifth_of_a_pixel(float x0, float y0, float x1, float y1)
    {
        var image = Rectangle(160, 120, x0, y0, x1, y1);

        var corners = CornerDetector.Detect(image);

        AssertCorners(corners, [new(x0, y0), new(x1, y0), new(x0, y1), new(x1, y1)], 0.2f);
    }

    // What bundle adjustment needs: a corner found in one photo and the corner found in another are the same surface
    // point. A checkerboard printed on a table, two photos 10 degrees apart; each corner of the first photo is carried
    // into the second through the true depth and compared with the nearest corner found there.
    [Fact]
    public void A_corner_is_found_on_the_same_surface_point_from_another_view()
    {
        var k = new CameraIntrinsics(480, 360, 420, 420, 239.5f, 179.5f);
        var (a, depthA) = CheckerPhoto(k, 0);
        var (b, _) = CheckerPhoto(k, 10);

        var inA = CornerDetector.Detect(a.Image);
        var inB = CornerDetector.Detect(b.Image);

        var distances = new List<float>();
        foreach (var c in inA)
        {
            float z = Depth(depthA, k.Width, c.X, c.Y);
            if (z <= 0) continue;
            var camera = Pinhole.ToCamera(b.CameraToWorld, Pinhole.BackProject(a, c.X, c.Y, z));
            if (!Pinhole.Project(k, camera, out float u, out float v) || u < 10 || v < 10 || u > k.Width - 11 || v > k.Height - 11) continue;
            distances.Add(inB.Min(d => MathF.Sqrt((d.X - u) * (d.X - u) + (d.Y - v) * (d.Y - v))));
        }
        var found = distances.Where(d => d <= 1f).Order().ToList();
        output.WriteLine($"{found.Count} of {distances.Count} corners found again, median {found[found.Count / 2]:F3} px, " +
                         $"90th percentile {found[found.Count * 9 / 10]:F3} px");
        Assert.True(found.Count >= distances.Count / 2, $"only {found.Count} of {distances.Count} corners found again");
        Assert.True(found[found.Count / 2] <= 0.2f, $"median {found[found.Count / 2]:F3} px");
        Assert.True(found[found.Count * 9 / 10] <= 0.5f, $"90th percentile {found[found.Count * 9 / 10]:F3} px");
    }

    [Fact]
    public void A_uniform_image_has_no_corners()
    {
        var image = new GrayImage(160, 120, Enumerable.Repeat((byte)128, 160 * 120).ToArray());

        Assert.Empty(CornerDetector.Detect(image));
    }

    // Shi-Tomasi scores the weaker of the two gradient directions, and a straight edge has only one, however strong.
    // The slanted edges are anti-aliased, yet sampled on the pixel grid and rounded to 8 bits they still wobble a
    // little; with nothing else in the image, that wobble is the strongest response there is.
    [Theory]
    [InlineData(0f)]
    [InlineData(20f)]
    [InlineData(45f)]
    public void A_straight_edge_has_no_corners(float degrees)
    {
        var normal = new Vector2(MathF.Cos(degrees * MathF.PI / 180f), MathF.Sin(degrees * MathF.PI / 180f));
        var pixels = new byte[160 * 120];
        for (int y = 0; y < 120; y++)
        for (int x = 0; x < 160; x++)
        {
            float coverage = Math.Clamp(0.5f + Vector2.Dot(new Vector2(x - 80.3f, y - 60.1f), normal), 0f, 1f);
            pixels[y * 160 + x] = (byte)MathF.Round(Dark + (Bright - Dark) * coverage);
        }

        Assert.Empty(CornerDetector.Detect(new GrayImage(160, 120, pixels)));
    }

    [Fact]
    public void Corners_of_a_textured_photo_spread_over_the_whole_image()
    {
        var image = Textured.Value;
        var options = new CornerOptions();

        var corners = CornerDetector.Detect(image, options);

        int columns = (image.Width + options.CellSize - 1) / options.CellSize;
        int rows = (image.Height + options.CellSize - 1) / options.CellSize;
        int occupied = corners.Select(c => Cell(c, options.CellSize, columns)).Distinct().Count();
        Assert.True(occupied >= columns * rows * 9 / 10, $"corners in only {occupied} of {columns * rows} cells");
        Assert.All(corners, c =>
        {
            Assert.InRange((int)(c.X + 0.5f), options.Border, image.Width - 1 - options.Border);
            Assert.InRange((int)(c.Y + 0.5f), options.Border, image.Height - 1 - options.Border);
        });
    }

    [Fact]
    public void Max_corners_keeps_the_strongest()
    {
        var image = Textured.Value;

        var all = CornerDetector.Detect(image);
        var strongest = CornerDetector.Detect(image, new CornerOptions(MaxCorners: 50));

        Assert.True(all.Count > 50, $"only {all.Count} corners");
        Assert.Equal(all.Take(50), strongest);
    }

    [Fact]
    public void Per_cell_limit_is_respected()
    {
        var image = Textured.Value;
        var options = new CornerOptions(CellSize: 20, PerCell: 2);
        int columns = (image.Width + options.CellSize - 1) / options.CellSize;

        var corners = CornerDetector.Detect(image, options);

        var perCell = corners.GroupBy(c => Cell(c, options.CellSize, columns)).Select(g => g.Count()).ToList();
        Assert.All(perCell, count => Assert.True(count <= 2, $"{count} corners in one cell"));
        // The limit binds: nearly every textured cell has more than two corners to offer.
        Assert.True(perCell.Count(count => count == 2) >= perCell.Count * 9 / 10);
    }

    [Fact]
    public void Corners_are_sorted_strongest_first_and_repeatable()
    {
        var image = Textured.Value;

        var first = CornerDetector.Detect(image);
        var second = CornerDetector.Detect(image);

        Assert.NotEmpty(first);
        Assert.Equal(first, second);
        for (int i = 1; i < first.Count; i++)
            Assert.True(first[i - 1].Response >= first[i].Response, $"corner {i} is stronger than corner {i - 1}");
    }

    // A phone photo: 1920x1080 of fine texture. Best of three runs, after one to compile the code.
    [Fact]
    public void A_full_hd_photo_takes_well_under_half_a_second()
    {
        var image = Noise(1920, 1080);
        CornerDetector.Detect(image);

        var best = TimeSpan.MaxValue;
        List<Feature> corners = [];
        for (int run = 0; run < 3; run++)
        {
            var watch = Stopwatch.StartNew();
            corners = CornerDetector.Detect(image);
            best = watch.Elapsed < best ? watch.Elapsed : best;
        }

        output.WriteLine($"1920x1080: {best.TotalMilliseconds:F1} ms, {corners.Count} corners");
        Assert.Equal(new CornerOptions().MaxCorners, corners.Count);
        Assert.True(best < TimeSpan.FromSeconds(0.5), $"took {best.TotalMilliseconds:F0} ms");
    }

    // The cell of the pixel a corner rounds to; cells are CellSize x CellSize blocks of pixels from the top left.
    private static int Cell(Feature c, int cellSize, int columns) =>
        (int)MathF.Floor((c.X + 0.5f) / cellSize) + (int)MathF.Floor((c.Y + 0.5f) / cellSize) * columns;

    private static void AssertCorners(List<Feature> corners, Vector2[] truth, float tolerance)
    {
        string found = string.Join(", ", corners.Select(c => $"({c.X:F2}, {c.Y:F2})"));
        Assert.True(corners.Count == truth.Length, $"expected {truth.Length} corners, found {corners.Count}: {found}");
        foreach (var t in truth)
        {
            float error = corners.Min(c => Vector2.Distance(new Vector2(c.X, c.Y), t));
            Assert.True(error <= tolerance, $"corner ({t.X}, {t.Y}) is {error:F3} px from the nearest of {found}");
        }
    }

    // A bright rectangle on a dark background with its edges at the given (continuous) coordinates.
    private static GrayImage Rectangle(int width, int height, float x0, float y0, float x1, float y1)
    {
        static float Overlap(float pixel, float from, float to) =>
            Math.Clamp(MathF.Min(pixel + 0.5f, to) - MathF.Max(pixel - 0.5f, from), 0f, 1f);

        var pixels = new byte[width * height];
        for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float coverage = Overlap(x, x0, x1) * Overlap(y, y0, y1);
            pixels[y * width + x] = (byte)MathF.Round(Dark + (Bright - Dark) * coverage);
        }
        return new GrayImage(width, height, pixels);
    }

    // A table and two objects on it, seen from 45 cm at an angle as during a scan; the table fills the view.
    private static ISdf Scene() => new UnionSdf([
        new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(1f, 0.01f, 1f)),
        new BoxSdf(new Vector3(0, 0.04f, 0), new Vector3(0.04f, 0.04f, 0.04f)),
        new SphereSdf(new Vector3(0.09f, 0.03f, -0.03f), 0.03f),
    ]);

    private static readonly Vector3 Target = new(0, 0.04f, 0);

    private static Matrix4x4 Pose(float degrees)
    {
        float azimuth = degrees * MathF.PI / 180f;
        return CameraPoses.LookAt(Target + new Vector3(MathF.Sin(azimuth) * 0.35f, 0.3f, -MathF.Cos(azimuth) * 0.35f), Target);
    }

    private static GrayImage TexturedPhoto() =>
        SyntheticPhotoRenderer.Render(Scene(), new CameraIntrinsics(320, 240, 280, 280, 159.5f, 119.5f), Pose(0)).Image;

    // A checkerboard printed on the table, blurred as by a lens: a checkerboard is the product of two square waves and
    // blur is separable, so each wave is blurred on its own, over about half a millimetre (half a pixel).
    private static (PhotoView View, float[] Depth) CheckerPhoto(CameraIntrinsics k, float degrees)
    {
        const float Square = 0.02f, Blur = 0.0005f;
        static float Wave(float t) => MathF.Tanh(MathF.Sin(t * MathF.PI / Square) * Square / (MathF.PI * Blur));
        static float Checker(Vector3 p) => (Dark + Bright) / 2 + (Bright - Dark) / 2 * Wave(p.X) * Wave(p.Z);
        var pose = Pose(degrees);
        var table = new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(1f, 0.01f, 1f));
        var (image, depth) = SyntheticPhotoRenderer.Render(table, k, pose, Checker);
        return (new PhotoView(image, k, pose), depth);
    }

    // Ground-truth depth at a sub-pixel position; 0 where the four pixels around it are not all on one surface.
    private static float Depth(float[] depth, int width, float x, float y)
    {
        int x0 = (int)MathF.Floor(x), y0 = (int)MathF.Floor(y);
        float ax = x - x0, ay = y - y0;
        float a = depth[y0 * width + x0], b = depth[y0 * width + x0 + 1];
        float c = depth[(y0 + 1) * width + x0], d = depth[(y0 + 1) * width + x0 + 1];
        float low = MathF.Min(MathF.Min(a, b), MathF.Min(c, d)), high = MathF.Max(MathF.Max(a, b), MathF.Max(c, d));
        if (low <= 0 || high > low * 1.01f) return 0;
        return (a + (b - a) * ax) * (1 - ay) + (c + (d - c) * ax) * ay;
    }

    // The solid texture on a plane facing the camera at 0.6 mm per pixel, as a phone sees an object 80 cm away.
    private static GrayImage Noise(int width, int height)
    {
        var pixels = new byte[width * height];
        Parallel.For(0, height, y =>
        {
            for (int x = 0; x < width; x++)
                pixels[y * width + x] = (byte)Math.Clamp(MathF.Round(SyntheticPhotoRenderer.SolidNoise(new Vector3(x, y, 0) * 0.0006f)), 0, 255);
        });
        return new GrayImage(width, height, pixels);
    }
}
