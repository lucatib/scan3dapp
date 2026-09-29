using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Photogrammetry;

public class AimPointTests
{
    private static readonly CameraIntrinsics K = new(64, 48, 60f, 60f, 32f, 24f);
    private static readonly Vector3 Piece = new(-3.1f, -0.06f, -3.7f);

    private static PhotoView View(Vector3 eye, Vector3 look) =>
        new(new GrayImage(K.Width, K.Height, new byte[K.Width * K.Height]), K, CameraPoses.LookAt(eye, look));

    private static Vector3 Around(float azimuthDegrees, float distance = 0.45f)
    {
        float a = azimuthDegrees * MathF.PI / 180, e = 50 * MathF.PI / 180;
        return Piece + distance * new Vector3(MathF.Cos(e) * MathF.Cos(a), MathF.Sin(e), MathF.Cos(e) * MathF.Sin(a));
    }

    /// <summary>A loop around the piece, each photo aimed a centimetre or two off it (hand-held): the point the photos
    /// look at is the piece, as in the scan whose ARCore depth failed while the photos all showed the grinder.</summary>
    [Fact]
    public void The_photos_of_a_loop_meet_at_the_piece()
    {
        var random = new Random(3);
        var views = Enumerable.Range(0, 30).Select(i => View(Around(i * 12f, 0.4f + 0.1f * random.NextSingle()),
            Piece + 0.015f * new Vector3(random.NextSingle() - .5f, random.NextSingle() - .5f, random.NextSingle() - .5f))).ToList();

        var aim = AimPoint.Estimate(views);

        Assert.NotNull(aim);
        Assert.True(Vector3.Distance(Piece, aim.Value) < 0.01f, $"{aim} vs {Piece}");
    }

    [Fact]
    public void A_short_arc_is_enough_once_the_views_turn()
    {
        var views = Enumerable.Range(0, 10).Select(i => View(Around(i * 4f), Piece)).ToList();

        var aim = AimPoint.Estimate(views);

        Assert.NotNull(aim);
        Assert.True(Vector3.Distance(Piece, aim.Value) < 0.01f);
    }

    [Fact]
    public void Photos_that_look_the_same_way_do_not_meet()
    {
        // Walking sideways past the table with the phone pointed straight ahead: parallel rays, no aim point.
        var views = Enumerable.Range(0, 20).Select(i =>
        {
            var eye = new Vector3(i * 0.05f, 0.3f, 0);
            return View(eye, eye + new Vector3(0, -0.5f, -0.6f));
        }).ToList();

        Assert.Null(AimPoint.Estimate(views));
    }

    [Fact]
    public void Too_few_photos_give_no_aim_point()
    {
        Assert.Null(AimPoint.Estimate(Enumerable.Range(0, AimPoint.MinViews - 1).Select(i => View(Around(i * 20f), Piece)).ToList()));
    }
}
