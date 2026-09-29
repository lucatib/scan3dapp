using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Photogrammetry;

public class LiveReconstructionTests
{
    private static PhotoView Blank(Vector3 eye)
    {
        var k = new CameraIntrinsics(64, 48, 60f, 60f, 32f, 24f);
        return new PhotoView(new GrayImage(k.Width, k.Height, new byte[k.Width * k.Height]), k, CameraPoses.LookAt(eye, Vector3.Zero));
    }

    private static bool Contains((Vector3 Min, Vector3 Max) box, Vector3 p) =>
        p.X >= box.Min.X && p.Y >= box.Min.Y && p.Z >= box.Min.Z && p.X <= box.Max.X && p.Y <= box.Max.Y && p.Z <= box.Max.Z;

    /// <summary>
    /// The box grows while the target stays (ARCore's cluster wanders), but a target that moved away starts it again:
    /// when ARCore's depth fails, the hit-test target is off the piece and the photos move it, and a box that kept the
    /// first place too grew to 75 cm and cropped the photos around empty table.
    /// </summary>
    [Fact]
    public void The_box_follows_a_target_that_moved_and_grows_around_one_that_stays()
    {
        var live = new LiveReconstruction();
        PhotoView[] photos = [Blank(new Vector3(0.4f, 0.4f, 0)), Blank(new Vector3(0, 0.4f, 0.4f))];
        bool[] blurred = [false, false]; // no depth maps: only the box is under test
        var first = new Vector3(0.5f, 0, -0.3f);
        var piece = Vector3.Zero;

        live.Add(photos, blurred, 0, first, []);
        Assert.True(Contains(live.Box, first));

        live.Add(photos, blurred, 0, piece, []);
        Assert.True(Contains(live.Box, piece));
        Assert.False(Contains(live.Box, first), "the box left the place the target moved away from");

        var nearby = new Vector3(0.02f, 0, 0);
        live.Add(photos, blurred, 0, nearby, []);
        Assert.True(Contains(live.Box, piece - new Vector3(0.09f, 0, 0)), "a small move grows the box");
    }
}
