using Scanner.Capture;
using Scanner.Core.Turntable;

namespace Scanner.Core.Tests.Turntable;

public class MatTurntableScanTests
{
    private static readonly CameraIntrinsics Camera = new(1280, 960, 1050, 1050, 639.5f, 479.5f);

    [Fact]
    public void Keeps_one_photo_per_new_angle_and_reports_where_the_camera_is()
    {
        var scan = new MatTurntableScan();
        float[] turns = [0, 2, 6, 6, 40, 41];
        var kept = new List<bool>();
        foreach (float turn in turns)
        {
            var pose = MatDetectorTests.CameraOnMat(turn, 30, 0.29f);
            var photo = new PhotoView(MatDetectorTests.Render(pose, 0.04f, 0.07f), Camera, System.Numerics.Matrix4x4.Identity);
            var result = scan.Offer(photo);
            kept.Add(result.Kept);
            Assert.NotNull(result.AngleDegrees);
            float off = MathF.Abs(result.AngleDegrees!.Value - MatTurntableScan.Angle(pose)) % 360;
            Assert.InRange(MathF.Min(off, 360 - off), 0, 0.3f); // 0° and 360° are the same angle
        }

        Assert.Equal([true, false, true, false, true, false], kept);
        Assert.Equal(3, scan.Views.Count);
        Assert.True(System.Numerics.Vector3.Distance(scan.Views[2].CameraToWorld.Translation,
            MatDetectorTests.CameraOnMat(40, 30, 0.29f).Translation) < 0.001f);
    }
}
