using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Synthetic;
using Scanner.Core.Turntable;

namespace Scanner.Core.Tests.Turntable;

public class LidarMatTurntableTests
{
    /// <summary>The LiDAR depth: a quarter of the photo's size, same field of view.</summary>
    private static readonly CameraIntrinsics Depth = new(320, 240, 1050 / 4f, 1050 / 4f, 159.5f, 119.5f);

    private static readonly CameraIntrinsics Photo = new(1280, 960, 1050, 1050, 639.5f, 479.5f);

    /// <summary>The platter (its top at y = 0) and a piece fixed on it: a box with a ball on one corner, so a turn shows.</summary>
    private static readonly ISdf Platter = new BoxSdf(new Vector3(0, -0.005f, 0), new Vector3(0.12f, 0.005f, 0.12f));

    private static readonly ISdf Piece = new UnionSdf([
        new BoxSdf(new Vector3(0.01f, 0.02f, 0), new Vector3(0.03f, 0.02f, 0.02f)),
        new SphereSdf(new Vector3(-0.015f, 0.05f, 0.01f), 0.015f),
    ]);

    [Fact]
    public void Poses_every_frame_from_the_ring_and_builds_the_piece_in_the_mat_frame()
    {
        var scan = new LidarMatTurntable(new LidarTurntableOptions(VoxelSize: 0.002f));
        for (int i = 0; i < 36; i++)
        {
            var result = scan.Add(RenderDepth(i * 10, i), RenderPhoto(i * 10));
            Assert.True(result.Accepted, $"frame {i}: {result.Reason}");
            Assert.Equal(TurntablePoseSource.Mat, result.Source);
        }
        Assert.Equal(36, scan.MatFrames);
        Assert.Equal(36, scan.CoveredSteps);

        var mesh = scan.Mesh();
        Assert.True(mesh.TriangleCount > 500);
        Assert.InRange(mesh.Positions.Max(p => p.Y), 0.065f - 0.004f, 0.065f + 0.004f); // the ball's top
        Assert.InRange(mesh.Positions.Min(p => p.Y), -0.001f, 0.001f); // flat on the platter
        Assert.InRange(mesh.Positions.Max(p => p.X), 0.04f - 0.004f, 0.04f + 0.004f); // the box's far end, true scale
        Assert.InRange(mesh.Positions.Min(p => p.X), -0.03f - 0.004f, -0.03f + 0.004f); // the ball's side
    }

    [Fact]
    public void Carries_the_pose_by_icp_while_the_ring_is_hidden_and_snaps_back_when_it_shows()
    {
        var scan = new LidarMatTurntable(new LidarTurntableOptions(VoxelSize: 0.002f));
        for (int i = 0; i < 16; i++)
        {
            float turn = i * 4;
            bool ringHidden = i is >= 6 and < 12;
            var result = scan.Add(RenderDepth(turn, i), ringHidden ? Blank() : RenderPhoto(turn));
            Assert.True(result.Accepted, $"frame {i}: {result.Reason}");
            Assert.Equal(ringHidden ? TurntablePoseSource.Icp : TurntablePoseSource.Mat, result.Source);

            var truth = MatDetectorTests.CameraOnMat(turn, 30, 0.33f);
            var pose = result.CameraToMat!.Value;
            Assert.True(Vector3.Distance(pose.Translation, truth.Translation) < 0.004f,
                $"frame {i}: camera at {pose.Translation}, truth {truth.Translation}");
            var look = Vector3.TransformNormal(Vector3.UnitZ, pose);
            Assert.True(Vector3.Dot(look, Vector3.TransformNormal(Vector3.UnitZ, truth)) > MathF.Cos(MathF.PI / 180),
                $"frame {i}: looks off by more than 1°");
        }
        Assert.Equal(6, scan.IcpFrames);
    }

    [Fact]
    public void Waits_for_the_ring_before_the_first_frame()
    {
        var scan = new LidarMatTurntable();
        var result = scan.Add(RenderDepth(0, 0), Blank());
        Assert.False(result.Accepted);
        Assert.Contains("dot ring", result.Reason);
        Assert.Equal(0, scan.AcceptedFrames);
    }

    [Fact]
    public void Rejects_a_ring_pose_the_depth_does_not_agree_with()
    {
        // The depth sees the platter 2 cm lower than the photo's ring says (a misread ring, or unregistered depth).
        var scan = new LidarMatTurntable();
        var pose = MatDetectorTests.CameraOnMat(0, 30, 0.33f);
        var shifted = pose * Matrix4x4.CreateTranslation(0, 0.02f, 0);
        var depth = SyntheticDepthRenderer.Render(new UnionSdf([Platter, Piece]), Depth, shifted);
        var result = scan.Add(depth, RenderPhoto(0));
        Assert.False(result.Accepted);
        Assert.Contains("disagree", result.Reason);
    }

    [Fact]
    public void Turn_about_y_reads_back_the_angle()
    {
        Assert.Equal(0.3f, LidarMatTurntable.TurnAboutY(Matrix4x4.CreateRotationY(0.3f)), 5);
        Assert.Equal(-1.2f, LidarMatTurntable.TurnAboutY(Matrix4x4.CreateRotationY(-1.2f)), 5);
    }

    /// <summary>The still camera about 33 cm from the platter centre, 30° down; the platter turned by
    /// <paramref name="turnDegrees"/> (seen from the mat, the camera goes round the other way).</summary>
    private static DepthFrame RenderDepth(float turnDegrees, int seed) =>
        SyntheticDepthRenderer.Render(new UnionSdf([Platter, Piece]), Depth, MatDetectorTests.CameraOnMat(turnDegrees, 30, 0.33f),
            noiseSigma: 0.0005f, seed: seed, timestampSeconds: seed * 0.5);

    private static PhotoView RenderPhoto(float turnDegrees) =>
        new(MatDetectorTests.Render(MatDetectorTests.CameraOnMat(turnDegrees, 30, 0.33f), 0.035f, 0.065f), Photo,
            Matrix4x4.Identity);

    private static PhotoView Blank() => new(new GrayImage(Photo.Width, Photo.Height, Enumerable.Repeat((byte)140,
        Photo.Width * Photo.Height).ToArray()), Photo, Matrix4x4.Identity);
}
