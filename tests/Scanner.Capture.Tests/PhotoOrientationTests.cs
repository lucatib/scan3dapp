using System.Numerics;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Tests;

public sealed class PhotoOrientationTests
{
    // Deliberately asymmetric in every field: a transposed fx/fy or a swapped cx/cy would survive a square image
    // with a centred principal point, which is what makes this kind of bug ship.
    private static readonly CameraIntrinsics Sensor = new(640, 480, 500f, 520f, 310f, 250f);

    // Not the identity, or the axis change would be the only thing under test and a wrong-handed one would pass.
    private static readonly Matrix4x4 Pose =
        Matrix4x4.CreateRotationY(0.6f) * Matrix4x4.CreateRotationX(-0.2f) * Matrix4x4.CreateTranslation(0.3f, 1.1f, -0.4f);

    // Given in camera space and pushed out to the world through the pose, so they are in front of the camera by
    // construction; a literal world point only stays in front for as long as nobody touches the pose above.
    private static readonly Vector3[] WorldPoints =
        new Vector3[] { new(0.05f, -0.02f, 0.8f), new(-0.12f, 0.09f, 1.4f), new(0.2f, 0.15f, 2.1f) }
            .Select(p => Vector3.Transform(p, Pose)).ToArray();

    /// <summary>Projects a world point through a camera→world pose and pinhole intrinsics (OpenCV, row vectors).</summary>
    private static Vector2 Project(CameraIntrinsics k, Matrix4x4 cameraToWorld, Vector3 world)
    {
        Assert.True(Matrix4x4.Invert(cameraToWorld, out var worldToCamera));
        var c = Vector3.Transform(world, worldToCamera);
        Assert.True(c.Z > 0, "The test points must be in front of the camera, or the projection is meaningless.");
        return new Vector2(k.Fx * c.X / c.Z + k.Cx, k.Fy * c.Y / c.Z + k.Cy);
    }

    /// <summary>Where a pixel lands when the picture itself is turned clockwise; the definition the rest follows from.</summary>
    private static Vector2 RotatePixel(Vector2 p, int degrees) => degrees switch
    {
        0 => p,
        90 => new Vector2(Sensor.Height - 1 - p.Y, p.X),
        180 => new Vector2(Sensor.Width - 1 - p.X, Sensor.Height - 1 - p.Y),
        _ => new Vector2(p.Y, Sensor.Width - 1 - p.X),
    };

    [Theory]
    [InlineData(0)]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(270)]
    public void Rotated_intrinsics_and_pose_project_to_the_rotated_pixel(int degrees)
    {
        var k = PhotoOrientation.RotateIntrinsics(Sensor, degrees);
        var pose = PhotoOrientation.RotateCameraToWorld(Pose, degrees);

        foreach (var world in WorldPoints)
        {
            var expected = RotatePixel(Project(Sensor, Pose, world), degrees);
            var actual = Project(k, pose, world);
            Assert.Equal(expected.X, actual.X, 0.002f); // a tolerance, not decimal places: the two routes differ in the last float bits
            Assert.Equal(expected.Y, actual.Y, 0.002f);
        }
    }

    [Fact]
    public void A_quarter_turn_swaps_the_image_dimensions_and_a_half_turn_does_not()
    {
        Assert.Equal((480, 640), Dimensions(90));
        Assert.Equal((640, 480), Dimensions(180));
        Assert.Equal((480, 640), Dimensions(270));
        Assert.Equal((640, 480), Dimensions(0));

        static (int, int) Dimensions(int degrees)
        {
            var k = PhotoOrientation.RotateIntrinsics(Sensor, degrees);
            return (k.Width, k.Height);
        }
    }

    [Fact]
    public void Four_quarter_turns_are_the_identity_and_negatives_wrap()
    {
        var k = Sensor;
        var pose = Pose;
        for (int i = 0; i < 4; i++)
        {
            pose = PhotoOrientation.RotateCameraToWorld(pose, 90);
            k = PhotoOrientation.RotateIntrinsics(k, 90);
        }

        Assert.Equal(Sensor, k);
        Assert.Equal(PhotoOrientation.RotateIntrinsics(Sensor, 270), PhotoOrientation.RotateIntrinsics(Sensor, -90));
        Assert.Equal(Sensor, PhotoOrientation.RotateIntrinsics(Sensor, 360));
        foreach (var world in WorldPoints)
        {
            var expected = Project(Sensor, Pose, world);
            var actual = Project(k, pose, world);
            Assert.Equal(expected.X, actual.X, 0.002f); // a tolerance, not decimal places: the two routes differ in the last float bits
            Assert.Equal(expected.Y, actual.Y, 0.002f);
        }
    }

    [Fact]
    public void An_angle_that_is_not_a_quarter_turn_is_rejected()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => PhotoOrientation.RotateIntrinsics(Sensor, 45));
        Assert.Throws<ArgumentOutOfRangeException>(() => PhotoOrientation.RotateCameraToWorld(Pose, 1));
    }

    [Theory]
    [InlineData(1f, 0f, 0)]
    [InlineData(0f, 1f, 90)]
    [InlineData(-1f, 0f, 180)]
    [InlineData(0f, -1f, 270)]
    [InlineData(0.9f, 0.3f, 0)] // off-axis but unambiguous: quarter turns are the only answers a camera can need
    public void The_rotation_follows_where_the_image_x_axis_points_on_screen(float dx, float dy, int expected) =>
        Assert.Equal(expected, PhotoOrientation.DegreesFromImageAxis(dx, dy));
}
