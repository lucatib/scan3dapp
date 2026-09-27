using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Photogrammetry;

public class TriangulationTests
{
    // A real phone photo: 1920x1080 with f ~ 1390 px.
    private static readonly CameraIntrinsics K = new(1920, 1080, 1390, 1390, 959.5f, 539.5f);

    // Away from the world origin, as an object in an ARCore session is.
    private static readonly Vector3 Target = new(0.6f, -0.3f, 0.2f);

    // Triangulation never looks at the pixels.
    private static readonly GrayImage Blank = new(1, 1, [0]);

    // A photo `radius` from the target, looking at it from 30 degrees above the table (world z is up).
    private static PhotoView Photo(float azimuthDegrees, float radius)
    {
        float azimuth = azimuthDegrees * MathF.PI / 180f, elevation = 30f * MathF.PI / 180f;
        var direction = new Vector3(MathF.Sin(azimuth) * MathF.Cos(elevation), -MathF.Cos(azimuth) * MathF.Cos(elevation),
            MathF.Sin(elevation));
        return new PhotoView(Blank, K, CameraPoses.LookAt(Target + direction * radius, Target));
    }

    // Photos 0.4 m from the target, stepDegrees apart on an arc around it: a walk around the object.
    private static List<PhotoView> Arc(int count, float stepDegrees = 10f) =>
        Enumerable.Range(0, count).Select(i => Photo((i - (count - 1) / 2f) * stepDegrees, 0.4f)).ToList();

    private static Observation Observe(IReadOnlyList<PhotoView> views, int view, Vector3 point)
    {
        Assert.True(Pinhole.Project(K, Pinhole.ToCamera(views[view].CameraToWorld, point), out float u, out float v));
        return new Observation(view, u, v);
    }

    private static Observation[] ObserveAll(IReadOnlyList<PhotoView> views, Vector3 point) =>
        Enumerable.Range(0, views.Count).Select(i => Observe(views, i, point)).ToArray();

    // Points of an object around the target, as far as 5 cm from it.
    private static List<Vector3> Points(int count, int seed)
    {
        var random = new Random(seed);
        return Enumerable.Range(0, count).Select(_ => Target + new Vector3(
            (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f) * 0.1f).ToList();
    }

    private static double Gaussian(Random random) =>
        Math.Sqrt(-2 * Math.Log(1 - random.NextDouble())) * Math.Cos(2 * Math.PI * random.NextDouble());

    private static Observation[] WithNoise(Observation[] observations, Random random, double sigmaPx) =>
        observations.Select(o => o with
        {
            X = o.X + (float)(sigmaPx * Gaussian(random)),
            Y = o.Y + (float)(sigmaPx * Gaussian(random)),
        }).ToArray();

    private static double SquaredError(IReadOnlyList<PhotoView> views, Vector3 point, Observation[] observations) =>
        observations.Sum(o => Math.Pow(Triangulation.ReprojectionError(views[o.View], point, o), 2));

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    public void Exact_observations_triangulate_to_the_point(int cameras)
    {
        var views = Arc(cameras);

        foreach (var point in Points(20, cameras))
        {
            var result = Triangulation.Triangulate(views, ObserveAll(views, point));

            Assert.NotNull(result);
            float error = Vector3.Distance(result.Value, point);
            Assert.True(error < 1e-5f, $"{cameras} cameras: error {error * 1e3f:F5} mm");
        }
    }

    [Fact]
    public void Half_a_pixel_of_noise_moves_the_point_a_few_tenths_of_a_millimetre()
    {
        var views = Arc(5);
        var random = new Random(7);
        double sumSquares = 0;
        var points = Points(300, 11);

        foreach (var point in points)
        {
            var result = Triangulation.Triangulate(views, WithNoise(ObserveAll(views, point), random, 0.5));

            Assert.NotNull(result);
            sumSquares += Vector3.DistanceSquared(result.Value, point);
        }

        // About 0.3 mm with five photos over 40 degrees.
        double rms = Math.Sqrt(sumSquares / points.Count);
        Assert.True(rms is > 0.05e-3 and < 0.5e-3, $"RMS error {rms * 1e3:F3} mm");
    }

    [Fact]
    public void The_point_minimises_the_reprojection_error_when_the_photos_are_at_different_distances()
    {
        // The linear solve weights each photo by the point's depth in it, so on its own it is off the least-squares
        // point whenever the depths differ; the refinement must land where no small step lowers the pixel error.
        List<PhotoView> views = [Photo(-30, 0.25f), Photo(-10, 0.7f), Photo(10, 0.3f), Photo(30, 0.6f)];
        var random = new Random(3);

        foreach (var point in Points(50, 5))
        {
            var observations = WithNoise(ObserveAll(views, point), random, 1);

            var result = Triangulation.Triangulate(views, observations, maxErrorPx: 5f);

            Assert.NotNull(result);
            double error = SquaredError(views, result.Value, observations);
            foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            foreach (float step in new[] { -5e-6f, 5e-6f })
                Assert.True(SquaredError(views, result.Value + axis * step, observations) >= error,
                    $"a {step * 1e6f} um step along {axis} lowers the error");
        }
    }

    [Fact]
    public void A_single_observation_gives_no_point()
    {
        var views = Arc(2);

        Assert.Null(Triangulation.Triangulate(views, [Observe(views, 0, Target)]));
    }

    [Fact]
    public void Nearly_parallel_rays_are_rejected()
    {
        // Two photos 2 mm apart: the rays meet at 0.3 degrees, so a pixel of error is centimetres in depth.
        var eye = Target + new Vector3(0, -0.4f, 0);
        var views = new List<PhotoView>
        {
            new(Blank, K, CameraPoses.LookAt(eye, Target)),
            new(Blank, K, CameraPoses.LookAt(eye + new Vector3(0.002f, 0, 0), Target)),
        };
        var observations = ObserveAll(views, Target + new Vector3(0.01f, 0, 0.02f));

        Assert.Null(Triangulation.Triangulate(views, observations));
        Assert.NotNull(Triangulation.Triangulate(views, observations, minAngleDegrees: 0.1f));
    }

    [Fact]
    public void The_widest_pair_of_rays_decides_the_angle()
    {
        // A photo every half second gives near-duplicate viewpoints: three photos 0.2 degrees apart and one 20 degrees
        // away pin the point down, although most pairs are nearly parallel.
        List<PhotoView> clustered = [Photo(0, 0.4f), Photo(0.2f, 0.4f), Photo(0.4f, 0.4f), Photo(20, 0.4f)];
        // A gentle arc: neighbours 0.6 degrees apart (0.7 of azimuth at 30 degrees up), the ends 1.2 apart.
        List<PhotoView> gentle = [Photo(0, 0.4f), Photo(0.7f, 0.4f), Photo(1.4f, 0.4f)];
        var point = Target + new Vector3(0.01f, -0.005f, 0.015f);

        var fromClustered = Triangulation.Triangulate(clustered, ObserveAll(clustered, point));
        var fromGentle = Triangulation.Triangulate(gentle, ObserveAll(gentle, point));

        Assert.NotNull(fromClustered);
        Assert.True(Vector3.Distance(fromClustered.Value, point) < 1e-5f);
        Assert.NotNull(fromGentle);
    }

    [Fact]
    public void The_error_limit_applies_to_the_rms_pixel_distance()
    {
        var views = Arc(4);
        var observations = ObserveAll(views, Target + new Vector3(0.01f, 0.02f, -0.01f));
        observations[1] = observations[1] with { X = observations[1].X + 3, Y = observations[1].Y - 2 };
        var fitted = Triangulation.Triangulate(views, observations, maxErrorPx: 1000f);
        Assert.NotNull(fitted);
        double rms = Math.Sqrt(SquaredError(views, fitted.Value, observations) / observations.Length);

        // Not the error per coordinate (smaller by √2) nor the largest one (larger): exactly the RMS distance.
        Assert.Null(Triangulation.Triangulate(views, observations, maxErrorPx: (float)(0.98 * rms)));
        Assert.NotNull(Triangulation.Triangulate(views, observations, maxErrorPx: (float)(1.02 * rms)));
    }

    [Fact]
    public void A_non_finite_observation_or_pose_gives_no_point()
    {
        var views = Arc(3);
        var observations = ObserveAll(views, Target);
        var broken = observations.ToArray();
        broken[2] = broken[2] with { X = float.NaN };
        var badPose = views.ToList();
        var pose = badPose[2].CameraToWorld;
        pose.M41 = float.NaN;
        badPose[2] = badPose[2] with { CameraToWorld = pose };

        Assert.Null(Triangulation.Triangulate(views, broken));
        Assert.Null(Triangulation.Triangulate(badPose, observations));
    }

    [Fact]
    public void A_point_behind_one_of_the_cameras_is_rejected()
    {
        var views = Arc(3, stepDegrees: 20f);
        var point = Target + new Vector3(0.02f, 0.01f, -0.01f);
        var observations = ObserveAll(views, point).ToList();
        Assert.NotNull(Triangulation.Triangulate(views, observations));

        // A fourth photo 0.4 m past the point, looking away from it. Its pixel is where the pinhole equations put the
        // point (x/z, y/z with z < 0), so the linear solution fits every observation exactly.
        var away = Vector3.Normalize(point - views[1].CameraToWorld.Translation);
        var behind = CameraPoses.LookAt(point + away * 0.4f, point + away * 0.8f);
        views.Add(new PhotoView(Blank, K, behind));
        var camera = Pinhole.ToCamera(behind, point);
        Assert.True(camera.Z < 0);
        observations.Add(new Observation(3, K.Fx * camera.X / camera.Z + K.Cx, K.Fy * camera.Y / camera.Z + K.Cy));

        Assert.Null(Triangulation.Triangulate(views, observations));
    }

    [Fact]
    public void Rays_that_diverge_meet_behind_the_cameras_and_are_rejected()
    {
        // Two photos side by side looking the same way; each sees the feature on its outer side, as a wrong match
        // would. The lines through the rays cross 0.4 m behind the cameras.
        var views = new List<PhotoView>
        {
            new(Blank, K, CameraPoses.LookAt(new Vector3(-0.05f, 0, 0), new Vector3(-0.05f, 1, 0))),
            new(Blank, K, CameraPoses.LookAt(new Vector3(0.05f, 0, 0), new Vector3(0.05f, 1, 0))),
        };
        float offset = K.Fx * 0.05f / 0.4f;
        Observation[] observations = [new(0, K.Cx - offset, K.Cy), new(1, K.Cx + offset, K.Cy)];

        Assert.Null(Triangulation.Triangulate(views, observations));
    }

    [Fact]
    public void One_gross_outlier_pushes_the_error_over_the_limit()
    {
        var views = Arc(4);
        var point = Target + new Vector3(-0.015f, 0.02f, 0.01f);
        var observations = ObserveAll(views, point);
        Assert.NotNull(Triangulation.Triangulate(views, observations));

        observations[2] = observations[2] with { X = observations[2].X + 15, Y = observations[2].Y - 15 };

        // The best fit leaves about 9 px RMS: rejected by the default 2 px limit, and only by it.
        Assert.Null(Triangulation.Triangulate(views, observations));
        Assert.NotNull(Triangulation.Triangulate(views, observations, maxErrorPx: 20f));
    }

    [Fact]
    public void Reprojection_error_is_the_pixel_distance_to_the_projection()
    {
        var k = new CameraIntrinsics(1920, 1080, 1000, 1000, 960, 540);
        // At (1, 2, 3) looking along world +x: camera x is world -z, camera y (down) is world +y.
        var pose = new Matrix4x4(
            0, 0, -1, 0,
            0, 1, 0, 0,
            1, 0, 0, 0,
            1, 2, 3, 1);
        var view = new PhotoView(Blank, k, pose);
        // In the camera frame (0.125, -0.0625, 2): it projects to (960 + 62.5, 540 - 31.25) = (1022.5, 508.75).
        var point = new Vector3(3, 1.9375f, 2.875f);

        Assert.Equal(0f, Triangulation.ReprojectionError(view, point, new Observation(0, 1022.5f, 508.75f)));
        Assert.Equal(5f, Triangulation.ReprojectionError(view, point, new Observation(0, 1025.5f, 512.75f)));
        Assert.Equal(5f, Triangulation.ReprojectionError(view, point, new Observation(0, 1019.5f, 504.75f)));
    }

    [Fact]
    public void Reprojection_error_is_infinite_behind_the_camera()
    {
        var k = new CameraIntrinsics(1920, 1080, 1000, 1000, 960, 540);
        var view = new PhotoView(Blank, k, Matrix4x4.CreateTranslation(1, 2, 3));

        Assert.Equal(float.PositiveInfinity, Triangulation.ReprojectionError(view, new Vector3(1, 2, 1), new Observation(0, 960, 540)));
        Assert.Equal(float.PositiveInfinity, Triangulation.ReprojectionError(view, new Vector3(1.1f, 2, 3), new Observation(0, 960, 540)));
    }
}
