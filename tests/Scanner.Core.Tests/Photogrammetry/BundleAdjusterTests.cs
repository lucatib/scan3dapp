using System.Diagnostics;
using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Synthetic;
using Xunit.Abstractions;

namespace Scanner.Core.Tests.Photogrammetry;

public class BundleAdjusterTests(ITestOutputHelper output)
{
    // The phone's 1080p photos.
    private static readonly CameraIntrinsics K = new(1920, 1080, 1390, 1390, 959.5f, 539.5f);

    // Bundle adjustment never looks at the pixels: one blank frame stands in for every photo.
    private static readonly GrayImage Blank = new(K.Width, K.Height, new byte[K.Width * K.Height]);

    [Fact]
    public void Recovers_the_geometry_from_perturbed_poses_and_noisy_observations()
    {
        var random = new Random(1);
        var truth = Scene(random, cameras: 16, firstAzimuth: -67.5, azimuthStep: 9, points: 400, viewsPerPoint: int.MaxValue);
        var (views, tracks, _) = Perturb(random, truth, noisePx: 0.3, outlierFraction: 0);

        var result = BundleAdjuster.Adjust(views, tracks);

        var check = Compare(truth, result);
        Report(result, check);
        Assert.True(result.InitialRmsPx > 10, $"initial RMS {result.InitialRmsPx:F2} px: the perturbation should matter");
        Assert.True(result.FinalRmsPx < 0.5, $"final RMS {result.FinalRmsPx:F3} px");
        Assert.True(result.Converged, $"not converged after {result.Iterations} iterations");
        AssertGeometry(check);
    }

    // Reprojections leave the scene free to shift, turn and scale; only the centre priors settle those seven degrees
    // of freedom, and the cost along them is very flat. At the optimum no similarity of the adjusted scene fits the
    // centres to their priors any better, so the residuals C - C_prior have no mean, no net stretch and no net twist
    // about the centroid. This is what keeps ARCore's metric scale and gravity.
    [Fact]
    public void Settles_position_orientation_and_scale_on_the_priors()
    {
        var random = new Random(1);
        var truth = Scene(random, cameras: 16, firstAzimuth: -67.5, azimuthStep: 9, points: 400, viewsPerPoint: int.MaxValue);
        var (views, tracks, _) = Perturb(random, truth, noisePx: 0.3, outlierFraction: 0);

        var result = BundleAdjuster.Adjust(views, tracks);

        int n = views.Count;
        var centres = result.CameraToWorld.Select(p => p.Translation).ToArray();
        var mean = centres.Aggregate(Vector3.Zero, (a, b) => a + b) / n;
        double shiftX = 0, shiftY = 0, shiftZ = 0, stretch = 0, twistX = 0, twistY = 0, twistZ = 0, spread = 0;
        for (int i = 0; i < n; i++)
        {
            var prior = views[i].CameraToWorld.Translation;
            double rx = (double)centres[i].X - prior.X, ry = (double)centres[i].Y - prior.Y, rz = (double)centres[i].Z - prior.Z;
            double mx = (double)centres[i].X - mean.X, my = (double)centres[i].Y - mean.Y, mz = (double)centres[i].Z - mean.Z;
            shiftX += rx / n;
            shiftY += ry / n;
            shiftZ += rz / n;
            stretch += mx * rx + my * ry + mz * rz;
            twistX += my * rz - mz * ry;
            twistY += mz * rx - mx * rz;
            twistZ += mx * ry - my * rx;
            spread += mx * mx + my * my + mz * mz;
        }
        double shift = Math.Sqrt(shiftX * shiftX + shiftY * shiftY + shiftZ * shiftZ);
        double twist = Math.Sqrt(twistX * twistX + twistY * twistY + twistZ * twistZ) / spread;
        stretch /= spread;

        output.WriteLine($"{result.Iterations} iterations; mean residual {shift * 1e3:E2} mm, stretch {stretch:E2}, twist {twist:E2} rad");
        Assert.True(result.Converged);
        Assert.True(shift < 1e-5, $"the centres sit {shift * 1e3:F4} mm off their priors on average");
        Assert.True(Math.Abs(stretch) < 1e-5, $"scaling the scene by {1 - stretch:F6} would fit the priors better");
        Assert.True(twist < 1e-5, $"turning the scene by {twist:E2} rad would fit the priors better");
    }

    [Fact]
    public void Huber_loss_keeps_gross_outliers_from_bending_the_geometry()
    {
        var random = new Random(2);
        var truth = Scene(random, cameras: 16, firstAzimuth: -67.5, azimuthStep: 9, points: 400, viewsPerPoint: int.MaxValue);
        var (views, tracks, outlier) = Perturb(random, truth, noisePx: 0.3, outlierFraction: 0.05);

        var robust = BundleAdjuster.Adjust(views, tracks);
        var plain = BundleAdjuster.Adjust(views, tracks, new BundleOptions(HuberPx: float.PositiveInfinity));

        var check = Compare(truth, robust);
        var plainCheck = Compare(truth, plain);
        Report(robust, check);
        output.WriteLine($"without Huber: centre distances off by up to {plainCheck.CentreDistanceMm:F3} mm, " +
                         $"relative rotations by up to {plainCheck.RotationDeg:F4} deg");

        var (inliers, outliers) = Residuals(robust, tracks, outlier);
        output.WriteLine($"inlier RMS {Rms(inliers):F3} px, median outlier residual {Median(outliers):F1} px");
        Assert.True(Rms(inliers) < 0.5, $"inlier RMS {Rms(inliers):F3} px");
        // The outliers keep their error instead of being absorbed by bending the cameras and points towards them.
        Assert.True(Median(outliers) > 10, $"median outlier residual {Median(outliers):F2} px");
        Assert.True(robust.Converged, $"not converged after {robust.Iterations} iterations");
        AssertGeometry(check);
        Assert.True(check.CentreDistanceMm < plainCheck.CentreDistanceMm,
            "Huber should do better than plain least squares on data with outliers");
    }

    [Fact]
    public void Exact_observations_at_the_true_poses_are_left_alone()
    {
        var random = new Random(3);
        var truth = Scene(random, cameras: 12, firstAzimuth: -55, azimuthStep: 10, points: 300, viewsPerPoint: int.MaxValue);
        var views = truth.Poses.Select(p => new PhotoView(Blank, K, p)).ToList();
        var tracks = truth.Points.Select((x, i) => new Track(x,
            truth.SeenBy[i].Select(c => Observe(truth, c, x, 0, 0)).ToArray())).ToList();

        var result = BundleAdjuster.Adjust(views, tracks);

        output.WriteLine($"RMS {result.InitialRmsPx:E2} -> {result.FinalRmsPx:E2} px, {result.Iterations} iterations");
        // Rounding the pixels to float is all there is to fit, and it is never exactly nothing: zero here would mean
        // the observations were not used at all.
        Assert.InRange(result.InitialRmsPx, 1e-9, 1e-3);
        Assert.True(result.FinalRmsPx < 1e-3, $"final RMS {result.FinalRmsPx:E2} px");
        Assert.True(result.Converged);
        for (int c = 0; c < views.Count; c++)
        {
            Assert.True(Vector3.Distance(truth.Poses[c].Translation, result.CameraToWorld[c].Translation) < 1e-6f);
            Assert.True(RotationAngleDeg(Rotation(truth.Poses[c]), Rotation(result.CameraToWorld[c])) < 1e-4);
            Assert.Equal(0f, result.CameraToWorld[c].M14);
            Assert.Equal(0f, result.CameraToWorld[c].M24);
            Assert.Equal(0f, result.CameraToWorld[c].M34);
            Assert.Equal(1f, result.CameraToWorld[c].M44);
        }
        for (int i = 0; i < tracks.Count; i++)
            Assert.True(Vector3.Distance(truth.Points[i], result.Points[i]) < 1e-6f);
    }

    [Fact]
    public void Adjusts_sixty_cameras_and_three_thousand_points_in_a_few_seconds()
    {
        var random = new Random(4);
        // A full walk around the object, each point seen by ten neighbouring photos.
        var truth = Scene(random, cameras: 60, firstAzimuth: 0, azimuthStep: 6, points: 3000, viewsPerPoint: 10);
        var (views, tracks, _) = Perturb(random, truth, noisePx: 0.3, outlierFraction: 0);
        int observations = tracks.Sum(t => t.Observations.Length);

        BundleAdjuster.Adjust(views.Take(3).ToList(), []); // JIT warm-up
        var clock = Stopwatch.StartNew();
        var result = BundleAdjuster.Adjust(views, tracks);
        clock.Stop();

        var check = Compare(truth, result);
        output.WriteLine($"{views.Count} cameras, {tracks.Count} points, {observations} observations: " +
                         $"{clock.Elapsed.TotalSeconds:F2} s");
        Report(result, check);
        Assert.True(clock.Elapsed.TotalSeconds < 10, $"took {clock.Elapsed.TotalSeconds:F1} s");
        Assert.True(result.FinalRmsPx < 0.5, $"final RMS {result.FinalRmsPx:F3} px");
        Assert.True(result.Converged, $"not converged after {result.Iterations} iterations");
        AssertGeometry(check);
    }

    [Fact]
    public void Tracks_seen_by_fewer_than_two_photos_are_returned_as_they_came()
    {
        var random = new Random(5);
        var truth = Scene(random, cameras: 8, firstAzimuth: -35, azimuthStep: 10, points: 100, viewsPerPoint: int.MaxValue);
        var (views, tracks, _) = Perturb(random, truth, noisePx: 0.3, outlierFraction: 0);
        var lonely = new Track(new Vector3(0.01f, 0.02f, 0.03f), [Observe(truth, 0, Vector3.Zero, 0, 0)]);
        var unseen = new Track(new Vector3(0.04f, 0.05f, 0.06f), []);
        tracks.Add(lonely);
        tracks.Add(unseen);

        var result = BundleAdjuster.Adjust(views, tracks);

        Assert.Equal(tracks.Count, result.Points.Length);
        Assert.Equal(lonely.Point, result.Points[^2]);
        Assert.Equal(unseen.Point, result.Points[^1]);
        Assert.True(result.FinalRmsPx < 0.5, $"final RMS {result.FinalRmsPx:F3} px");
    }

    [Fact]
    public void Rejects_an_observation_of_a_photo_that_does_not_exist()
    {
        var views = new[] { new PhotoView(Blank, K, CameraPoses.LookAt(new Vector3(0, 0, -0.4f), Vector3.Zero)) };
        var tracks = new[] { new Track(Vector3.Zero, [new Observation(0, 959.5f, 539.5f), new Observation(1, 959.5f, 539.5f)]) };

        Assert.Throws<ArgumentException>(() => BundleAdjuster.Adjust(views, tracks));
    }

    private sealed record Truth(Matrix4x4[] Poses, Vector3[] Points, int[][] SeenBy);

    private sealed record Check(double Scale, double CentreDistanceMm, double RotationDeg, double PointDistanceMm, double CentreShiftMm);

    // Cameras 0.4 m from the centre of a 20 cm box of random points, 25 degrees above it, spaced along an arc.
    private static Truth Scene(Random random, int cameras, double firstAzimuth, double azimuthStep, int points, int viewsPerPoint)
    {
        var poses = new Matrix4x4[cameras];
        double elevation = 25 * Math.PI / 180;
        for (int c = 0; c < cameras; c++)
        {
            double azimuth = (firstAzimuth + c * azimuthStep) * Math.PI / 180;
            var eye = 0.4f * new Vector3((float)(Math.Sin(azimuth) * Math.Cos(elevation)), (float)Math.Sin(elevation),
                (float)(-Math.Cos(azimuth) * Math.Cos(elevation)));
            poses[c] = CameraPoses.LookAt(eye, Vector3.Zero);
        }

        var views = poses.Select(p => new PhotoView(Blank, K, p)).ToArray();
        var cloud = new List<Vector3>();
        var seenBy = new List<int[]>();
        while (cloud.Count < points)
        {
            var x = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * 0.2f
                    - new Vector3(0.1f);
            // Walk the cameras from a random one, as consecutive photos of a walk see the same part of the object.
            int start = random.Next(cameras);
            var seen = new List<int>();
            for (int k = 0; k < cameras && seen.Count < viewsPerPoint; k++)
            {
                int c = (start + k) % cameras;
                if (Pinhole.Sees(views[c], x)) seen.Add(c);
            }
            if (seen.Count < 2) continue;
            seen.Sort();
            cloud.Add(x);
            seenBy.Add(seen.ToArray());
        }
        return new Truth(poses, cloud.ToArray(), seenBy.ToArray());
    }

    // ARCore-like poses (centres off by about 1 cm, rotations by 0.5-1 degree), points off by about 5 mm,
    // observations with Gaussian noise and optionally a share of gross outliers (20-80 px off).
    private static (List<PhotoView> Views, List<Track> Tracks, bool[][] Outlier) Perturb(Random random, Truth truth,
        double noisePx, double outlierFraction)
    {
        var views = new List<PhotoView>();
        foreach (var pose in truth.Poses)
        {
            var axis = Vector3.Normalize(new Vector3((float)Gaussian(random), (float)Gaussian(random), (float)Gaussian(random)));
            double angle = (0.5 + 0.5 * random.NextDouble()) * Math.PI / 180;
            var rotation = Multiply(Exp(axis * (float)angle), Rotation(pose));
            var centre = pose.Translation + GaussianVector(random, 0.006);
            views.Add(new PhotoView(Blank, K, Pose(rotation, centre)));
        }

        var tracks = new List<Track>();
        var outlier = new bool[truth.Points.Length][];
        for (int i = 0; i < truth.Points.Length; i++)
        {
            var seen = truth.SeenBy[i];
            outlier[i] = new bool[seen.Length];
            var observations = new Observation[seen.Length];
            for (int k = 0; k < seen.Length; k++)
            {
                double dx = noisePx * Gaussian(random), dy = noisePx * Gaussian(random);
                if (random.NextDouble() < outlierFraction)
                {
                    double direction = random.NextDouble() * 2 * Math.PI, size = 20 + 60 * random.NextDouble();
                    dx = size * Math.Cos(direction);
                    dy = size * Math.Sin(direction);
                    outlier[i][k] = true;
                }
                observations[k] = Observe(truth, seen[k], truth.Points[i], dx, dy);
            }
            tracks.Add(new Track(truth.Points[i] + GaussianVector(random, 0.003), observations));
        }
        return (views, tracks, outlier);
    }

    // The projection computed in double precision, so exact data is exact to the last bit of the float pixel.
    private static Observation Observe(Truth truth, int view, Vector3 world, double dx, double dy)
    {
        var (u, v) = Project(truth.Poses[view], world);
        return new Observation(view, (float)(u + dx), (float)(v + dy));
    }

    private static (double U, double V) Project(Matrix4x4 pose, Vector3 world)
    {
        var r = Rotation(pose);
        double px = world.X - (double)pose.M41, py = world.Y - (double)pose.M42, pz = world.Z - (double)pose.M43;
        double x = r[0] * px + r[1] * py + r[2] * pz;
        double y = r[3] * px + r[4] * py + r[5] * pz;
        double z = r[6] * px + r[7] * py + r[8] * pz;
        return (K.Fx * x / z + K.Cx, K.Fy * y / z + K.Cy);
    }

    // The priors sit on the perturbed centres, so the adjusted scene is the true one moved by a similarity: absolute
    // positions, and even the scale (fixed only by those noisy priors), need not match. Distances are compared after
    // dividing out the least-squares scale; relative rotations need no correction.
    private static Check Compare(Truth truth, BundleResult result)
    {
        var trueCentres = truth.Poses.Select(p => p.Translation).ToArray();
        var centres = result.CameraToWorld.Select(p => p.Translation).ToArray();
        int n = centres.Length;

        double sumProduct = 0, sumSquare = 0;
        for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
        {
            double t = Vector3.Distance(trueCentres[i], trueCentres[j]), e = Vector3.Distance(centres[i], centres[j]);
            sumProduct += t * e;
            sumSquare += t * t;
        }
        double scale = sumProduct / sumSquare;

        double centreError = 0, rotationError = 0, pointError = 0, shift = 0;
        for (int i = 0; i < n; i++)
        {
            shift = Math.Max(shift, Vector3.Distance(trueCentres[i], centres[i]));
            for (int j = i + 1; j < n; j++)
            {
                double t = Vector3.Distance(trueCentres[i], trueCentres[j]), e = Vector3.Distance(centres[i], centres[j]);
                centreError = Math.Max(centreError, Math.Abs(e / scale - t));
                var trueRelative = MultiplyTransposed(Rotation(truth.Poses[i]), Rotation(truth.Poses[j]));
                var relative = MultiplyTransposed(Rotation(result.CameraToWorld[i]), Rotation(result.CameraToWorld[j]));
                rotationError = Math.Max(rotationError, RotationAngleDeg(trueRelative, relative));
            }
        }

        // Each point by its distances to the cameras that saw it.
        for (int p = 0; p < truth.Points.Length; p++)
        foreach (int c in truth.SeenBy[p])
        {
            double t = Vector3.Distance(truth.Points[p], trueCentres[c]), e = Vector3.Distance(result.Points[p], centres[c]);
            pointError = Math.Max(pointError, Math.Abs(e / scale - t));
        }
        return new Check(scale, centreError * 1000, rotationError, pointError * 1000, shift * 1000);
    }

    private static void AssertGeometry(Check check)
    {
        Assert.True(Math.Abs(check.Scale - 1) < 0.03, $"scale {check.Scale:F4}");
        Assert.True(check.CentreDistanceMm < 1, $"centre distances off by up to {check.CentreDistanceMm:F3} mm");
        Assert.True(check.RotationDeg < 0.05, $"relative rotations off by up to {check.RotationDeg:F4} deg");
        Assert.True(check.PointDistanceMm < 1, $"point-camera distances off by up to {check.PointDistanceMm:F3} mm");
        // The priors keep the cameras where ARCore put them, give or take its own error.
        Assert.True(check.CentreShiftMm < 40, $"a camera moved {check.CentreShiftMm:F1} mm from the truth");
    }

    private void Report(BundleResult result, Check check)
    {
        output.WriteLine($"RMS {result.InitialRmsPx:F2} -> {result.FinalRmsPx:F3} px in {result.Iterations} iterations, " +
                         $"converged {result.Converged}");
        output.WriteLine($"scale {check.Scale:F5}; after it, centre distances off by up to {check.CentreDistanceMm:F4} mm, " +
                         $"point-camera distances by up to {check.PointDistanceMm:F4} mm; relative rotations off by up to " +
                         $"{check.RotationDeg:F5} deg; largest centre shift from the truth {check.CentreShiftMm:F2} mm");
    }

    private static (List<double> Inliers, List<double> Outliers) Residuals(BundleResult result, List<Track> tracks, bool[][] outlier)
    {
        var inliers = new List<double>();
        var outliers = new List<double>();
        for (int i = 0; i < tracks.Count; i++)
        for (int k = 0; k < tracks[i].Observations.Length; k++)
        {
            var o = tracks[i].Observations[k];
            var (u, v) = Project(result.CameraToWorld[o.View], result.Points[i]);
            double error = Math.Sqrt((u - o.X) * (u - o.X) + (v - o.Y) * (v - o.Y));
            (outlier[i][k] ? outliers : inliers).Add(error);
        }
        return (inliers, outliers);
    }

    private static double Rms(List<double> values) => Math.Sqrt(values.Sum(v => v * v) / values.Count);

    private static double Median(List<double> values) => values.Order().ElementAt(values.Count / 2);

    private static double Gaussian(Random random) =>
        Math.Sqrt(-2 * Math.Log(1 - random.NextDouble())) * Math.Cos(2 * Math.PI * random.NextDouble());

    private static Vector3 GaussianVector(Random random, double sigma) =>
        new((float)(sigma * Gaussian(random)), (float)(sigma * Gaussian(random)), (float)(sigma * Gaussian(random)));

    // Row-major 3x3 world→camera rotation: the rows are the camera axes, as in the pose matrix.
    private static double[] Rotation(Matrix4x4 m) => [m.M11, m.M12, m.M13, m.M21, m.M22, m.M23, m.M31, m.M32, m.M33];

    private static Matrix4x4 Pose(double[] r, Vector3 centre) => new(
        (float)r[0], (float)r[1], (float)r[2], 0,
        (float)r[3], (float)r[4], (float)r[5], 0,
        (float)r[6], (float)r[7], (float)r[8], 0,
        centre.X, centre.Y, centre.Z, 1);

    private static double[] Multiply(double[] a, double[] b)
    {
        var m = new double[9];
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
        return m;
    }

    // a·bᵀ
    private static double[] MultiplyTransposed(double[] a, double[] b)
    {
        var m = new double[9];
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m[i * 3 + j] = a[i * 3] * b[j * 3] + a[i * 3 + 1] * b[j * 3 + 1] + a[i * 3 + 2] * b[j * 3 + 2];
        return m;
    }

    private static double[] Exp(Vector3 w)
    {
        double theta = w.Length();
        if (theta == 0) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
        double x = w.X / theta, y = w.Y / theta, z = w.Z / theta, s = Math.Sin(theta), c = 1 - Math.Cos(theta);
        return
        [
            1 + c * (x * x - 1), -s * z + c * x * y, s * y + c * x * z,
            s * z + c * x * y, 1 + c * (y * y - 1), -s * x + c * y * z,
            -s * y + c * x * z, s * x + c * y * z, 1 + c * (z * z - 1),
        ];
    }

    // The angle of a·bᵀ from its antisymmetric part and trace, which stays accurate for tiny angles.
    private static double RotationAngleDeg(double[] a, double[] b)
    {
        var m = MultiplyTransposed(a, b);
        double sx = m[7] - m[5], sy = m[2] - m[6], sz = m[3] - m[1];
        return Math.Atan2(Math.Sqrt(sx * sx + sy * sy + sz * sz), m[0] + m[4] + m[8] - 1) * 180 / Math.PI;
    }
}
