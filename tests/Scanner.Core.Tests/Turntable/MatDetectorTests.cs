using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Synthetic;
using Scanner.Core.Turntable;

namespace Scanner.Core.Tests.Turntable;

public class MatDetectorTests
{
    private static readonly CameraIntrinsics Camera = new(1280, 960, 1050, 1050, 639.5f, 479.5f);

    [Fact]
    public void Code_windows_are_unique_both_ways_round()
    {
        Assert.Equal(TurntableMat.DotCount, TurntableMat.Code.Count);
        for (int start = 0; start < TurntableMat.DotCount; start++)
        for (int direction = -1; direction <= 1; direction += 2)
        {
            var window = Enumerable.Range(0, TurntableMat.WindowLength)
                .Select(i => TurntableMat.Code[((start + direction * i) % 36 + 36) % 36]).ToArray();
            Assert.Equal((start, direction), TurntableMat.Locate(window));
        }
    }

    [Theory]
    [InlineData(0, 35)]
    [InlineData(47, 25)]
    [InlineData(200, 50)]
    public void Recovers_the_camera_pose_with_the_piece_hiding_part_of_the_ring(float turnDegrees, float elevationDegrees)
    {
        var pose = CameraOnMat(turnDegrees, elevationDegrees, distance: 0.32f);
        var image = Render(pose, pieceRadius: 0.045f, pieceHeight: 0.08f);

        var dots = MatDetector.Detect(image);
        foreach (var dot in dots) // every dot named is the right one
        {
            var expected = Project(pose, TurntableMat.DotPosition(dot.Index));
            Assert.True(Vector2.Distance(expected, dot.Pixel) < 1.5f, $"dot {dot.Index} at {dot.Pixel}, expected {expected}");
        }
        Assert.True(dots.Count >= 20, $"{dots.Count} dots");

        var result = MatPose.Solve(dots, Camera);
        Assert.NotNull(result);
        Assert.True(result.RmsPixels < 0.5f, $"rms {result.RmsPixels}");
        Assert.True(Vector3.Distance(result.CameraToMat.Translation, pose.Translation) < 0.001f,
            $"centre {result.CameraToMat.Translation} vs {pose.Translation}");
        var look = Vector3.TransformNormal(Vector3.UnitZ, result.CameraToMat);
        Assert.True(Vector3.Dot(look, Vector3.TransformNormal(Vector3.UnitZ, pose)) > MathF.Cos(0.2f * MathF.PI / 180));
    }

    [Fact]
    public void Finds_nothing_in_a_photo_without_the_mat()
    {
        var pixels = new byte[640 * 480];
        var rng = new Random(3);
        for (int i = 0; i < pixels.Length; i++) pixels[i] = (byte)(120 + rng.Next(20));
        Assert.Empty(MatDetector.Detect(new GrayImage(640, 480, pixels)));
    }

    /// <summary>A camera looking at the mat centre from <paramref name="elevationDegrees"/> above it, with the mat turned
    /// by <paramref name="turnDegrees"/> (the same as the camera going round the other way).</summary>
    internal static Matrix4x4 CameraOnMat(float turnDegrees, float elevationDegrees, float distance)
    {
        float azimuth = -turnDegrees * MathF.PI / 180, elevation = elevationDegrees * MathF.PI / 180;
        var eye = distance * new Vector3(MathF.Cos(elevation) * MathF.Cos(azimuth), MathF.Sin(elevation),
            MathF.Cos(elevation) * MathF.Sin(azimuth));
        return CameraPoses.LookAt(eye, new Vector3(0, 0.02f, 0));
    }

    internal static Vector2 Project(Matrix4x4 cameraToWorld, Vector3 p)
    {
        Matrix4x4.Invert(cameraToWorld, out var toCamera);
        var c = Vector3.Transform(p, toCamera);
        return new Vector2(Camera.Fx * c.X / c.Z + Camera.Cx, Camera.Fy * c.Y / c.Z + Camera.Cy);
    }

    /// <summary>The mat on a grey table with a grey cylinder (the piece) on it, 2×2 supersampled.</summary>
    internal static GrayImage Render(Matrix4x4 cameraToWorld, float pieceRadius, float pieceHeight)
    {
        var k = Camera;
        var pixels = new byte[k.Width * k.Height];
        var eye = cameraToWorld.Translation;
        Parallel.For(0, k.Height, v =>
        {
            for (int u = 0; u < k.Width; u++)
            {
                int sum = 0;
                for (int sy = 0; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++)
                {
                    var ray = new Vector3((u - 0.25f + 0.5f * sx - k.Cx) / k.Fx, (v - 0.25f + 0.5f * sy - k.Cy) / k.Fy, 1);
                    sum += Shade(eye, Vector3.Normalize(Vector3.TransformNormal(ray, cameraToWorld)), pieceRadius, pieceHeight);
                }
                pixels[v * k.Width + u] = (byte)(sum / 4);
            }
        });
        return new GrayImage(k.Width, k.Height, pixels);
    }

    private static int Shade(Vector3 eye, Vector3 direction, float pieceRadius, float pieceHeight)
    {
        if (direction.Y >= 0) return 140;
        // The piece: a vertical cylinder, its side hit first if at all.
        float a = direction.X * direction.X + direction.Z * direction.Z;
        float b = 2 * (eye.X * direction.X + eye.Z * direction.Z);
        float c = eye.X * eye.X + eye.Z * eye.Z - pieceRadius * pieceRadius;
        float disc = b * b - 4 * a * c;
        if (disc >= 0)
        {
            float t = (-b - MathF.Sqrt(disc)) / (2 * a);
            float y = eye.Y + t * direction.Y;
            if (t > 0 && y >= 0 && y <= pieceHeight) return 90;
            float tTop = (pieceHeight - eye.Y) / direction.Y;
            var top = eye + tTop * direction;
            if (top.X * top.X + top.Z * top.Z <= pieceRadius * pieceRadius) return 110;
        }
        float tTable = -eye.Y / direction.Y;
        var p = eye + tTable * direction;
        float r = MathF.Sqrt(p.X * p.X + p.Z * p.Z);
        if (r > TurntableMat.PlatterRadius) return 140; // the table around the platter
        for (int i = 0; i < TurntableMat.DotCount; i++)
        {
            float d = Vector3.Distance(p, TurntableMat.DotPosition(i));
            if (d > TurntableMat.DotRadius) continue;
            return TurntableMat.Code[i] && d < TurntableMat.HoleRadius ? 235 : 25;
        }
        return 235;
    }
}
