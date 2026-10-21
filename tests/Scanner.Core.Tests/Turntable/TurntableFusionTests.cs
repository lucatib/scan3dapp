using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Synthetic;
using Scanner.Core.Turntable;

namespace Scanner.Core.Tests.Turntable;

public class TurntableFusionTests
{
    private static readonly CameraIntrinsics Camera = new(200, 150, 170, 170, 99.5f, 74.5f);
    private static readonly Matrix4x4 CameraPose = CameraPoses.LookAt(new Vector3(0, 0.17f, 0.24f), new Vector3(0, 0.03f, 0));

    /// <summary>A table (y below 0, never turning) and a piece on it: a box with a ball on one corner, so a turn shows.</summary>
    private static readonly ISdf Table = new BoxSdf(new Vector3(0, -0.01f, 0), new Vector3(0.3f, 0.01f, 0.3f));

    private static readonly ISdf Piece = new UnionSdf([
        new BoxSdf(new Vector3(0.01f, 0.02f, 0), new Vector3(0.03f, 0.02f, 0.02f)),
        new SphereSdf(new Vector3(-0.015f, 0.05f, 0.01f), 0.015f),
    ]);

    [Fact]
    public void Follows_a_full_turn_and_builds_the_piece_standing_on_the_table()
    {
        const float step = 6;
        TurntableFusion? fusion = null;
        int accepted = 0;
        for (int i = 0; i < 60; i++)
        {
            var frame = Render(i * step, seed: i);
            fusion ??= new TurntableFusion(CentreTarget(frame), new TurntableOptions(VoxelSize: 0.002f, MaxRms: 0.002f));
            if (fusion.Add(frame).Accepted) accepted++;
        }

        Assert.True(accepted >= 57, $"accepted {accepted} of 60");
        Assert.InRange(fusion!.TurnDegrees, 59 * step * 0.97f, 59 * step * 1.03f);

        var mesh = fusion.Mesh();
        Assert.True(mesh.TriangleCount > 500);
        float top = mesh.Positions.Max(p => p.Y), bottom = mesh.Positions.Min(p => p.Y);
        Assert.InRange(top, 0.065f - 0.004f, 0.065f + 0.004f); // the ball's top
        Assert.InRange(bottom, -0.001f, 0.001f); // flat on the table
    }

    [Fact]
    public void Finds_the_table_rather_than_the_top_of_the_piece()
    {
        var frame = Render(0, seed: 0);
        var fusion = new TurntableFusion(CentreTarget(frame));
        fusion.Add(frame);

        var (normal, point) = fusion.Table!.Value;
        var worldNormal = Vector3.TransformNormal(normal, CameraPose);
        Assert.True(worldNormal.Y > 0.99f);
        Assert.InRange(Vector3.Transform(point, CameraPose).Y, -0.001f, 0.001f);
    }

    private static DepthFrame Render(float degrees, int seed)
    {
        var turned = new Turned(Piece, degrees * MathF.PI / 180);
        return SyntheticDepthRenderer.Render(new UnionSdf([Table, turned]), Camera, CameraPose, noiseSigma: 0.0003f, seed: seed);
    }

    /// <summary>The point under the centre pixel, in camera coordinates: what the crosshair picks.</summary>
    private static Vector3 CentreTarget(DepthFrame frame)
    {
        int u = Camera.Width / 2, v = Camera.Height / 2;
        float d = frame.DepthAt(u, v);
        return new Vector3((u - Camera.Cx) / Camera.Fx * d, (v - Camera.Cy) / Camera.Fy * d, d);
    }

    /// <summary>A shape turned about the vertical axis through the origin (the turntable's).</summary>
    private sealed record Turned(ISdf Inner, float Radians) : ISdf
    {
        private readonly Matrix4x4 _back = Matrix4x4.CreateRotationY(-Radians);

        public float Distance(Vector3 p) => Inner.Distance(Vector3.Transform(p, _back));
    }
}
