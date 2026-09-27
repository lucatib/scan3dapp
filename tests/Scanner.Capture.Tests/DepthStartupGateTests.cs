using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class DepthStartupGateTests
{
    private static Vector3[] Surface(float height, int count = 1000) =>
        Enumerable.Range(0, count).Select(i => new Vector3((i % 40) * .005f, height, (i / 40) * .005f)).ToArray();

    [Fact]
    public void Sparse_frames_never_start_accumulation()
    {
        var gate = new DepthStartupGate();
        for (int i = 0; i < 20; i++) Assert.False(gate.Accept(Surface(0, 300), 14400));
    }

    [Fact]
    public void Surface_jumps_restart_the_consecutive_frame_check()
    {
        var gate = new DepthStartupGate();
        Assert.False(gate.Accept(Surface(0), 14400));
        Assert.False(gate.Accept(Surface(.004f), 14400));
        Assert.False(gate.Accept(Surface(.04f), 14400));
        Assert.False(gate.Accept(Surface(.042f), 14400));
        Assert.True(gate.Accept(Surface(.041f), 14400));
        // Once started, occlusion or a change of viewpoint must not restart startup.
        Assert.True(gate.Accept(Surface(.08f, 200), 14400));
    }

    [Fact]
    public void Sparse_frame_breaks_a_stable_sequence()
    {
        var gate = new DepthStartupGate();
        Assert.False(gate.Accept(Surface(0), 14400));
        Assert.False(gate.Accept(Surface(0), 14400));
        Assert.False(gate.Accept(Surface(0, 100), 14400));
        Assert.False(gate.Accept(Surface(0), 14400));
        Assert.False(gate.Accept(Surface(0), 14400));
        Assert.True(gate.Accept(Surface(0), 14400));
    }
}
