using System.Numerics;
using Scanner.Core.Photogrammetry;

namespace Scanner.Core.Tests.Photogrammetry;

public class LoopCoverageTests
{
    private static readonly Vector3 Target = new(0.2f, -0.4f, -0.3f);

    private static Vector3 Camera(float azimuthDegrees, float height = 0.3f)
    {
        float a = azimuthDegrees * MathF.PI / 180;
        return Target + new Vector3(0.35f * MathF.Cos(a), height, 0.35f * MathF.Sin(a));
    }

    [Fact]
    public void A_full_loop_leaves_only_the_step_between_photos()
    {
        var cameras = Enumerable.Range(0, 36).Select(i => Camera(i * 10));

        Assert.Equal(10, LoopCoverage.LargestGapDegrees(cameras, Target), 0.01f);
    }

    [Fact]
    public void Photos_over_250_degrees_leave_a_110_degree_gap_even_across_the_wrap()
    {
        // From 150° up through 180° to 40°: the gap is 40°..150°, and the photos cross the ±180° seam.
        var cameras = Enumerable.Range(0, 51).Select(i => Camera(150 + i * 5, 0.2f + 0.002f * i));

        Assert.Equal(110, LoopCoverage.LargestGapDegrees(cameras, Target), 0.01f);
    }

    [Fact]
    public void Fewer_than_two_photos_cover_nothing()
    {
        Assert.Equal(360, LoopCoverage.LargestGapDegrees([], Target));
        Assert.Equal(360, LoopCoverage.LargestGapDegrees([Camera(30)], Target));
    }
}
