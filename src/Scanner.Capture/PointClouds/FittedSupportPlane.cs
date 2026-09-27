namespace Scanner.Capture.PointClouds;

/// <summary>Table surface y = SlopeX*x + SlopeZ*z + Offset, in world metres.</summary>
public readonly record struct FittedSupportPlane(float SlopeX, float SlopeZ, float Offset, float Margin = .004f)
{
    public float HeightAt(float x, float z) => SlopeX * x + SlopeZ * z + Offset;
}
