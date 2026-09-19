using System.Numerics;

namespace Scanner.Core.Synthetic;

/// <summary>Signed distance function: negative inside the object.</summary>
public interface ISdf
{
    float Distance(Vector3 p);
}

public sealed record SphereSdf(Vector3 Center, float Radius) : ISdf
{
    public float Distance(Vector3 p) => Vector3.Distance(p, Center) - Radius;
}

public sealed record BoxSdf(Vector3 Center, Vector3 HalfSize) : ISdf
{
    public float Distance(Vector3 p)
    {
        var q = Vector3.Abs(p - Center) - HalfSize;
        float outside = Vector3.Max(q, Vector3.Zero).Length();
        float inside = MathF.Min(MathF.Max(q.X, MathF.Max(q.Y, q.Z)), 0f);
        return outside + inside;
    }
}

/// <summary>Drilled cylinder with Z axis centered at <see cref="Center"/>.</summary>
public sealed record TubeSdf(Vector3 Center, float OuterRadius, float InnerRadius, float Height) : ISdf
{
    public float Distance(Vector3 p)
    {
        var d = p - Center;
        float r = MathF.Sqrt(d.X * d.X + d.Y * d.Y);
        float dr = MathF.Abs(r - (OuterRadius + InnerRadius) / 2) - (OuterRadius - InnerRadius) / 2;
        float dz = MathF.Abs(d.Z) - Height / 2;
        float outside = new Vector2(MathF.Max(dr, 0f), MathF.Max(dz, 0f)).Length();
        float inside = MathF.Min(MathF.Max(dr, dz), 0f);
        return outside + inside;
    }
}
