using System.Numerics;

namespace Scanner.Core.Shapes;

public abstract record Primitive;

/// <summary>Plane Normal·x = D, with Normal a unit vector pointing out of the solid.</summary>
public sealed record PlanePrimitive(Vector3 Normal, float D) : Primitive
{
    public float SignedDistance(Vector3 p) => Vector3.Dot(Normal, p) - D;
}

/// <summary>Infinite cylinder with axis through AxisPoint in direction Axis (unit vector).
/// IsHole is true when the surface normals point toward the axis (a hole).</summary>
public sealed record CylinderPrimitive(Vector3 AxisPoint, Vector3 Axis, float Radius, bool IsHole) : Primitive
{
    public Vector3 RadialVector(Vector3 p)
    {
        var d = p - AxisPoint;
        return d - Vector3.Dot(d, Axis) * Axis;
    }

    public float Distance(Vector3 p) => MathF.Abs(RadialVector(p).Length() - Radius);
}
