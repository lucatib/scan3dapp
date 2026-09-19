using System.Numerics;

namespace Scanner.Core.LinearAlgebra;

public static class Basis
{
    /// <summary>Base ortonormale (U, V) del piano ortogonale a <paramref name="n"/> (unitario), con U × V = n.</summary>
    public static (Vector3 U, Vector3 V) Orthonormal(Vector3 n)
    {
        var helper = MathF.Abs(n.X) < 0.9f ? Vector3.UnitX : Vector3.UnitY;
        var u = Vector3.Normalize(Vector3.Cross(helper, n));
        var v = Vector3.Cross(n, u);
        return (u, v);
    }
}
