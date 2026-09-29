using System.Numerics;
using Scanner.Capture;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// Where the photos look: the point nearest to every photo's optical axis. The user keeps the piece under the crosshair
/// while walking around it, so the axes meet at the piece. It locates the piece when ARCore cannot: in a dim room its
/// depth once put a table 0.45 m away at 2-5 m for a whole scan, so no depth point passed and the target it picked was
/// off the piece, while every photo showed the grinder.
/// </summary>
public static class AimPoint
{
    /// <summary>Photos needed before the aim point is trusted.</summary>
    public const int MinViews = 8;

    /// <summary>Some axis must turn at least this far from the average one, or the rays are too parallel to meet.</summary>
    private const float MinSpreadDegrees = 8f;

    /// <summary>The point, or null with too few photos, rays too parallel to meet, or a point most photos do not face.</summary>
    public static Vector3? Estimate(IReadOnlyList<PhotoView> views)
    {
        if (views.Count < MinViews) return null;
        var axes = views.Select(v => Vector3.Normalize(Vector3.TransformNormal(Vector3.UnitZ, v.CameraToWorld))).ToArray();
        var mean = Vector3.Normalize(axes.Aggregate(Vector3.Zero, (a, b) => a + b));
        float spread = axes.Max(a => MathF.Acos(Math.Clamp(Vector3.Dot(a, mean), -1f, 1f))) * 180f / MathF.PI;
        if (spread < MinSpreadDegrees) return null;

        // Σ (I − d dᵀ) x = Σ (I − d dᵀ) c: least squares over the distances to the axes.
        var a = new double[3, 3];
        var b = new double[3];
        for (int i = 0; i < views.Count; i++)
        {
            var d = axes[i];
            var c = views[i].CameraToWorld.Translation;
            double[] dv = [d.X, d.Y, d.Z], cv = [c.X, c.Y, c.Z];
            for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
            {
                double m = (r == col ? 1 : 0) - dv[r] * dv[col];
                a[r, col] += m;
                b[r] += m * cv[col];
            }
        }
        if (!Linear3.TrySolve(a, b, out var x)) return null;
        var point = new Vector3((float)x[0], (float)x[1], (float)x[2]);

        int facing = views.Count(v => Pinhole.ToCamera(v.CameraToWorld, point).Z > 0.05f);
        return facing * 5 >= views.Count * 4 ? point : null;
    }
}
