using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Waits for a sufficiently populated, stable horizontal surface before starting accumulation.
/// Used only at startup; raw frames remain available for diagnosis. Call from the integration worker.</summary>
public sealed class DepthStartupGate
{
    private int _consistentFrames;
    private float _minimum;
    private float _maximum;
    public bool IsReady { get; private set; }

    public bool Accept(IReadOnlyList<Vector3> points, int imagePixels)
    {
        if (IsReady) return true;
        float? height = points.Count >= Math.Max(500, imagePixels / 20)
            ? SupportPlaneFinder.Find(points) : null;
        if (height is not { } y)
        {
            _consistentFrames = 0;
            return false;
        }
        float min = MathF.Min(_minimum, y), max = MathF.Max(_maximum, y);
        if (_consistentFrames == 0 || max - min > .01f)
        {
            _minimum = _maximum = y;
            _consistentFrames = 1;
            return false;
        }
        _minimum = min;
        _maximum = max;
        IsReady = ++_consistentFrames >= 3;
        return IsReady;
    }
}
