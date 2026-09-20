using System.Numerics;
using Scanner.Core.Meshing;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

public sealed record PointCloud(Vector3[] Points, Vector3[] Normals)
{
    public int Count => Points.Length;

    public static PointCloud FromMesh(TriangleMesh mesh) => new(mesh.Positions.ToArray(), mesh.Normals.ToArray());
}

public sealed record RansacOptions(
    float DistanceThreshold,
    float NormalThresholdDegrees,
    int MinInliers,
    int IterationsPerShape = 1500,
    int MaxShapes = 20,
    float MaxCylinderRadius = 0.5f,
    int Seed = 1,
    // Angular span, in 5° bins of the sweep around its own axis, that a cylinder candidate's inliers must
    // cover; 18 bins = 90°. What the span guarantees is "no gap longer than the candidate's own mean bin
    // spacing", not "every bin occupied" - see RansacDetector.MeasureCoverage.
    //
    // Deliberately NOT clamped, because a value past a full circle is a meaningful way to switch cylinder
    // detection off entirely. The useful range is [5, 72]. At 0 or below the gate never rejects anything,
    // since a span is never negative. Below 5 (25°) the gate stops being the binding constraint: it admits
    // candidates narrower than the ~20° arc PrimitiveFitter.FitCylinder needs to fit an axis at all, so
    // Refine returns null and detection silently falls back to unrefined primitives instead of reporting a
    // narrow shape. Above 72 every cylinder is rejected, which is the switch-off case.
    int MinCoverageBins = 18);

public sealed record DetectedShape(Primitive Primitive, int[] InlierIndices);
