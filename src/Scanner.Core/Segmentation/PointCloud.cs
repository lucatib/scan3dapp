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
    int MinCoverageBins = 18);

public sealed record DetectedShape(Primitive Primitive, int[] InlierIndices);
