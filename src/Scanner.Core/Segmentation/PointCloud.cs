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
    int Seed = 1);

public sealed record DetectedShape(Primitive Primitive, int[] InlierIndices);
