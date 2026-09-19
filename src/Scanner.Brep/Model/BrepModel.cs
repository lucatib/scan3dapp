using System.Numerics;

namespace Scanner.Brep.Model;

public sealed class BrepVertex(Vector3 position)
{
    public Vector3 Position { get; } = position;
}

public abstract record BrepCurve;

/// <summary>Line through Origin with unit direction Direction.</summary>
public sealed record LineCurve(Vector3 Origin, Vector3 Direction) : BrepCurve;

/// <summary>Circle in the plane orthogonal to Axis, traversed counterclockwise around Axis starting from RefDirection.</summary>
public sealed record CircleCurve(Vector3 Center, Vector3 Axis, Vector3 RefDirection, float Radius) : BrepCurve;

public sealed class BrepEdge(BrepVertex start, BrepVertex end, BrepCurve curve)
{
    public BrepVertex Start { get; } = start;
    public BrepVertex End { get; } = end;
    public BrepCurve Curve { get; } = curve;
}

public readonly record struct OrientedEdge(BrepEdge Edge, bool SameSense)
{
    public BrepVertex StartVertex => SameSense ? Edge.Start : Edge.End;
    public BrepVertex EndVertex => SameSense ? Edge.End : Edge.Start;
}

/// <summary>Closed cycle of oriented edges; the face interior is on the left when looking along the face normal.</summary>
public sealed class BrepLoop(IReadOnlyList<OrientedEdge> edges)
{
    public IReadOnlyList<OrientedEdge> Edges { get; } = edges;
}

public abstract record BrepSurface;

public sealed record PlaneSurface(Vector3 Origin, Vector3 Normal, Vector3 RefDirection) : BrepSurface;

/// <summary>Cylindrical surface; its normal points away from the axis.</summary>
public sealed record CylinderSurface(Vector3 Origin, Vector3 Axis, Vector3 RefDirection, float Radius) : BrepSurface;

/// <summary>Face bounded by one or more loops; for planar faces Loops[0] is the outer boundary.
/// SameSense = false flips the surface normal.</summary>
public sealed class BrepFace(BrepSurface surface, IReadOnlyList<BrepLoop> loops, bool sameSense)
{
    public BrepSurface Surface { get; } = surface;
    public IReadOnlyList<BrepLoop> Loops { get; } = loops;
    public bool SameSense { get; } = sameSense;
}

public sealed class BrepSolid(IReadOnlyList<BrepFace> faces)
{
    public IReadOnlyList<BrepFace> Faces { get; } = faces;

    public IReadOnlyList<BrepEdge> DistinctEdges() =>
        Faces.SelectMany(f => f.Loops).SelectMany(l => l.Edges).Select(e => e.Edge).Distinct().ToList();

    public IReadOnlyList<BrepVertex> DistinctVertices() =>
        DistinctEdges().SelectMany(e => new[] { e.Start, e.End }).Distinct().ToList();
}
