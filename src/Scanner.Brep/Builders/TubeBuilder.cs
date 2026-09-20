using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Brep.Builders;

/// <summary>
/// Outer cylinder with an optional coaxial hole and two planar caps orthogonal to the axis.
/// Each circle is a single closed edge (start vertex = end vertex), traversed counterclockwise around the axis.
/// Orientations (face interior on the left when looking along the face normal):
/// outer: bottom +, top −; hole (SameSense=false): bottom −, top +;
/// bottom cap (normal −axis): outer −, hole +; top cap (normal +axis): outer +, hole −.
/// </summary>
public static class TubeBuilder
{
    public static BrepSolid Build(Vector3 axisPoint, Vector3 axis, float outerRadius, float? innerRadius, float zBottom, float zTop)
    {
        // Every guard is written in positive form ("reject unless valid"): any comparison against NaN
        // is false, so an "if (invalid) throw" phrasing would let NaN inputs through into the geometry.
        // LengthSquared is finite only when every component is, so this one test covers all three.
        if (!float.IsFinite(axisPoint.LengthSquared()))
            throw new ArgumentOutOfRangeException(nameof(axisPoint), axisPoint, "The axis point must be finite.");
        float axisLength = axis.Length();
        if (!float.IsFinite(axisLength) || axisLength < 1e-6f)
            throw new ArgumentOutOfRangeException(nameof(axis), axis, "The axis must be finite and of non-negligible length.");
        if (!(float.IsFinite(outerRadius) && outerRadius > 0))
            throw new ArgumentOutOfRangeException(nameof(outerRadius), outerRadius, "The outer radius must be finite and strictly positive.");
        if (innerRadius is { } r && !(float.IsFinite(r) && r > 0 && r < outerRadius))
            throw new ArgumentOutOfRangeException(nameof(innerRadius), r, "The hole radius must be finite and within (0, outerRadius).");
        if (!float.IsFinite(zBottom))
            throw new ArgumentOutOfRangeException(nameof(zBottom), zBottom, "zBottom must be finite.");
        if (!(float.IsFinite(zTop) && zTop > zBottom))
            throw new ArgumentOutOfRangeException(nameof(zTop), zTop, "zTop must be finite and greater than zBottom.");

        // Divide by the length already validated above: Vector3.Normalize(Vector3.Zero) returns NaN silently.
        var a = axis / axisLength;
        var (refDirection, _) = Basis.Orthonormal(a);
        var bottom = axisPoint + a * zBottom;
        var top = axisPoint + a * zTop;

        var outerBottom = Circle(bottom, a, refDirection, outerRadius);
        var outerTop = Circle(top, a, refDirection, outerRadius);

        var faces = new List<BrepFace>
        {
            new(new CylinderSurface(bottom, a, refDirection, outerRadius),
                [Loop(outerBottom, true), Loop(outerTop, false)], sameSense: true),
        };

        var bottomLoops = new List<BrepLoop> { Loop(outerBottom, false) };
        var topLoops = new List<BrepLoop> { Loop(outerTop, true) };

        if (innerRadius is { } inner)
        {
            // Structurally a through-hole: the hole always spans the outer cylinder's bottom/top,
            // so a blind or partially drilled hole cannot be expressed through this API.
            var innerBottom = Circle(bottom, a, refDirection, inner);
            var innerTop = Circle(top, a, refDirection, inner);
            // The surface origin is `bottom` because a cylinder only needs any point on its axis, and
            // the hole shares the outer cylinder's axis; it is not the hole's own centre or base point.
            faces.Add(new BrepFace(new CylinderSurface(bottom, a, refDirection, inner),
                [Loop(innerBottom, false), Loop(innerTop, true)], sameSense: false));
            bottomLoops.Add(Loop(innerBottom, true));
            topLoops.Add(Loop(innerTop, false));
        }

        faces.Add(new BrepFace(new PlaneSurface(bottom, -a, refDirection), bottomLoops, sameSense: true));
        faces.Add(new BrepFace(new PlaneSurface(top, a, refDirection), topLoops, sameSense: true));
        return new BrepSolid(faces);
    }

    private static BrepEdge Circle(Vector3 center, Vector3 axis, Vector3 refDirection, float radius)
    {
        var vertex = new BrepVertex(center + refDirection * radius);
        return new BrepEdge(vertex, vertex, new CircleCurve(center, axis, refDirection, radius));
    }

    private static BrepLoop Loop(BrepEdge edge, bool sameSense) => new([new OrientedEdge(edge, sameSense)]);
}
