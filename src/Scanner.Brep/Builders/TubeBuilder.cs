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
        if (zTop <= zBottom) throw new ArgumentException("zTop must be greater than zBottom.");
        if (innerRadius is { } r && (r <= 0 || r >= outerRadius)) throw new ArgumentException("Invalid hole radius.");

        var a = Vector3.Normalize(axis);
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
            var innerBottom = Circle(bottom, a, refDirection, inner);
            var innerTop = Circle(top, a, refDirection, inner);
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
