using System.Numerics;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Brep.Model;

/// <summary>
/// Checks that a solid is a closed manifold shell and that its geometry agrees with that topology.
/// <para>
/// The topological pass alone is a checksum: it never reads a coordinate, so a face whose loops do not
/// lie on its own plane, or a shell that is consistently inside out, passes it unchanged. Written to
/// STEP, either produces a file that is syntactically perfect and geometrically wrong, which imports
/// and only then misbehaves. The geometric pass therefore checks planarity face by face and the sign of
/// the volume of the whole shell; it runs only once the topology is sound, because a broken loop makes
/// every measurement meaningless.
/// </para>
/// </summary>
public static class BrepValidator
{
    /// <summary>
    /// Out-of-plane tolerance in meters, 5 mm.
    /// <para>
    /// Deliberately loose. The primitives a face is built from are only as accurate as the detector's
    /// distance threshold - 4 mm on the Draft profile, and its fallback path knowingly emits primitives
    /// at exactly that accuracy - so a tolerance derived from least-squares accuracy would reject real
    /// scans for being precisely as accurate as they were designed to be. What this check is for is
    /// gross errors, a loop attached to the wrong face or a plane built from the wrong normal, which are
    /// on the scale of the object rather than of the fit. Callers holding tighter data may pass less.
    /// </para>
    /// </summary>
    public const float DefaultPlanarityTolerance = 5e-3f;

    /// <summary>Smallest |cos| accepted between two directions that must be parallel, about 2.6 degrees.</summary>
    private const float ParallelCosLimit = 0.999f;

    public static IReadOnlyList<string> Validate(BrepSolid solid, float planarityTolerance = DefaultPlanarityTolerance)
    {
        var errors = ValidateTopology(solid);
        if (errors.Count > 0) return errors;
        ValidateGeometry(solid, planarityTolerance, errors);
        return errors;
    }

    private static List<string> ValidateTopology(BrepSolid solid)
    {
        var errors = new List<string>();
        var uses = new Dictionary<BrepEdge, (int Forward, int Backward)>();
        var edgesInLoop = new HashSet<BrepEdge>();
        int loopCount = 0;

        for (int f = 0; f < solid.Faces.Count; f++)
        foreach (var loop in solid.Faces[f].Loops)
        {
            loopCount++;
            if (loop.Edges.Count == 0)
            {
                errors.Add($"Face {f}: empty loop.");
                continue;
            }
            // Not "fewer than three edges": a circular loop legitimately has exactly one. What is never
            // legitimate is walking the same edge twice, the degenerate sliver [e forward, e backward],
            // which balances the use counts and keeps the characteristic at 2 - no other rule sees it.
            edgesInLoop.Clear();
            for (int i = 0; i < loop.Edges.Count; i++)
            {
                var current = loop.Edges[i];
                var next = loop.Edges[(i + 1) % loop.Edges.Count];
                if (current.EndVertex != next.StartVertex)
                    errors.Add($"Face {f}: loop not closed after edge {i}.");
                if (!edgesInLoop.Add(current.Edge))
                    errors.Add($"Face {f}: a loop uses the same edge twice.");

                uses.TryGetValue(current.Edge, out var count);
                uses[current.Edge] = current.SameSense ? (count.Forward + 1, count.Backward) : (count.Forward, count.Backward + 1);
            }
        }

        foreach (var (_, count) in uses)
            if (count.Forward != 1 || count.Backward != 1)
                errors.Add($"Edge used {count.Forward} times forward and {count.Backward} backward (expected 1 and 1).");

        int vertices = solid.DistinctVertices().Count;
        int faces = solid.Faces.Count;
        // Euler-Poincaré: V − E + F − (L − F) = 2(S − G), with S = 1 shell.
        int chi = vertices - uses.Count + faces - (loopCount - faces);
        if (chi > 2 || chi % 2 != 0)
            errors.Add($"Invalid Euler-Poincaré characteristic: {chi}.");

        return errors;
    }

    /// <summary>
    /// Per-face planarity and the sign of the volume of the shell. The volume is exact rather than
    /// tessellated: by the divergence theorem with F = p/3, each face contributes (p·n)/3 integrated
    /// over its area, which is (origin·normal)·area/3 for a planar face and 2*pi*r^2*height/3 for a full
    /// cylindrical one. A consistently inverted shell is the one defect no topological rule can see.
    /// </summary>
    private static void ValidateGeometry(BrepSolid solid, float planarityTolerance, List<string> errors)
    {
        double volume = 0;
        bool measured = true;
        for (int f = 0; f < solid.Faces.Count; f++)
        {
            var face = solid.Faces[f];
            double? contribution = face.Surface switch
            {
                PlaneSurface plane => PlanarFaceVolume(f, face, plane, planarityTolerance, errors),
                CylinderSurface cylinder => CylindricalFaceVolume(f, face, cylinder, planarityTolerance, errors),
                var other => Unsupported(f, other, errors),
            };
            if (contribution is { } value) volume += value;
            else measured = false;
        }

        // A face that could not be measured leaves the total meaningless, so do not read a sign into it.
        if (measured && !(volume > 0))
            errors.Add(FormattableString.Invariant(
                $"The shell encloses a non-positive volume ({volume} m3): its faces point into the material."));
    }

    private static double? PlanarFaceVolume(int f, BrepFace face, PlaneSurface plane, float tolerance, List<string> errors)
    {
        var normal = Vector3.Normalize(plane.Normal);
        if (!float.IsFinite(normal.LengthSquared()))
        {
            errors.Add($"Face {f}: the plane normal is not a usable direction.");
            return null;
        }

        bool planar = true;
        foreach (var loop in face.Loops)
        foreach (var oriented in loop.Edges)
        {
            planar &= OnPlane(f, "a loop vertex", oriented.StartVertex.Position, plane.Origin, normal, tolerance, errors);
            if (oriented.Edge.Curve is not CircleCurve circle) continue;
            // A circle whose centre is on the plane and whose axis is normal to it lies on the plane whole.
            planar &= OnPlane(f, "a circle centre", circle.Center, plane.Origin, normal, tolerance, errors);
            if (!(MathF.Abs(Vector3.Dot(Vector3.Normalize(circle.Axis), normal)) >= ParallelCosLimit))
            {
                errors.Add($"Face {f}: a circle of a loop does not lie on the face plane, its axis is not normal to it.");
                planar = false;
            }
        }
        if (!planar) return null;

        var outward = face.SameSense ? normal : -normal;
        var (u, v) = Basis.Orthonormal(outward);
        double area = 0;
        foreach (var loop in face.Loops)
        {
            double loopArea = SignedLoopArea(loop, plane.Origin, u, v);
            if (double.IsNaN(loopArea))
            {
                errors.Add($"Face {f}: a loop carries a circular arc, which this validation cannot measure.");
                return null;
            }
            area += loopArea;
        }

        // Enclosing loops counterclockwise, holes clockwise: their areas add up to the area of the face.
        // A negative or null total means the loops disagree with the normal about which side is material.
        if (!(area > 0))
        {
            errors.Add(FormattableString.Invariant(
                $"Face {f}: the loops do not run counterclockwise around the face normal (net area {area} m2)."));
            return null;
        }
        return (double)Vector3.Dot(plane.Origin, outward) * area / 3.0;
    }

    private static double? CylindricalFaceVolume(int f, BrepFace face, CylinderSurface cylinder, float tolerance, List<string> errors)
    {
        var axis = Vector3.Normalize(cylinder.Axis);
        if (!float.IsFinite(axis.LengthSquared()))
        {
            errors.Add($"Face {f}: the cylinder axis is not a usable direction.");
            return null;
        }
        if (face.Loops.Count != 2)
        {
            errors.Add($"Face {f}: a cylindrical face is measured as a full cylinder between two rims, found {face.Loops.Count} loops.");
            return null;
        }

        var rims = new (float Axial, int Winding)[2];
        for (int l = 0; l < 2; l++)
        {
            var edges = face.Loops[l].Edges;
            if (edges.Count != 1 || edges[0].Edge.Curve is not CircleCurve circle
                || !ReferenceEquals(edges[0].Edge.Start, edges[0].Edge.End))
            {
                errors.Add($"Face {f}: a rim of the cylindrical face is not a single closed circle.");
                return null;
            }
            var offset = circle.Center - cylinder.Origin;
            float axial = Vector3.Dot(offset, axis);
            var circleAxis = Vector3.Normalize(circle.Axis);
            if (!((offset - axis * axial).Length() <= tolerance
                  && MathF.Abs(circle.Radius - cylinder.Radius) <= tolerance
                  && MathF.Abs(Vector3.Dot(circleAxis, axis)) >= ParallelCosLimit))
            {
                errors.Add($"Face {f}: a rim of the cylindrical face is not a circle of that very cylinder.");
                return null;
            }
            // Counterclockwise around the cylinder axis counts as +1, whatever the circle's own axis.
            int winding = (Vector3.Dot(circleAxis, axis) > 0 ? 1 : -1) * (edges[0].SameSense ? 1 : -1);
            rims[l] = (axial, winding);
        }

        // With the face normal pointing away from the axis, the boundary runs counterclockwise at the
        // lower rim and clockwise at the upper one; a hole (SameSense = false) reverses both.
        var lower = rims[0].Axial <= rims[1].Axial ? rims[0] : rims[1];
        var upper = rims[0].Axial <= rims[1].Axial ? rims[1] : rims[0];
        int expected = face.SameSense ? 1 : -1;
        if (lower.Winding != expected || upper.Winding != -expected)
        {
            errors.Add($"Face {f}: the rims of the cylindrical face do not run counterclockwise around its outward normal.");
            return null;
        }

        double height = (double)upper.Axial - lower.Axial;
        return expected * 2.0 * Math.PI * (double)cylinder.Radius * cylinder.Radius * height / 3.0;
    }

    private static double? Unsupported(int f, BrepSurface surface, List<string> errors)
    {
        errors.Add($"Face {f}: {surface.GetType().Name} is not measured by this validation.");
        return null;
    }

    private static bool OnPlane(int f, string what, Vector3 point, Vector3 origin, Vector3 normal, float tolerance, List<string> errors)
    {
        float distance = Vector3.Dot(point - origin, normal);
        // Positive form on purpose: a comparison against NaN is false, so "if (invalid)" would pass it.
        if (MathF.Abs(distance) <= tolerance) return true;
        errors.Add(FormattableString.Invariant($"Face {f}: {what} does not lie on the face plane, {distance} m off."));
        return false;
    }

    /// <summary>
    /// Signed area of a planar loop measured in the basis (u, v), positive when the loop runs
    /// counterclockwise around u x v. This is Green's theorem, so the loops of a face simply add up:
    /// an enclosing loop contributes its area and a hole subtracts its own. Straight segments and
    /// closed circles are supported, which is everything the builders produce; an arc (a circle edge
    /// whose endpoints differ) returns NaN.
    /// </summary>
    internal static double SignedLoopArea(BrepLoop loop, Vector3 origin, Vector3 u, Vector3 v)
    {
        var normal = Vector3.Cross(u, v);
        double area = 0;
        foreach (var oriented in loop.Edges)
        {
            if (oriented.Edge.Curve is CircleCurve circle)
            {
                if (!ReferenceEquals(oriented.Edge.Start, oriented.Edge.End)) return double.NaN;
                // A circle traversed counterclockwise around its own axis reads as counterclockwise in
                // (u, v) when that axis and u x v agree, and as clockwise when they oppose.
                double sign = Vector3.Dot(Vector3.Normalize(circle.Axis), normal) > 0 ? 1 : -1;
                area += (oriented.SameSense ? sign : -sign) * Math.PI * (double)circle.Radius * circle.Radius;
                continue;
            }

            var start = oriented.StartVertex.Position - origin;
            var end = oriented.EndVertex.Position - origin;
            area += 0.5 * ((double)Vector3.Dot(start, u) * Vector3.Dot(end, v)
                           - (double)Vector3.Dot(end, u) * Vector3.Dot(start, v));
        }
        return area;
    }
}
