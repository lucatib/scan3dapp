using System.Collections;
using System.Globalization;
using System.Numerics;
using System.Text;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Brep.Step;

/// <summary>
/// Writes a <see cref="BrepSolid"/> as a STEP AP214 file (ISO 10303-21) in millimeters.
/// </summary>
/// <remarks>
/// <para>
/// Representation choices that a CAD kernel may refuse to sew, decided here once and pinned by tests:
/// </para>
/// <para>
/// 1. Every circle is emitted as a SINGLE CLOSED EDGE: one EDGE_CURVE whose start and end vertex are
/// the same VERTEX_POINT, carried by a single ORIENTED_EDGE in its EDGE_LOOP. The alternative is to
/// split each circle into two half-circle edges over two opposite vertices. The closed edge is kept
/// because it is what the model carries (<c>TubeBuilder</c> creates one shared edge per circle) and
/// therefore the only topology <see cref="BrepValidator"/> has verified: splitting would emit a
/// topology no check in this codebase has ever seen. It is also the shape mainstream exporters
/// produce for a full circle, so importers are well tested against it. IF AN IMPORT FAILS, THIS IS
/// THE FIRST SUSPECT: the remedy is to emit, per circle, two EDGE_CURVEs over the vertices at
/// <c>centre ± refDirection * radius</c> sharing one CIRCLE, and to expand each ORIENTED_EDGE into
/// the matching pair (forward: first then second; reversed: second then first, both reversed).
/// </para>
/// <para>
/// 2. Cap faces keep their outer and inner boundaries as SEPARATE LOOPS, emitted as one
/// FACE_OUTER_BOUND plus one FACE_BOUND per hole. This is the standard AP214 encoding of a face with
/// a hole; no bridge edge is introduced. A full cylindrical face has two boundaries of which neither
/// encloses the other; its first rim is designated FACE_OUTER_BOUND anyway, to match what mainstream
/// exporters emit - see <see cref="OuterBoundIndex"/>.
/// </para>
/// </remarks>
public sealed class StepWriter
{
    private const double MetersToMillimeters = 1000.0;

    /// <summary>
    /// Smallest radius, in meters, that may be emitted: 1 um, i.e. 0.001 mm.
    /// <para>
    /// The builders guarantee a finite, strictly positive radius but no magnitude floor, so a 1e-30 m
    /// radius is "valid" all the way down to here, where <see cref="Real"/> rounds it to 0. and produces
    /// a degenerate CIRCLE in a file that is otherwise perfect. The floor is owned by the writer rather
    /// than by the caller because it is a property of the emitted format: it sits three decades above
    /// the 5e-7 mm resolution of the emitted numbers, and three decades below the voxel size of the
    /// coarsest processing profile, so it can only ever reject geometry that was already meaningless.
    /// </para>
    /// </summary>
    private const float MinimumRadiusMeters = 1e-6f;

    private readonly StringBuilder _data = new();
    private readonly Dictionary<object, int> _ids = new(ReferenceEqualityComparer.Instance);
    private int _nextId = 1;

    private StepWriter()
    {
    }

    /// <summary>Writes <paramref name="solid"/> as a complete ISO 10303-21 AP214 file.</summary>
    /// <exception cref="ArgumentException">
    /// <paramref name="solid"/> is not a valid closed B-Rep (see <see cref="BrepValidator.Validate"/>),
    /// or two of its vertices would be emitted at the same coordinates.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">A radius of <paramref name="solid"/> is below the
    /// smallest magnitude the file format can carry.</exception>
    public static string Write(BrepSolid solid, string productName, DateTime timestampUtc)
    {
        ArgumentNullException.ThrowIfNull(solid);
        ArgumentNullException.ThrowIfNull(productName);

        // A file that is syntactically perfect and geometrically wrong is the worst failure mode here,
        // because it imports and only then misbehaves. Emission is gated on the full validator, which
        // checks planarity and the sign of the volume as well as the topology.
        var errors = BrepValidator.Validate(solid);
        if (errors.Count > 0)
            throw new ArgumentException("The solid is not a valid closed B-Rep: " + string.Join(" ", errors), nameof(solid));
        CheckEmittableMagnitudes(solid);

        return new StepWriter().WriteFile(solid, productName, timestampUtc);
    }

    /// <summary>Rejects magnitudes that survive validation but cannot be expressed at the resolution of the file.</summary>
    private static void CheckEmittableMagnitudes(BrepSolid solid)
    {
        foreach (var face in solid.Faces)
            if (face.Surface is CylinderSurface cylinder && !(cylinder.Radius >= MinimumRadiusMeters))
                throw new ArgumentOutOfRangeException(nameof(solid), cylinder.Radius,
                    $"A cylindrical surface radius below {MinimumRadiusMeters} m cannot be emitted.");

        foreach (var edge in solid.DistinctEdges())
            if (edge.Curve is CircleCurve circle && !(circle.Radius >= MinimumRadiusMeters))
                throw new ArgumentOutOfRangeException(nameof(solid), circle.Radius,
                    $"A circle radius below {MinimumRadiusMeters} m cannot be emitted.");

        // The mirror image of the radius floor: an extent below the emitted resolution keeps every radius
        // healthy and collapses distinct vertices onto one point instead.
        var emitted = new HashSet<string>(StringComparer.Ordinal);
        foreach (var vertex in solid.DistinctVertices())
            if (!emitted.Add(Coordinates(vertex.Position)))
                throw new ArgumentException(
                    $"Two distinct vertices are emitted at the same coordinates ({Coordinates(vertex.Position)}).", nameof(solid));
    }

    private string WriteFile(BrepSolid solid, string productName, DateTime timestampUtc)
    {
        string name = Escape(productName);

        int app = Add("APPLICATION_CONTEXT('automotive design')");
        Add($"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})");
        int productContext = Add($"PRODUCT_CONTEXT('',#{app},'mechanical')");
        int product = Add($"PRODUCT('{name}','{name}','',(#{productContext}))");
        Add($"PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))");
        int formation = Add($"PRODUCT_DEFINITION_FORMATION('','',#{product})");
        int definitionContext = Add($"PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')");
        int definition = Add($"PRODUCT_DEFINITION('design','',#{formation},#{definitionContext})");
        int shape = Add($"PRODUCT_DEFINITION_SHAPE('','',#{definition})");

        int length = Add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        int angle = Add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        int solidAngle = Add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
        // 1.E-03 is a fixed header literal, not a formatted coordinate: it is the only exponent in the file.
        int uncertainty = Add($"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-03),#{length},'distance_accuracy_value','confusion accuracy')");
        int context = Add("(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
                          + $"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty}))"
                          + $"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solidAngle}))"
                          + "REPRESENTATION_CONTEXT('Context3D','3D Context'))");

        int brep = WriteSolid(solid);
        int origin = WriteAxis(Vector3.Zero, Vector3.UnitZ, Vector3.UnitX);
        int representation = Add($"ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',(#{origin},#{brep}),#{context})");
        Add($"SHAPE_DEFINITION_REPRESENTATION(#{shape},#{representation})");

        var file = new StringBuilder();
        file.Append("ISO-10303-21;\n");
        file.Append("HEADER;\n");
        file.Append("FILE_DESCRIPTION(('Scan3D model'),'2;1');\n");
        file.Append($"FILE_NAME('{name}.stp','{timestampUtc.ToString("yyyy-MM-ddTHH:mm:ss", CultureInfo.InvariantCulture)}',('Scan3D'),(''),'Scan3D','Scan3D','');\n");
        file.Append("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
        file.Append("ENDSEC;\n");
        file.Append("DATA;\n");
        file.Append(_data);
        file.Append("ENDSEC;\n");
        file.Append("END-ISO-10303-21;\n");
        return file.ToString();
    }

    private int WriteSolid(BrepSolid solid)
    {
        var faces = solid.Faces.Select(WriteFace).ToList();
        int shell = Add($"CLOSED_SHELL('',({Refs(faces)}))");
        return Add($"MANIFOLD_SOLID_BREP('',#{shell})");
    }

    private int WriteFace(BrepFace face)
    {
        int surface = WriteSurface(face.Surface);
        int outer = OuterBoundIndex(face);
        var bounds = new List<int>();
        for (int i = 0; i < face.Loops.Count; i++)
        {
            int loop = WriteLoop(face.Loops[i]);
            string kind = i == outer ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            bounds.Add(Add($"{kind}('',#{loop},.T.)"));
        }
        // SameSense comes from the model: it is the only signal that a hole's surface normal is inverted,
        // and hardcoding .T. here would leave every bore pointing into the material.
        return Add($"ADVANCED_FACE('',({Refs(bounds)}),#{surface},{Bool(face.SameSense)})");
    }

    /// <summary>
    /// Index of the loop to emit as FACE_OUTER_BOUND. For a planar face this is the enclosing loop,
    /// selected by area rather than by position, so that a reordering of <see cref="BrepFace.Loops"/>
    /// cannot silently swap FACE_OUTER_BOUND and FACE_BOUND and turn the part inside out at the hole.
    ///
    /// On a full cylindrical face neither rim encloses the other, so ISO 10303-42 would permit emitting
    /// both as FACE_BOUND and designating no outer bound ("at most one", not "exactly one"). We designate
    /// the first rim anyway, because that is what mainstream exporters emit - OCCT picks one wire via
    /// BRepTools::OuterWire and writes FACE_OUTER_BOUND for it even on a full cylinder - and a reader that
    /// classifies bounds from the flag rather than from geometry would otherwise see a face with no outer
    /// boundary at all. The choice between the two rims is arbitrary by construction, not by accident.
    /// </summary>
    private static int OuterBoundIndex(BrepFace face)
    {
        if (face.Surface is not PlaneSurface plane) return face.Loops.Count > 0 ? 0 : -1;

        var normal = Vector3.Normalize(face.SameSense ? plane.Normal : -plane.Normal);
        var (u, v) = Basis.Orthonormal(normal);
        int outer = 0;
        double largest = double.NegativeInfinity;
        for (int i = 0; i < face.Loops.Count; i++)
        {
            double area = Math.Abs(BrepValidator.SignedLoopArea(face.Loops[i], plane.Origin, u, v));
            if (area > largest)
            {
                largest = area;
                outer = i;
            }
        }
        return outer;
    }

    private int WriteLoop(BrepLoop loop)
    {
        var oriented = loop.Edges
            .Select(e => Add($"ORIENTED_EDGE('',*,*,#{WriteEdge(e.Edge)},{Bool(e.SameSense)})"))
            .ToList();
        return Add($"EDGE_LOOP('',({Refs(oriented)}))");
    }

    private int WriteEdge(BrepEdge edge)
    {
        if (_ids.TryGetValue(edge, out int id)) return id;
        int start = WriteVertex(edge.Start);
        int end = WriteVertex(edge.End);
        int curve = WriteCurve(edge.Curve);
        id = Add($"EDGE_CURVE('',#{start},#{end},#{curve},.T.)");
        _ids[edge] = id;
        return id;
    }

    private int WriteVertex(BrepVertex vertex)
    {
        if (_ids.TryGetValue(vertex, out int id)) return id;
        id = Add($"VERTEX_POINT('',#{WritePoint(vertex.Position)})");
        _ids[vertex] = id;
        return id;
    }

    private int WriteCurve(BrepCurve curve) => curve switch
    {
        LineCurve line => Add($"LINE('',#{WritePoint(line.Origin)},#{Add($"VECTOR('',#{WriteDirection(line.Direction)},1.0)")})"),
        CircleCurve circle => Add($"CIRCLE('',#{WriteAxis(circle.Center, circle.Axis, circle.RefDirection)},{Length(circle.Radius)})"),
        _ => throw new NotSupportedException($"Unsupported curve: {curve.GetType().Name}"),
    };

    private int WriteSurface(BrepSurface surface) => surface switch
    {
        PlaneSurface plane => Add($"PLANE('',#{WriteAxis(plane.Origin, plane.Normal, plane.RefDirection)})"),
        CylinderSurface cylinder => Add($"CYLINDRICAL_SURFACE('',#{WriteAxis(cylinder.Origin, cylinder.Axis, cylinder.RefDirection)},{Length(cylinder.Radius)})"),
        _ => throw new NotSupportedException($"Unsupported surface: {surface.GetType().Name}"),
    };

    private int WriteAxis(Vector3 location, Vector3 axis, Vector3 refDirection) =>
        Add($"AXIS2_PLACEMENT_3D('',#{WritePoint(location)},#{WriteDirection(axis)},#{WriteDirection(refDirection)})");

    private int WritePoint(Vector3 p) => Add($"CARTESIAN_POINT('',({Coordinates(p)}))");

    private int WriteDirection(Vector3 d)
    {
        var n = Vector3.Normalize(d);
        return Add($"DIRECTION('',({Real(n.X)},{Real(n.Y)},{Real(n.Z)}))");
    }

    private int Add(string entity)
    {
        int id = _nextId++;
        _data.Append('#').Append(id).Append('=').Append(entity).Append(";\n");
        return id;
    }

    private static string Refs(IEnumerable<int> ids) => string.Join(",", ids.Select(i => $"#{i}"));

    private static string Bool(bool value) => value ? ".T." : ".F.";

    private static string Coordinates(Vector3 p) => $"{Length(p.X)},{Length(p.Y)},{Length(p.Z)}";

    private static string Length(float meters) => Real(meters * MetersToMillimeters);

    // Rounding to 1e-6 (nm for lengths) to avoid float noise like 19.9999995. Negative zero is folded
    // onto zero so that a mirrored direction does not read as "-0.0". Never exponent notation.
    private static string Real(double value)
    {
        // Last line of defence, at the one point every number in the file passes through. The validator
        // checks plane normals, cylinder axes, vertex positions and circle centres, but it never reads a
        // RefDirection or a LineCurve.Direction - a non-finite one of those would otherwise emit
        // DIRECTION((NaN,NaN,NaN)), which is not legal STEP. Positive form: any comparison with NaN is
        // false, so "if (invalid) throw" would let NaN through.
        if (!double.IsFinite(value))
            throw new ArgumentOutOfRangeException(nameof(value), value, "A non-finite number cannot be written to a STEP file.");

        double rounded = Math.Round(value, 6);
        if (rounded == 0) rounded = 0d;
        return rounded.ToString("0.0#####", CultureInfo.InvariantCulture);
    }

    // In an ISO 10303-21 string a quote is doubled and a backslash, which introduces the control
    // directives, is doubled too; a lone backslash would make the literal ambiguous. Neither
    // replacement can produce the other's character, so the order between them does not matter.
    //
    // Anything outside printable ASCII is REJECTED rather than passed through. A newline in the product
    // name splits a STEP line in two and structurally breaks the file, and a non-ASCII character is not
    // conformant Part 21 (which requires the \X2\....\X0\ encoding we do not implement). Both failures
    // produce a corrupt file rather than an exception, and the name reaches here from user-entered text,
    // so rejecting loudly at the boundary is the only safe default.
    private static string Escape(string text)
    {
        foreach (char c in text)
            if (c < ' ' || c > '~')
                throw new ArgumentException(
                    $"The product name contains a character that cannot be written to a STEP file: U+{(int)c:X4}.",
                    nameof(text));

        return text.Replace("\\", "\\\\", StringComparison.Ordinal).Replace("'", "''", StringComparison.Ordinal);
    }
}
