using System.Globalization;
using System.Numerics;
using System.Text;

namespace Scanner.Core.Turntable;

/// <summary>
/// The printed turntable mat: a ring of <see cref="DotCount"/> dark dots, each either filled or a ring (with a white
/// hole), read around the circle as a code in which any <see cref="WindowLength"/> consecutive dots, read either way
/// round, appear once. A photo that shows that many neighbours in a row therefore knows which dots they are, and
/// so the mat's angle; the dots' printed positions give the pose and the scale.
/// The mat frame is the pipeline's world: the mat is the plane y = 0 (+Y up), centred on the origin; dot 0 sits on +X
/// and the numbering runs towards +Z.
/// </summary>
public static class TurntableMat
{
    public const int DotCount = 36;
    public const int WindowLength = 7;

    /// <summary>The motorised platter's radius, metres (Ø 240 mm).</summary>
    public const float PlatterRadius = 0.120f;

    /// <summary>Ring radius, metres: the dots run 8 mm in from the platter's edge, leaving the piece the most room.</summary>
    public const float RingRadius = PlatterRadius - 0.008f;

    public const float DotRadius = 0.0045f;

    /// <summary>The hole of a ring dot. Half the dot: the dark ring stays as wide as the hole, so neither closes up
    /// when the photo is soft.</summary>
    public const float HoleRadius = 0.00225f;

    /// <summary>The piece has to stand inside this radius to leave the ring visible.</summary>
    public const float FreeRadius = RingRadius - 0.012f;

    /// <summary>True for a ring dot (with a hole), false for a filled one.</summary>
    public static IReadOnlyList<bool> Code { get; } = FindCode();

    public static Vector3 DotPosition(int index)
    {
        float angle = 2 * MathF.PI * index / DotCount;
        return new Vector3(RingRadius * MathF.Cos(angle), 0, RingRadius * MathF.Sin(angle));
    }

    /// <summary>
    /// Where <paramref name="window"/> (consecutive codes as photographed) sits on the ring: the index of its first dot
    /// and +1 if it runs with the numbering, -1 against it. Null when it is not on the mat.
    /// </summary>
    public static (int Start, int Direction)? Locate(IReadOnlyList<bool> window)
    {
        if (window.Count < WindowLength) return null;
        (int, int)? found = null;
        for (int direction = 1; direction >= -1; direction -= 2)
        for (int start = 0; start < DotCount; start++)
        {
            bool match = true;
            for (int i = 0; i < window.Count && match; i++)
                match = Code[Mod(start + direction * i)] == window[i];
            if (!match) continue;
            if (found is not null) return null; // ambiguous (only possible for a window shorter than the code allows)
            found = (start, direction);
        }
        return found;
    }

    /// <summary>The mat as an SVG at true size (millimetres): print at 100 %, no "fit to page".</summary>
    public static string Svg()
    {
        var c = CultureInfo.InvariantCulture;
        const float page = 2 * PlatterRadius * 1000; // mm, square: the platter's size
        float centre = page / 2;
        var svg = new StringBuilder();
        svg.Append(c, $"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{page}mm\" height=\"{page}mm\" viewBox=\"0 0 {page} {page}\">\n");
        svg.Append("<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n");
        for (int i = 0; i < DotCount; i++)
        {
            var p = DotPosition(i) * 1000;
            // Paper y runs down: +Z of the mat is drawn downwards, so seen from above the numbering runs clockwise.
            float x = centre + p.X, y = centre + p.Z;
            svg.Append(c, $"<circle cx=\"{x:F3}\" cy=\"{y:F3}\" r=\"{DotRadius * 1000:F3}\" fill=\"black\"/>\n");
            if (Code[i]) svg.Append(c, $"<circle cx=\"{x:F3}\" cy=\"{y:F3}\" r=\"{HoleRadius * 1000:F3}\" fill=\"white\"/>\n");
        }
        svg.Append(c, $"<circle cx=\"{centre}\" cy=\"{centre}\" r=\"{FreeRadius * 1000:F1}\" fill=\"none\" stroke=\"#bbb\" stroke-width=\"0.3\" stroke-dasharray=\"2 2\"/>\n");
        svg.Append(c, $"<text x=\"{centre}\" y=\"{page - 4}\" font-family=\"sans-serif\" font-size=\"3.5\" text-anchor=\"middle\" fill=\"#888\">"
                      + $"Scan3D turntable mat · print at 100 % · ring Ø {2 * RingRadius * 1000:F0} mm · piece inside the dashed circle</text>\n");
        svg.Append("</svg>\n");
        return svg.ToString();
    }

    /// <summary>
    /// A fixed code with every window unique both ways round, so no window is a palindrome either. 72 windows out of
    /// 128 patterns: a random code almost never qualifies, so it is built dot by dot, backtracking (deterministic).
    /// </summary>
    private static bool[] FindCode()
    {
        var code = new bool[DotCount];
        var seen = new HashSet<int>();
        if (!Extend(code, 0, seen)) throw new InvalidOperationException("No mat code found.");
        return code;
    }

    private static bool Extend(bool[] code, int length, HashSet<int> seen)
    {
        if (length == DotCount)
        {
            // The windows that wrap around the end of the ring.
            var added = new List<int>();
            bool ok = true;
            for (int start = DotCount - WindowLength + 1; start < DotCount && ok; start++)
                ok = TryAddWindow(code, start, seen, added);
            if (!ok) foreach (int key in added) seen.Remove(key);
            return ok;
        }
        foreach (bool bit in (bool[])[length % 3 == 0, length % 3 != 0])
        {
            code[length] = bit;
            var added = new List<int>();
            if (length + 1 >= WindowLength && !TryAddWindow(code, length + 1 - WindowLength, seen, added))
            {
                foreach (int key in added) seen.Remove(key);
                continue;
            }
            if (Extend(code, length + 1, seen)) return true;
            foreach (int key in added) seen.Remove(key);
        }
        return false;
    }

    /// <summary>Adds the window starting at <paramref name="start"/> read forwards and backwards; false (with what it
    /// added listed in <paramref name="added"/>) when either reading is taken.</summary>
    private static bool TryAddWindow(bool[] code, int start, HashSet<int> seen, List<int> added)
    {
        int forward = 0, backward = 0;
        for (int i = 0; i < WindowLength; i++)
        {
            forward = forward * 2 + (code[Mod(start + i)] ? 1 : 0);
            backward = backward * 2 + (code[Mod(start + WindowLength - 1 - i)] ? 1 : 0);
        }
        if (forward == backward || !seen.Add(forward)) return false;
        added.Add(forward);
        if (!seen.Add(backward)) return false;
        added.Add(backward);
        return true;
    }

    private static int Mod(int i) => ((i % DotCount) + DotCount) % DotCount;
}
