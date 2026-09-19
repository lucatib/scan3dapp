namespace Scanner.Brep.Model;

/// <summary>Minimal topological checks for a closed manifold solid.</summary>
public static class BrepValidator
{
    public static IReadOnlyList<string> Validate(BrepSolid solid)
    {
        var errors = new List<string>();
        var uses = new Dictionary<BrepEdge, (int Forward, int Backward)>();
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
            for (int i = 0; i < loop.Edges.Count; i++)
            {
                var current = loop.Edges[i];
                var next = loop.Edges[(i + 1) % loop.Edges.Count];
                if (current.EndVertex != next.StartVertex)
                    errors.Add($"Face {f}: loop not closed after edge {i}.");

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
}
