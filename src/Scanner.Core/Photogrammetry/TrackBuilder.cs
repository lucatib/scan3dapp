namespace Scanner.Core.Photogrammetry;

/// <summary>
/// Chains pairwise feature matches into tracks: the features of every photo that show the same scene point. Matches
/// are only ever found between pairs of photos, but a point seen in many photos is worth far more to bundle adjustment
/// than the same point counted pair by pair, and a long track exposes wrong matches that no single pair can.
/// </summary>
public static class TrackBuilder
{
    /// <summary>
    /// Every (photo, feature) is a node and every match joins two nodes; each connected set is a candidate track. A set
    /// holding two different features of one photo is inconsistent: a point shows up once per photo, so somewhere along
    /// the chain a match is wrong, and since nothing tells which one, the whole set is dropped. Sets seen in fewer than
    /// <paramref name="minViews"/> photos are dropped too.
    /// </summary>
    /// <param name="features">The features of each photo; <see cref="FeatureMatch"/> indices point into these lists.</param>
    /// <param name="minViews">Fewest photos a track must be seen in; at 1, unmatched features come back as tracks of
    /// one observation.</param>
    /// <returns>One array per track with the observations ordered by photo index. Tracks are ordered by their first
    /// (photo, feature) node, so the output does not depend on the order or direction of the matches.</returns>
    public static List<Observation[]> Build(IReadOnlyList<IReadOnlyList<Feature>> features, IEnumerable<FeatureMatch> matches,
        int minViews = 2)
    {
        ArgumentNullException.ThrowIfNull(features);
        ArgumentNullException.ThrowIfNull(matches);

        // Nodes are numbered photo by photo, so walking them in order visits each set's members in photo order.
        var first = new int[features.Count + 1];
        for (int view = 0; view < features.Count; view++) first[view + 1] = first[view] + features[view].Count;
        int nodeCount = first[features.Count];

        int Node(int view, int feature)
        {
            if ((uint)view >= (uint)features.Count || (uint)feature >= (uint)features[view].Count)
                throw new ArgumentOutOfRangeException(nameof(matches), $"A match refers to feature {feature} of view {view}, which does not exist.");
            return first[view] + feature;
        }

        var sets = new DisjointSets(nodeCount);
        foreach (var match in matches) sets.Union(Node(match.ViewA, match.FeatureA), Node(match.ViewB, match.FeatureB));

        // Pass 1: size and consistency of every set. A repeated photo shows up as the same photo twice in a row.
        var root = new int[nodeCount];
        var size = new int[nodeCount];
        var lastView = new int[nodeCount];
        var inconsistent = new bool[nodeCount];
        Array.Fill(lastView, -1);
        for (int view = 0, node = 0; view < features.Count; view++)
        for (int feature = 0; feature < features[view].Count; feature++, node++)
        {
            int r = root[node] = sets.Find(node);
            if (lastView[r] == view) inconsistent[r] = true;
            lastView[r] = view;
            size[r]++;
        }

        // Pass 2: the surviving sets in order of their first node, observations in photo order.
        var trackOf = new int[nodeCount];
        var filled = new int[nodeCount];
        var tracks = new List<Observation[]>();
        Array.Fill(trackOf, -1);
        for (int view = 0, node = 0; view < features.Count; view++)
        for (int feature = 0; feature < features[view].Count; feature++, node++)
        {
            int r = root[node];
            if (inconsistent[r] || size[r] < minViews) continue;
            if (trackOf[r] < 0)
            {
                trackOf[r] = tracks.Count;
                tracks.Add(new Observation[size[r]]);
            }
            var f = features[view][feature];
            tracks[trackOf[r]][filled[r]++] = new Observation(view, f.X, f.Y);
        }
        return tracks;
    }

    /// <summary>Union-find with path halving and union by size: near-constant time per operation.</summary>
    private sealed class DisjointSets
    {
        private readonly int[] _parent;
        private readonly int[] _size;

        public DisjointSets(int count)
        {
            _parent = new int[count];
            _size = new int[count];
            for (int i = 0; i < count; i++)
            {
                _parent[i] = i;
                _size[i] = 1;
            }
        }

        public int Find(int node)
        {
            while (_parent[node] != node)
            {
                _parent[node] = _parent[_parent[node]];
                node = _parent[node];
            }
            return node;
        }

        public void Union(int a, int b)
        {
            a = Find(a);
            b = Find(b);
            if (a == b) return;
            if (_size[a] < _size[b]) (a, b) = (b, a);
            _parent[b] = a;
            _size[a] += _size[b];
        }
    }
}
