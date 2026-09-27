namespace Scanner.Core.Photogrammetry;

/// <summary>
/// Chains pairwise feature matches into tracks: the features of every photo that show the same scene point. Matches
/// are only ever found between pairs of photos, but a point seen in many photos is worth far more to bundle adjustment
/// than the same point counted pair by pair, and a long track exposes wrong matches that no single pair can.
/// </summary>
public static class TrackBuilder
{
    /// <summary>
    /// Every (photo, feature) is a node and every match joins two nodes into one set, strongest match first. A match
    /// whose two sets already hold features of a common photo is left out: a point shows up once per photo, so one of
    /// the matches that led there is wrong, and the weaker one is the likelier culprit. A wrong match then costs one
    /// link instead of every track it touches; on real photos, where a few chance matches among thousands chain most
    /// tracks together, that keeps more than half the observations. Sets seen in fewer than
    /// <paramref name="minViews"/> photos are dropped.
    /// </summary>
    /// <param name="features">The features of each photo; <see cref="FeatureMatch"/> indices point into these lists.</param>
    /// <param name="minViews">Fewest photos a track must be seen in; at 1, unmatched features come back as tracks of
    /// one observation.</param>
    /// <returns>One array per track with the observations ordered by photo index. Tracks are ordered by their first
    /// (photo, feature) node, and equal scores are taken in node order, so the output does not depend on the order or
    /// direction of the matches.</returns>
    public static List<Observation[]> Build(IReadOnlyList<IReadOnlyList<Feature>> features, IEnumerable<FeatureMatch> matches,
        int minViews = 2)
    {
        ArgumentNullException.ThrowIfNull(features);
        ArgumentNullException.ThrowIfNull(matches);

        // Nodes are numbered photo by photo, so walking them in order visits each set's members in photo order.
        var first = new int[features.Count + 1];
        for (int view = 0; view < features.Count; view++) first[view + 1] = first[view] + features[view].Count;
        int nodeCount = first[features.Count];
        var viewOf = new int[nodeCount];
        for (int view = 0; view < features.Count; view++) Array.Fill(viewOf, view, first[view], features[view].Count);

        int Node(int view, int feature)
        {
            if ((uint)view >= (uint)features.Count || (uint)feature >= (uint)features[view].Count)
                throw new ArgumentOutOfRangeException(nameof(matches), $"A match refers to feature {feature} of view {view}, which does not exist.");
            return first[view] + feature;
        }

        var links = new List<(float Score, int A, int B)>();
        foreach (var match in matches)
        {
            int a = Node(match.ViewA, match.FeatureA), b = Node(match.ViewB, match.FeatureB);
            links.Add(a <= b ? (match.Score, a, b) : (match.Score, b, a));
        }
        links.Sort((x, y) => x.Score != y.Score ? y.Score.CompareTo(x.Score) : x.A != y.A ? x.A.CompareTo(y.A) : x.B.CompareTo(y.B));

        var sets = new DisjointSets(viewOf);
        foreach (var (_, a, b) in links) sets.TryUnion(a, b);

        // The surviving sets in order of their first node, observations in photo order.
        var trackOf = new int[nodeCount];
        var filled = new int[nodeCount];
        var tracks = new List<Observation[]>();
        Array.Fill(trackOf, -1);
        for (int view = 0, node = 0; view < features.Count; view++)
        for (int feature = 0; feature < features[view].Count; feature++, node++)
        {
            int r = sets.Find(node);
            int size = sets.Size(r);
            if (size < minViews) continue;
            if (trackOf[r] < 0)
            {
                trackOf[r] = tracks.Count;
                tracks.Add(new Observation[size]);
            }
            var f = features[view][feature];
            tracks[trackOf[r]][filled[r]++] = new Observation(view, f.X, f.Y);
        }
        return tracks;
    }

    /// <summary>Union-find with path halving and union by size, which also keeps the photos each set is seen in so
    /// that a union joining two features of one photo can be refused: near-constant time per operation, and each
    /// photo index is copied only when its set is the smaller one of a union, O(n log n) in all.</summary>
    private sealed class DisjointSets
    {
        private readonly int[] _parent;
        private readonly int[] _size;
        private readonly int[] _viewOf;
        // The photos of a set of two or more nodes, kept at its root; a single node is seen in its own photo only.
        private readonly HashSet<int>?[] _views;

        public DisjointSets(int[] viewOf)
        {
            int count = viewOf.Length;
            _viewOf = viewOf;
            _parent = new int[count];
            _size = new int[count];
            _views = new HashSet<int>?[count];
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

        public int Size(int root) => _size[root];

        /// <summary>Joins the sets of the two nodes unless they already hold features of a common photo.</summary>
        public void TryUnion(int a, int b)
        {
            a = Find(a);
            b = Find(b);
            if (a == b) return;
            if (_size[a] < _size[b]) (a, b) = (b, a);
            var large = _views[a] ?? [_viewOf[a]];
            if (_views[b] is { } small)
            {
                foreach (int view in small)
                    if (large.Contains(view)) return;
                large.UnionWith(small);
                _views[b] = null;
            }
            else if (!large.Add(_viewOf[b])) return;
            _views[a] = large;
            _parent[b] = a;
            _size[a] += _size[b];
        }
    }
}
