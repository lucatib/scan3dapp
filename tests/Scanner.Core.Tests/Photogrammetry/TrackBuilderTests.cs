using Scanner.Core.Photogrammetry;

namespace Scanner.Core.Tests.Photogrammetry;

public class TrackBuilderTests
{
    // Feature i of view v sits at (100 v + i, 1000 + 100 v + i), so an observation shows which feature it came from.
    private static List<IReadOnlyList<Feature>> Features(params int[] counts) =>
        counts.Select((count, view) => (IReadOnlyList<Feature>)Enumerable.Range(0, count)
            .Select(i => new Feature(100 * view + i, 1000 + 100 * view + i, 1f)).ToList()).ToList();

    private static FeatureMatch Match(int viewA, int featureA, int viewB, int featureB, float score = 0.9f) =>
        new(viewA, featureA, viewB, featureB, score);

    private static Observation Seen(int view, int feature) => new(view, 100 * view + feature, 1000 + 100 * view + feature);

    [Fact]
    public void Matches_chain_into_one_track_across_three_views()
    {
        var features = Features(4, 4, 4);

        // The B-C match is given as C -> B: the direction of a match must not matter.
        var tracks = TrackBuilder.Build(features, [Match(0, 1, 1, 2), Match(2, 3, 1, 2)]);

        var track = Assert.Single(tracks);
        Assert.Equal(new[] { Seen(0, 1), Seen(1, 2), Seen(2, 3) }, track);
    }

    [Theory]
    [InlineData(0.95f, 0.85f)]
    [InlineData(0.85f, 0.95f)]
    public void Of_two_matches_that_would_join_two_features_of_one_view_the_weaker_is_left_out(float first, float second)
    {
        var features = Features(6, 6, 6);

        var tracks = TrackBuilder.Build(features, [
            // View 0 feature 1 -> view 1 feature 2 <- view 0 feature 5: two features of view 0 cannot be one point.
            Match(0, 1, 1, 2, first), Match(1, 2, 0, 5, second),
            Match(1, 2, 2, 0),
            // An unrelated, consistent track.
            Match(0, 3, 1, 4), Match(1, 4, 2, 4),
        ]);

        int kept = first > second ? 1 : 5;
        Assert.Equal(2, tracks.Count);
        Assert.Contains(new[] { Seen(0, kept), Seen(1, 2), Seen(2, 0) }, tracks);
        Assert.Contains(new[] { Seen(0, 3), Seen(1, 4), Seen(2, 4) }, tracks);
    }

    [Fact]
    public void A_wrong_match_between_two_long_tracks_costs_one_link_not_the_tracks()
    {
        var features = Features(4, 4, 4, 4);
        var matches = new List<FeatureMatch>();
        for (int view = 0; view < 3; view++)
        {
            matches.Add(Match(view, 0, view + 1, 0));
            matches.Add(Match(view, 1, view + 1, 1));
        }
        // A chance match between the two points, weaker than the true ones.
        matches.Add(Match(1, 0, 2, 1, 0.81f));

        var tracks = TrackBuilder.Build(features, matches);

        Assert.Equal(2, tracks.Count);
        Assert.Equal(new[] { Seen(0, 0), Seen(1, 0), Seen(2, 0), Seen(3, 0) }, tracks[0]);
        Assert.Equal(new[] { Seen(0, 1), Seen(1, 1), Seen(2, 1), Seen(3, 1) }, tracks[1]);
    }

    [Fact]
    public void Tracks_seen_in_fewer_than_min_views_are_dropped()
    {
        var features = Features(3, 3, 3);
        FeatureMatch[] matches = [Match(0, 0, 1, 0), Match(1, 0, 2, 0), Match(0, 1, 2, 1)];

        var all = TrackBuilder.Build(features, matches);
        var long3 = TrackBuilder.Build(features, matches, minViews: 3);

        Assert.Equal(2, all.Count);
        Assert.Equal(new[] { Seen(0, 1), Seen(2, 1) }, all[1]);
        var track = Assert.Single(long3);
        Assert.Equal(new[] { Seen(0, 0), Seen(1, 0), Seen(2, 0) }, track);
    }

    [Fact]
    public void Output_is_ordered_by_first_observation_whatever_the_order_direction_or_repetition_of_matches()
    {
        var features = Features(5, 5, 5, 5);
        var matches = new List<FeatureMatch>
        {
            Match(2, 1, 3, 2),
            Match(1, 4, 2, 0), Match(2, 0, 3, 3),
            Match(0, 2, 3, 0),
            Match(0, 0, 1, 1), Match(1, 1, 2, 2), Match(2, 2, 3, 4),
        };
        var flipped = matches.Select(m => new FeatureMatch(m.ViewB, m.FeatureB, m.ViewA, m.FeatureA, m.Score));

        var tracks = TrackBuilder.Build(features, matches);
        var shuffled = TrackBuilder.Build(features, flipped.Reverse().Concat(matches));

        var expected = new List<Observation[]>
        {
            new[] { Seen(0, 0), Seen(1, 1), Seen(2, 2), Seen(3, 4) },
            new[] { Seen(0, 2), Seen(3, 0) },
            new[] { Seen(1, 4), Seen(2, 0), Seen(3, 3) },
            new[] { Seen(2, 1), Seen(3, 2) },
        };
        Assert.Equal(expected, tracks);
        Assert.Equal(expected, shuffled);
    }

    [Fact]
    public void A_match_to_a_feature_that_does_not_exist_is_an_error()
    {
        var features = Features(2, 2);

        Assert.Throws<ArgumentOutOfRangeException>(() => TrackBuilder.Build(features, [Match(0, 0, 1, 2)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => TrackBuilder.Build(features, [Match(0, 0, 2, 0)]));
    }
}
