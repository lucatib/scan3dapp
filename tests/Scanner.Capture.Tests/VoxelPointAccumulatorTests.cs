using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class VoxelPointAccumulatorTests
{
    [Fact]
    public void Points_in_the_same_voxel_are_averaged()
    {
        var acc = new VoxelPointAccumulator(0.005f);

        acc.AddRange([new Vector3(0.001f, 0.001f, 0.001f), new Vector3(0.003f, 0.003f, 0.003f)]);

        var snapshot = acc.Snapshot();
        Assert.Single(snapshot);
        Assert.True(Vector3.Distance(snapshot[0], new Vector3(0.002f)) < 1e-6f);
    }

    [Fact]
    public void Negative_coordinates_use_floor_cells()
    {
        var acc = new VoxelPointAccumulator(0.005f);

        acc.AddRange([new Vector3(-0.001f, 0, 0), new Vector3(0.001f, 0, 0)]);

        Assert.Equal(2, acc.CellCount);
    }

    [Fact]
    public void Min_observations_filters_single_hits()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        acc.AddRange([Vector3.Zero, Vector3.Zero, new Vector3(0.1f, 0, 0)]);

        Assert.Equal(2, acc.Snapshot().Length);
        Assert.Single(acc.Snapshot(minObservations: 2));
    }

    [Fact]
    public void Clear_empties_the_cloud()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        acc.AddRange([Vector3.One]);

        acc.Clear();

        Assert.Equal(0, acc.CellCount);
        Assert.Empty(acc.Snapshot());
    }

    [Fact]
    public async Task Concurrent_adds_and_snapshots_do_not_throw()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        var writer = Task.Run(() =>
        {
            for (int i = 0; i < 200; i++)
                acc.AddRange(Enumerable.Range(0, 100).Select(j => new Vector3(i * 0.01f, j * 0.01f, 0)));
        });
        var reader = Task.Run(() =>
        {
            for (int i = 0; i < 200; i++) _ = acc.Snapshot();
        });

        await Task.WhenAll(writer, reader);

        Assert.Equal(20000, acc.CellCount);
    }
}
