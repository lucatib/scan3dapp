using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <summary>What the scan knows when a worker looks: the target (null before Start), the ARCore points, and whether it
/// is still recording.</summary>
public readonly record struct ScanProgress(Vector3? Target, IReadOnlyList<Vector3> Guide, bool Recording);

/// <summary>
/// Photogrammetry that runs while the user scans, on two background workers: one refines the photo poses every
/// <c>refineEvery</c> new photos (<see cref="LivePoses"/>), the other turns each new photo into a depth map and fuses
/// it into the live surface (<see cref="LiveReconstruction"/>) with the best poses at hand. Photos that arrive while a
/// worker is busy are not lost: the poses keep every photo, and the surface worker matches the newest one next.
/// </summary>
public sealed class LivePhotogrammetry
{
    private readonly LiveReconstruction _reconstruction;
    private readonly int _refineEvery;
    private int _reconstructing;
    private int _refining;
    private Task _reconstructTask = Task.CompletedTask;
    private Task _refineTask = Task.CompletedTask;
    private volatile bool _stopped;
    private Vector3[]? _pieceBounds;

    public LivePhotogrammetry(VoxelReconstructionOptions reconstruction, PoseRefinementOptions? refinement = null,
        int maxRefinedPhotos = 48, int refineEvery = 8)
    {
        _reconstruction = new LiveReconstruction(reconstruction);
        Poses = new LivePoses(refinement, maxRefinedPhotos);
        _refineEvery = refineEvery;
    }

    public LivePoses Poses { get; }

    /// <summary>The latest live surface, or null before there is one.</summary>
    public Meshing.TriangleMesh? Surface => _reconstruction.Surface;

    /// <summary>The extent of the live surface (min, max), or null before there is one.</summary>
    public (Vector3 Min, Vector3 Max)? PieceBounds => Volatile.Read(ref _pieceBounds) is { } b ? (b[0], b[1]) : null;

    /// <summary>The last worker failure, for the status line; the workers carry on.</summary>
    public string? LastError { get; private set; }

    /// <summary>Hands over the next recorded photo, in capture order. Call from the photo worker; returns at once
    /// apart from finding the photo's corners.</summary>
    public void Offer(PhotoView photo, Func<ScanProgress> state)
    {
        if (_stopped) return;
        PoseRefiner.Prepare(photo, Poses.Options); // corners now, not when the user presses Finish
        Poses.Add(photo);

        if (Poses.Count - Poses.RefinedCount >= _refineEvery && Interlocked.CompareExchange(ref _refining, 1, 0) == 0)
        {
            _refineTask = Task.Run(() =>
            {
                try
                {
                    if (!_stopped && state() is { Recording: true, Target: { } target }) Poses.Refine(target);
                }
                catch (Exception ex)
                {
                    LastError = $"Pose refinement failed: {ex.Message}";
                }
                finally
                {
                    Volatile.Write(ref _refining, 0);
                }
            });
        }

        if (Interlocked.CompareExchange(ref _reconstructing, 1, 0) != 0) return;
        _reconstructTask = Task.Run(() =>
        {
            try
            {
                if (_stopped || state() is not { Recording: true, Target: { } target } current) return;
                var (photos, version) = Poses.Snapshot();
                if (_reconstruction.Add(photos, version, target, current.Guide) && Surface is { Positions.Count: > 0 } surface)
                    Volatile.Write(ref _pieceBounds, Bounds(surface.Positions));
            }
            catch (Exception ex)
            {
                LastError = $"Live reconstruction failed: {ex.Message}";
            }
            finally
            {
                Volatile.Write(ref _reconstructing, 0);
            }
        });
    }

    /// <summary>The 1st to 99th percentile per axis: a few stray vertices must not inflate the box Finish crops to.</summary>
    private static Vector3[] Bounds(IReadOnlyList<Vector3> points)
    {
        float At(Func<Vector3, float> axis, float q)
        {
            var values = points.Select(axis).Order().ToArray();
            return values[(int)((values.Length - 1) * q)];
        }
        return [new(At(p => p.X, .01f), At(p => p.Y, .01f), At(p => p.Z, .01f)),
            new(At(p => p.X, .99f), At(p => p.Y, .99f), At(p => p.Z, .99f))];
    }

    /// <summary>Waits for both workers; for replaying a scan in tests, where every photo should be processed.</summary>
    public void WaitIdleForTest() => Task.WaitAll(Volatile.Read(ref _reconstructTask), Volatile.Read(ref _refineTask));

    /// <summary>Stops accepting photos and waits for both workers, so that the finished scan has the CPU.</summary>
    public async Task StopAsync()
    {
        _stopped = true;
        await Task.WhenAll(Volatile.Read(ref _reconstructTask), Volatile.Read(ref _refineTask)).ConfigureAwait(false);
    }
}
