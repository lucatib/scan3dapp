using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// The pose of every photo of a running scan, refined while the scan goes on. Photos arrive with ARCore's pose; now and
/// then <see cref="Refine"/> bundle-adjusts a spread of them (from ARCore's poses, so the result stays anchored to
/// ARCore's frame, scale and gravity). A photo that was not part of the last refinement takes the correction of the
/// nearest refined photo in capture order: ARCore's error drifts slowly, so its neighbour's correction is most of its own.
/// </summary>
/// <remarks>Thread-safe: photos are added by the capture worker, refined by another, read by the reconstruction.</remarks>
public sealed class LivePoses(PoseRefinementOptions? options = null, int maxRefinedPhotos = 48)
{
    private readonly object _gate = new();
    private readonly List<PhotoView> _arcore = [];
    private Dictionary<int, Matrix4x4> _refined = [];
    private int[] _refinedOrder = [];

    public PoseRefinementOptions Options { get; } = options ?? new PoseRefinementOptions();

    /// <summary>Photos added so far.</summary>
    public int Count
    {
        get { lock (_gate) return _arcore.Count; }
    }

    /// <summary>Photos there were when the last refinement ran, trusted or not; 0 before one.</summary>
    public int RefinedCount { get; private set; }

    /// <summary>Changes whenever a refinement changes poses.</summary>
    public int Version { get; private set; }

    /// <summary>The photos the last trusted refinement adjusted, in capture order; empty before one. Only these have
    /// bundle-adjusted poses: use them, not a fresh spread over all photos, where precision matters.</summary>
    public int[] RefinedPhotos
    {
        get { lock (_gate) return [.. _refinedOrder]; }
    }

    /// <summary>Adds the next photo in capture order, with its ARCore pose.</summary>
    public int Add(PhotoView photo)
    {
        lock (_gate)
        {
            _arcore.Add(photo);
            return _arcore.Count - 1;
        }
    }

    /// <summary>Every photo so far, with the best pose known for it, and the pose version it reflects.</summary>
    public (PhotoView[] Photos, int Version) Snapshot()
    {
        lock (_gate)
        {
            var photos = new PhotoView[_arcore.Count];
            for (int i = 0; i < photos.Length; i++) photos[i] = _arcore[i] with { CameraToWorld = Best(i) };
            return (photos, Version);
        }
    }

    /// <summary>
    /// Bundle-adjusts up to maxRefinedPhotos photos spread over the ones so far. Returns false (and keeps the poses)
    /// when there are too few photos or the adjustment is not trusted. Runs on one worker at a time.
    /// </summary>
    public bool Refine(Vector3 target, TextWriter? log = null)
    {
        PhotoView[] arcore;
        lock (_gate) arcore = _arcore.ToArray();
        if (arcore.Length < 6) return false;
        var used = ViewSelection.References(arcore.Length, maxRefinedPhotos);
        var result = PoseRefiner.Refine(used.Select(i => arcore[i]).ToArray(), target, Options, log);
        lock (_gate)
        {
            RefinedCount = arcore.Length;
            if (!result.Applied) return false;
            _refined = used.Select((index, k) => (index, k)).ToDictionary(p => p.index, p => result.Views[p.k].CameraToWorld);
            _refinedOrder = [.. used];
            Version++;
            return true;
        }
    }

    // Caller holds _gate.
    private Matrix4x4 Best(int index)
    {
        var arcore = _arcore[index].CameraToWorld;
        if (_refined.TryGetValue(index, out var refined)) return refined;
        if (_refinedOrder.Length == 0) return arcore;
        int nearest = _refinedOrder.MinBy(i => Math.Abs(i - index));
        // Row vectors: world = camera · pose. The neighbour's correction is the world→world map D with
        // refined = arcore · D, applied to this photo's ARCore pose.
        if (!Matrix4x4.Invert(_arcore[nearest].CameraToWorld, out var inverse)) return arcore;
        return arcore * (inverse * _refined[nearest]);
    }
}
