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
    private readonly List<float> _sharpness = [];
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

    /// <summary>Adds the next photo in capture order, with its ARCore pose and its <see cref="PhotoQuality.Sharpness"/>.</summary>
    public int Add(PhotoView photo, float sharpness)
    {
        lock (_gate)
        {
            _arcore.Add(photo);
            _sharpness.Add(sharpness);
            return _arcore.Count - 1;
        }
    }

    /// <summary>Every photo so far, with the best pose known for it; which of them are sharp enough to match (see
    /// <see cref="PhotoQuality.MinRelativeSharpness"/>); and the pose version it reflects.</summary>
    public (PhotoView[] Photos, bool[] Sharp, int Version) Snapshot()
    {
        lock (_gate)
        {
            var photos = new PhotoView[_arcore.Count];
            for (int i = 0; i < photos.Length; i++) photos[i] = _arcore[i] with { CameraToWorld = Best(i) };
            return (photos, SharpFlags(), Version);
        }
    }

    /// <summary>True when the newest photo is blurred: the user is moving too fast.</summary>
    public bool LatestIsBlurred
    {
        get
        {
            lock (_gate) return _arcore.Count > 4 && !SharpFlags()[^1];
        }
    }

    /// <summary>
    /// The world-to-world correction of the newest photo: refined = ARCore · correction. ARCore'"'"'s current frame is that
    /// of its newest poses, so geometry built with refined poses is drawn over the camera image through its inverse.
    /// </summary>
    public Matrix4x4 LatestCorrection()
    {
        lock (_gate)
        {
            if (_arcore.Count == 0 || !Matrix4x4.Invert(_arcore[^1].CameraToWorld, out var inverse)) return Matrix4x4.Identity;
            return inverse * Best(_arcore.Count - 1);
        }
    }

    /// <summary>
    /// Bundle-adjusts up to maxRefinedPhotos photos spread over the ones so far. Returns false (and keeps the poses)
    /// when there are too few photos or the adjustment is not trusted. Runs on one worker at a time.
    /// </summary>
    public bool Refine(Vector3 target, TextWriter? log = null)
    {
        PhotoView[] arcore;
        bool[] sharp;
        lock (_gate)
        {
            arcore = _arcore.ToArray();
            sharp = SharpFlags();
        }
        // Blurred photos are left out: they match poorly, and nothing downstream uses them.
        var candidates = Enumerable.Range(0, arcore.Length).Where(i => sharp[i]).ToList();
        if (candidates.Count < 6) return false;
        var used = ViewSelection.References(candidates.Count, maxRefinedPhotos).Select(k => candidates[k]).ToList();
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

    // Caller holds _gate. Sharp: at least MinRelativeSharpness of the median so far.
    private bool[] SharpFlags()
    {
        if (_sharpness.Count == 0) return [];
        float median = _sharpness.Order().ElementAt(_sharpness.Count / 2);
        return _sharpness.Select(s => s >= PhotoQuality.MinRelativeSharpness * median).ToArray();
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
