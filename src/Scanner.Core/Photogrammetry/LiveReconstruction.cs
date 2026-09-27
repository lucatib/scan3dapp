using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// The photo reconstruction built up while the user scans, for the live view: each new photo becomes a depth map of the
/// object box (the same cropped plane sweep as <see cref="PhotoVoxelReconstruction"/>), is cross-checked against the
/// earlier maps and fused into one TSDF, and the surface is re-extracted and cut from the table. Poses come from
/// <see cref="LivePoses"/>; when a refinement changes them, the stored maps are fused again with the new poses, so
/// that layers the old poses put side by side merge.
/// </summary>
/// <remarks><see cref="Add"/> runs on one worker thread at a time; <see cref="Surface"/> may be read from any thread.</remarks>
public sealed class LiveReconstruction
{
    /// <summary>Most recent depth maps used to cross-check a new one.</summary>
    private const int CheckWindow = 24;

    /// <summary>Fewest confirmed pixels for a map to be worth fusing.</summary>
    private const int MinConfirmed = 100;

    private readonly VoxelReconstructionOptions _options;
    private readonly int _minAgreeing;
    private readonly List<Entry> _maps = [];
    private TsdfVolume _volume;
    private int _fusedVersion = -1;
    private int _lastPhoto = -1;
    private int _shownVersion = -1;
    private TriangleMesh? _surface;
    private Vector3 _boxMin = new(float.PositiveInfinity), _boxMax = new(float.NegativeInfinity);
    private FittedSupportPlane? _table; // locked once the photos measured it

    /// <summary>A depth map of photo <see cref="Photo"/>'s crop; <see cref="Confirmed"/> once other maps agreed with it.</summary>
    private sealed class Entry(int photo, PhotoView crop, DepthMap map)
    {
        public int Photo { get; } = photo;
        public PhotoView Crop { get; set; } = crop;
        /// <summary>Released once the map leaves the cross-check window; only the confirmed depths stay.</summary>
        public DepthMap? Map { get; set; } = map;
        public float[]? Confirmed { get; set; }
    }

    /// <param name="minAgreeing">Other depth maps that must confirm a pixel before it is fused.</param>
    public LiveReconstruction(VoxelReconstructionOptions? options = null, int minAgreeing = 1)
    {
        _options = options ?? new VoxelReconstructionOptions();
        _minAgreeing = minAgreeing;
        _volume = NewVolume();
    }

    /// <summary>The latest surface of the piece, or null before there is one.</summary>
    public TriangleMesh? Surface => Volatile.Read(ref _surface);

    /// <summary>Depth maps fused so far.</summary>
    public int FusedMaps => _maps.Count(m => m.Confirmed is not null);

    /// <summary>
    /// Matches the newest of <paramref name="photos"/> against its neighbours, fuses what the other maps confirm, and
    /// re-extracts the surface. Returns true when the surface changed.
    /// </summary>
    /// <param name="photos">Every photo so far with its current best pose (<see cref="LivePoses.Snapshot"/>).</param>
    /// <param name="sharp">Which photos are sharp enough to match; blurred ones are neither references nor neighbours.</param>
    /// <param name="poseVersion">The version of those poses; a new one re-fuses every stored map.</param>
    /// <param name="guide">The ARCore points so far: they locate the piece and the table, nothing more.</param>
    public bool Add(IReadOnlyList<PhotoView> photos, IReadOnlyList<bool> sharp, int poseVersion, Vector3 target,
        IReadOnlyList<Vector3> guide)
    {
        if (photos.Count < 2) return false;
        bool changed = false;
        if (poseVersion != _fusedVersion)
        {
            // New poses: every stored map moves with its photo, and the volume is fused again from scratch.
            foreach (var entry in _maps) entry.Crop = entry.Crop with { CameraToWorld = photos[entry.Photo].CameraToWorld };
            _volume = NewVolume();
            foreach (var entry in _maps.Where(m => m.Confirmed is not null)) Integrate(entry);
            _fusedVersion = poseVersion;
            _table = null; // measured in the old poses: measure it again in the new ones
            changed = _maps.Count > 0;
        }

        var guidePlane = SupportPlaneFinder.Fit(guide);
        // The box only grows: ARCore's cluster wanders as points come in, and a shrinking box would cut away
        // surface the user has already seen appear.
        var (newMin, newMax) = PhotoVoxelReconstruction.ObjectBox(guide, target, guidePlane, _options.BoxPadding,
            _options.MaxHalfWidth);
        _boxMin = Vector3.Min(_boxMin, newMin);
        _boxMax = Vector3.Max(_boxMax, newMax);

        int reference = photos.Count - 1;
        if (reference != _lastPhoto && sharp[reference] && AddMap(photos, sharp, reference)) changed |= FuseConfirmed();
        _lastPhoto = reference;
        if (!changed) return false;

        var mesh = SurfaceNets.Extract(_volume, _options.MinWeight);
        var plane = _table;
        if (plane is null)
        {
            // Locked once the photos measure it: a table that moved between updates would cut differently.
            plane = PhotoVoxelReconstruction.TablePlane(mesh.Positions, guidePlane, out bool measured);
            if (measured) _table = plane;
        }
        var piece = MeshCleanup.Piece(mesh, target, plane, 2 * _options.VoxelSize);
        // Between pose updates the live view must only grow: a surface that suddenly halves is an unlucky update, not the
        // scan shrinking. After a pose update it may: fused again with better poses, splatter goes. Refusing that froze
        // an exploded surface on screen for the rest of a scan.
        if (Surface is { } shown && _shownVersion == poseVersion && piece.Positions.Count < shown.Positions.Count / 2) return false;
        _shownVersion = poseVersion;
        Volatile.Write(ref _surface, piece);
        return true;
    }

    /// <summary>The depth map of photo <paramref name="reference"/>'s view of the box, if it sees it and has neighbours.</summary>
    private bool AddMap(IReadOnlyList<PhotoView> photos, IReadOnlyList<bool> sharp, int reference)
    {
        var view = photos[reference];
        var center = (_boxMin + _boxMax) / 2;
        if (!Pinhole.Sees(view, center)) return false;
        if (PhotoVoxelReconstruction.Window(view, _boxMin, _boxMax) is not { } window) return false;
        var neighbours = ViewSelection.Neighbours(photos, reference, center, _options.Neighbours, sharp);
        if (neighbours.Count == 0) return false;

        var (x0, y0, width, height, near, far) = window;
        var crop = PhotoVoxelReconstruction.FitCrop(view.Crop(x0, y0, width, height), _options);
        var others = neighbours.Select(i => photos[i]).ToList();
        int samples = PhotoVoxelReconstruction.Samples(crop, others, near, far, _options);
        var stereo = (_options.Stereo ?? new StereoOptions()) with { DepthSamples = samples };
        _maps.Add(new Entry(reference, crop, PlaneSweepStereo.Compute(crop, others, near, far, stereo)));
        return true;
    }

    /// <summary>Fuses every recent map that another now confirms: the first ones wait for later photos.</summary>
    private bool FuseConfirmed()
    {
        var window = _maps.Skip(Math.Max(0, _maps.Count - CheckWindow)).ToList();
        foreach (var old in _maps.Take(Math.Max(0, _maps.Count - CheckWindow))) old.Map = null;
        var pairs = window.Select(m => (m.Crop, m.Map!)).ToList();
        bool fused = false;
        for (int i = 0; i < window.Count; i++)
        {
            if (window[i].Confirmed is not null) continue;
            var confirmed = DepthMapFusion.Confirmed(pairs, i, _minAgreeing);
            var view = window[i].Crop;
            int count = 0;
            for (int v = 0, pixel = 0; v < confirmed.Height; v++)
            for (int u = 0; u < confirmed.Width; u++, pixel++)
            {
                float d = confirmed.Depth[pixel];
                if (d <= 0) continue;
                if (PhotoVoxelReconstruction.Inside(Pinhole.BackProject(view, u, v, d), _boxMin, _boxMax)) count++;
                else confirmed.Depth[pixel] = 0;
            }
            if (count < MinConfirmed) continue;
            window[i].Confirmed = confirmed.Depth;
            Integrate(window[i]);
            fused = true;
        }
        return fused;
    }

    private void Integrate(Entry entry) =>
        _volume.Integrate(new DepthFrame(entry.Crop.Intrinsics, entry.Confirmed!, entry.Crop.CameraToWorld, 0));

    private TsdfVolume NewVolume() => new(_options.VoxelSize, _options.TruncationVoxels * _options.VoxelSize);
}
