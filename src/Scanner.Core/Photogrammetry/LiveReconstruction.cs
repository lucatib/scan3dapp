using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Photogrammetry;

/// <summary>
/// The photo reconstruction built up while the user scans, for the live view: every photo offered becomes a depth
/// map of the object box (the same cropped plane sweep as <see cref="PhotoVoxelReconstruction"/>), is cross-checked
/// against the earlier maps and fused into one TSDF, and the surface is re-extracted and cut from the table.
/// Uses ARCore's poses as they are; the finished scan refines them and reconstructs again.
/// </summary>
/// <remarks><see cref="Add"/> runs on one worker thread at a time; <see cref="Surface"/> may be read from any thread.</remarks>
public sealed class LiveReconstruction
{
    /// <summary>Depth maps kept for cross-checking; older ones stay fused but no longer confirm new ones.</summary>
    private const int MaxMaps = 24;

    /// <summary>Fewest confirmed pixels for a map to be worth fusing.</summary>
    private const int MinConfirmed = 100;

    private readonly VoxelReconstructionOptions _options;
    private readonly List<PhotoView> _photos = [];
    private readonly List<(PhotoView View, DepthMap Map)> _maps = [];
    private readonly List<bool> _fused = [];
    private readonly TsdfVolume _volume;
    private TriangleMesh? _surface;
    private Vector3 _boxMin = new(float.PositiveInfinity), _boxMax = new(float.NegativeInfinity);
    private FittedSupportPlane? _table; // locked once the photos measured it

    public LiveReconstruction(VoxelReconstructionOptions? options = null)
    {
        _options = options ?? new VoxelReconstructionOptions();
        _volume = new TsdfVolume(_options.VoxelSize, _options.TruncationVoxels * _options.VoxelSize);
    }

    /// <summary>The latest surface of the piece, or null before there is one.</summary>
    public TriangleMesh? Surface => Volatile.Read(ref _surface);

    /// <summary>Depth maps fused so far.</summary>
    public int FusedMaps { get; private set; }

    /// <summary>Remembers <paramref name="photo"/> as a stereo neighbour for later photos. Cheap; call for every photo.</summary>
    public void Remember(PhotoView photo) => _photos.Add(photo);

    /// <summary>
    /// Matches the most recent remembered photo against its neighbours, fuses what the other maps confirm, and
    /// re-extracts the surface. Returns true when the surface changed.
    /// </summary>
    /// <param name="guide">The ARCore points so far: they locate the piece and the table, nothing more.</param>
    public bool Add(Vector3 target, IReadOnlyList<Vector3> guide)
    {
        if (_photos.Count < 2) return false;
        int reference = _photos.Count - 1;
        var view = _photos[reference];

        var guidePlane = SupportPlaneFinder.Fit(guide);
        // The box only grows: ARCore's cluster wanders as points come in, and a shrinking box would cut away
        // surface the user has already seen appear.
        var (newMin, newMax) = PhotoVoxelReconstruction.ObjectBox(guide, target, guidePlane, _options.BoxPadding,
            _options.MaxHalfWidth);
        _boxMin = Vector3.Min(_boxMin, newMin);
        _boxMax = Vector3.Max(_boxMax, newMax);
        var (boxMin, boxMax) = (_boxMin, _boxMax);
        var center = (boxMin + boxMax) / 2;
        if (!Pinhole.Sees(view, center)) return false;
        if (PhotoVoxelReconstruction.Window(view, boxMin, boxMax) is not { } window) return false;
        var neighbours = ViewSelection.Neighbours(_photos, reference, center, _options.Neighbours);
        if (neighbours.Count == 0) return false;

        var (x0, y0, width, height, near, far) = window;
        var crop = view.Crop(x0, y0, width, height);
        var others = neighbours.Select(i => _photos[i]).ToList();
        int samples = PhotoVoxelReconstruction.Samples(crop, others, near, far, _options);
        var stereo = (_options.Stereo ?? new StereoOptions()) with { DepthSamples = samples };
        _maps.Add((crop, PlaneSweepStereo.Compute(crop, others, near, far, stereo)));
        _fused.Add(false);
        if (_maps.Count > MaxMaps)
        {
            _maps.RemoveAt(0);
            _fused.RemoveAt(0);
        }

        // A map is fused once another confirms it: the first ones wait for later photos.
        bool changed = false;
        for (int i = 0; i < _maps.Count; i++)
        {
            if (_fused[i]) continue;
            var confirmed = DepthMapFusion.Confirmed(_maps, i, minAgreeing: 1);
            var mapView = _maps[i].View;
            int count = 0;
            for (int v = 0, pixel = 0; v < confirmed.Height; v++)
            for (int u = 0; u < confirmed.Width; u++, pixel++)
            {
                float d = confirmed.Depth[pixel];
                if (d <= 0) continue;
                if (PhotoVoxelReconstruction.Inside(Pinhole.BackProject(mapView, u, v, d), boxMin, boxMax)) count++;
                else confirmed.Depth[pixel] = 0;
            }
            if (count < MinConfirmed) continue;
            _volume.Integrate(new DepthFrame(mapView.Intrinsics, confirmed.Depth, mapView.CameraToWorld, 0));
            _fused[i] = true;
            FusedMaps++;
            changed = true;
        }
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
        // The live view must only grow: a surface that suddenly halves is an unlucky update, not the scan shrinking.
        if (Surface is { } shown && piece.Positions.Count < shown.Positions.Count / 2) return false;
        Volatile.Write(ref _surface, piece);
        return true;
    }
}
