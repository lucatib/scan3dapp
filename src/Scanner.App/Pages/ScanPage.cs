using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Live;
using Scanner.Capture.Sessions;
using Scanner.Core.Photogrammetry;
using Scanner.Core.Texturing;

namespace Scanner.App.Pages;

public sealed class ScanPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly ArScanView _arView = new();
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startPause = new() { Text = "Start", IsEnabled = false };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private LiveScanSession? _scan;
    private LivePhotogrammetry? _live;

    /// <summary>Settings of the photogrammetry that runs during the scan.</summary>
    private static readonly VoxelReconstructionOptions LiveOptions = new(MaxDepthSamples: 64, MaxHalfWidth: 0.12f, MinWeight: 3);

    /// <summary>New photos between pose refinements during the scan.</summary>
    private const int RefineEvery = 8;

    /// <summary>Most photos the finished scan refines, reconstructs and textures with.</summary>
    private const int MaxFinishPhotos = 48;
    private string? _sessionId;
    private Window? _window;
    private bool _visible;
    private bool _preparing;
    private bool _finishing;

    public ScanPage(SessionStore store)
    {
        _store = store;
        Title = "Scan";
        _startPause.Clicked += OnStartPauseClicked;
        _finish.Clicked += OnFinishClicked;
        _arView.StatusChanged += OnStatusChanged;

        var crosshair = new Label
        {
            Text = "+", FontSize = 40, TextColor = Colors.White, InputTransparent = true,
            HorizontalOptions = LayoutOptions.Center, VerticalOptions = LayoutOptions.Center,
        };
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startPause, _finish } } },
        };
        Content = new Grid { Children = { _arView, crosshair, panel } };
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        _visible = true;
        try
        {
            if (Window is { } window && !ReferenceEquals(window, _window))
            {
                // Subscribe before preparing: installing ARCore leaves the app, and Resumed brings us back here.
                DetachWindow();
                _window = window;
                window.Stopped += OnWindowStopped;
                window.Resumed += OnWindowResumed;
            }
            await StartAsync();
        }
        catch (Exception ex)
        {
            // async void: an exception here would be unobserved and would take the process down.
            _status.Text = $"Could not start the scan: {ex.Message}";
        }
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        _visible = false;
        DetachWindow();
        _arView.Pause();

        // Leaving without finishing discards the unfinished session.
        if (!_finishing && _scan is not null && _scan.State != LiveScanState.Completed && _sessionId is not null)
        {
            _arView.Session = null;
            _arView.Live = null;
            _ = _live?.StopAsync();
            _live = null;
            _scan.Pause();
            _store.Delete(_sessionId);
            _scan = null;
            _sessionId = null;
        }
    }

    /// <summary>Prepares the platform (permission, ARCore install) and creates the session once, then starts AR.
    /// Never throws: it is awaited from async void entry points, where an exception would kill the process.</summary>
    private async Task StartAsync()
    {
        if (_finishing) return;
        try
        {
            if (_scan is null)
            {
                if (_preparing) return;
                _preparing = true;
                string? error;
                try
                {
                    _status.Text = "Preparing the camera…";
                    error = await ArPlatform.PrepareAsync();
                }
                finally
                {
                    _preparing = false;
                }
                if (!_visible) return;
                if (error is not null)
                {
                    _status.Text = error;
                    _startPause.IsEnabled = false;
                    return;
                }
                var (id, directory) = _store.CreateNew();
                _sessionId = id;
                // ScanSessionWriter creates the session folder, so this can fail on a full or read-only volume.
                _scan = new LiveScanSession(new ScanSessionWriter(directory, id, DeviceInfo.Current.Model),
                    new LiveScanOptions(RequireStableDepth: true));
                _live = new LivePhotogrammetry(LiveOptions, refineEvery: RefineEvery);
                _arView.Live = _live;
                _arView.Session = _scan;
                _startPause.IsEnabled = true;
                _status.Text = "Aim the crosshair at the piece and press Start.";
            }
            _arView.Resume();
        }
        catch (Exception ex)
        {
            // Expected: IOException and UnauthorizedAccessException from creating the session folder, and whatever
            // the platform preparation throws. Anything else must be shown rather than escape unobserved.
            // A session that was created stays usable (only AR failed to start); without one, Start stays disabled.
            _status.Text = $"Could not start the scan: {ex.Message}";
            _startPause.IsEnabled = _scan is not null;
        }
    }

    private void DetachWindow()
    {
        if (_window is null) return;
        _window.Stopped -= OnWindowStopped;
        _window.Resumed -= OnWindowResumed;
        _window = null;
    }

    private void OnWindowStopped(object? sender, EventArgs e) => _arView.Pause();

    private async void OnWindowResumed(object? sender, EventArgs e)
    {
        try
        {
            if (_visible) await StartAsync(); // also retries preparation after the ARCore install flow
        }
        catch (Exception ex)
        {
            // StartAsync already guards itself; this is the async void backstop that must never let anything escape.
            _status.Text = $"Could not resume the scan: {ex.Message}";
        }
    }

    private void OnStartPauseClicked(object? sender, EventArgs e)
    {
        if (_scan is null || _scan.State == LiveScanState.Completed) return;
        if (_scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget) _scan.Pause();
        else _scan.RequestStart();
        UpdateButtons();
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        if (_scan is null || _sessionId is null || _finishing) return;
        var scan = _scan;
        string sessionId = _sessionId;
        if (LoopGap(scan) is > LoopCoverage.MaxGapDegrees and < 360 and var gap)
        {
            _finishing = true; // no second Finish while the question is open
            bool finish;
            try
            {
                finish = await DisplayAlertAsync("Loop not closed",
                    $"The photos miss {gap:F0}° of the way around the piece. That side will come out wrong.",
                    "Finish anyway", "Keep scanning");
            }
            finally
            {
                _finishing = false;
            }
            if (!finish || _scan != scan) return;
        }
        _finishing = true;
        _finish.IsEnabled = false;
        _startPause.IsEnabled = false;
        _status.Text = "Reconstructing photos — this may take a minute…";
        scan.Pause();
        _arView.Pause(); // drains the GL draw before waiting for its registered photo worker
        try
        {
            await scan.WaitForPhotoCaptureAsync();
            var live = _live;
            if (live is not null) await live.StopAsync(); // the finished scan gets the whole CPU
            string directory = _store.DirectoryOf(sessionId);
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var (result, textured) = await Task.Run(() =>
            {
                // The ARCore points only locate the piece: the geometry is the photos', fused in voxels.
                // At most MaxFinishPhotos, spread over the capture: a 47-photo scan was as good as a 90-photo one, and
                // pose refinement and texturing grow with the count. Indices pair them with the stored photos.
                var previews = scan.PreviewPhotos();
                bool paired = live is not null && live.Poses.Count == previews.Length;
                var all = paired ? live!.Poses.Snapshot().Photos : previews;
                var used = ViewSelection.References(all.Length, MaxFinishPhotos);
                var photos = used.Select(i => all[i]).ToArray();
                VoxelReconstruction? reconstruction = null;
                if (scan.Target is { } target && photos.Length >= 3)
                {
                    try
                    {
                        // ARCore poses are only the starting guess: the photos refined them against each other during the
                        // scan. Only when the last photos went unrefined for too long is it done again here.
                        var log = new LogcatWriter();
                        if (!paired)
                            photos = PoseRefiner.Refine(photos, target, log: log).Views;
                        else
                        {
                            if (live!.Poses.Version == 0 || all.Length - live.Poses.RefinedCount >= RefineEvery)
                            {
                                live.Poses.Refine(target, log);
                                all = live.Poses.Snapshot().Photos;
                            }
                            // Exactly the photos the refinement adjusted: the others only carry a neighbour's correction.
                            if (live.Poses.RefinedPhotos is { Length: >= 3 } refined) used = [.. refined];
                            photos = used.Select(i => all[i]).ToArray();
                        }
                        Console.WriteLine($"Scan3D: refined {photos.Length} of {all.Length} photos at {clock.Elapsed.TotalSeconds:F1} s");
                        reconstruction = PhotoVoxelReconstruction.Reconstruct(photos, target, scan.SnapshotPoints(),
                            new VoxelReconstructionOptions(ReferenceViews: 8, MaxDepthSamples: 64, MaxHalfWidth: 0.12f, MinWeight: 1,
                                MaxCropPixels: 384),
                            log, live?.PieceBounds);
                        Console.WriteLine($"Scan3D: reconstructed at {clock.Elapsed.TotalSeconds:F1} s");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Scan3D: Photo reconstruction failed: {ex}");
                    }
                }
                var completed = scan.Complete(reconstructedPiece: reconstruction?.Mesh.Positions,
                    reconstructedPlane: reconstruction?.Plane);
                bool wroteModel = false;
                if (reconstruction is { Mesh.TriangleCount: > 0 } && completed.PiecePoints.Length == reconstruction.Mesh.Positions.Count)
                {
                    try
                    {
                        // The previews are the photos in capture order, so they pair with the stored photos one to one.
                        var stored = ScanSessionReader.ReadPhotos(directory);
                        var cameras = stored.Count == all.Length
                            ? used.Select((index, k) => TextureCamera.FromView(stored[index].Photo, photos[k])).ToList()
                            : stored.Select(p => TextureCamera.FromPhoto(p.Photo)).ToList();
                        TexturedModelFile.Write(directory, MeshTexturer.Texture(reconstruction.Mesh, cameras));
                        wroteModel = true;
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Scan3D: Texturing failed: {ex}");
                    }
                }
                Console.WriteLine($"Scan3D finish: {clock.Elapsed.TotalSeconds:F1} s, maps {reconstruction?.DepthMaps}, "
                    + $"triangles {reconstruction?.Mesh.TriangleCount}, piece {completed.PiecePoints.Length}, model {wroteModel}");
                return (completed, wroteModel);
            });
            _status.Text = textured ? $"Photo model ready in {clock.Elapsed.TotalSeconds:F0} s."
                : result.Isolated ? "Piece isolated from the table." : "Could not isolate the piece; showing all points.";
            if (_visible) await Shell.Current.GoToAsync($"preview?id={sessionId}");
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not finish the scan: {ex.Message}";
        }
    }

    private void OnStatusChanged(object? sender, ArScanStatus status)
    {
        if (_scan is null || _finishing || _scan.State == LiveScanState.Completed) return;
        _status.Text = $"{status.Tracking} · {status.State} · {status.PointCount:N0} points · {status.FrameCount} depth frames"
                       + $" · {status.PhotoCount} photos"
                       + (LoopGap(_scan) is < 360 and var gap ? $" · loop {360 - gap:F0}°" : "")
                       + (_scan.State == LiveScanState.Recording && !_scan.IsDepthReady
                           ? "\nWaiting for stable depth — keep the table in view and move slowly." : "")
                       + (status.Message is { } message ? $"\n{message}" : "");
        UpdateButtons();
    }

    /// <summary>The widest turn around the target no photo was taken from (360 before there is a target).</summary>
    private static float LoopGap(LiveScanSession scan) => scan.Target is { } target
        ? LoopCoverage.LargestGapDegrees(scan.PreviewPhotos().Select(p => p.CameraToWorld.Translation), target)
        : 360f;

    private void UpdateButtons()
    {
        if (_scan is null || _scan.State == LiveScanState.Completed) return;
        bool running = _scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget;
        _startPause.Text = running ? "Pause" : _scan.FrameCount > 0 ? "Resume" : "Start";
        _finish.IsEnabled = _scan.PointCount > 0;
    }
}

/// <summary>Sends reconstruction progress lines to the device log (tag DOTNET, prefixed Scan3D).</summary>
internal sealed class LogcatWriter : StringWriter
{
    public override void WriteLine(string? value) => Console.WriteLine($"Scan3D: {value}");
}
