using System.Diagnostics;
using System.Numerics;
using ARKit;
using CoreGraphics;
using Foundation;
using Microsoft.Maui.Handlers;
using SceneKit;
using Scanner.App.Controls;
using Scanner.App.iOS.Ar;
using Scanner.Capture.Live;
using Scanner.Core.Meshing;
using UIKit;

namespace Scanner.App.iOS.Handlers;

/// <summary>
/// The scan screen on a LiDAR iPhone: ARKit tracks the phone and builds a mesh of the room, drawn as a wireframe
/// over the camera. Every frame goes through <see cref="FrameReceiver"/> on the main thread, which picks the target,
/// hands LiDAR depth and photos to the scan, and reports the status. On Finish the page reads ARKit's mesh
/// (<see cref="ArScanView.ReadSensorMesh"/>) and cuts the piece out of it.
/// </summary>
public sealed class ArScanViewHandler : ViewHandler<ArScanView, ARSCNView>
{
    public static readonly IPropertyMapper<ArScanView, ArScanViewHandler> PropertyMapper =
        new PropertyMapper<ArScanView, ArScanViewHandler>(ViewMapper);

    public static readonly CommandMapper<ArScanView, ArScanViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(ArScanView.Resume)] = (handler, _, _) => handler.ResumeAr(),
            [nameof(ArScanView.Pause)] = (handler, _, _) => handler.PauseAr(),
        };

    private FrameReceiver? _receiver;
    private bool _running;

    public ArScanViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    private ArScanView? CurrentView => ((IElementHandler)this).VirtualView as ArScanView;

    protected override ARSCNView CreatePlatformView()
    {
        var view = new ARSCNView { AutomaticallyUpdatesLighting = true };
        _receiver = new FrameReceiver(this, view);
        view.Delegate = new MeshOverlay();
        view.Session.Delegate = _receiver;
        return view;
    }

    protected override void ConnectHandler(ARSCNView platformView)
    {
        base.ConnectHandler(platformView);
        VirtualView.SensorMeshReader = ReadMesh;
        if (VirtualView.IsResumeRequested) ResumeAr(); // Resume() was called before this handler existed
    }

    protected override void DisconnectHandler(ARSCNView platformView)
    {
        PauseAr();
        if (CurrentView is { } view) view.SensorMeshReader = null;
        platformView.Session.Delegate = null!;
        platformView.Delegate = null!;
        base.DisconnectHandler(platformView);
    }

    private void ResumeAr()
    {
        if (_running || PlatformView is not { } view) return;
        try
        {
            var configuration = new ARWorldTrackingConfiguration
            {
                SceneReconstruction = ARSceneReconstruction.Mesh,
                FrameSemantics = ARFrameSemantics.SceneDepth,
                PlaneDetection = ARPlaneDetection.Horizontal,
            };
            view.Session.Run(configuration, 0); // resuming keeps the mesh and the world frame
            _running = true;
        }
        catch (Exception ex)
        {
            CurrentView?.ReportStatus(new ArScanStatus("Unavailable", CurrentView.Session?.State ?? LiveScanState.Idle,
                0, 0, 0, ex.Message));
        }
    }

    private void PauseAr()
    {
        if (!_running) return;
        _running = false;
        PlatformView?.Session.Pause();
    }

    /// <summary>ARKit's mesh of everything it has seen, in world metres. Main thread.</summary>
    private IReadOnlyList<MeshChunk> ReadMesh()
    {
        using var frame = PlatformView?.Session.CurrentFrame;
        return frame is null ? [] : ArKitReader.ReadMesh(frame.Anchors);
    }

    /// <summary>Receives ARKit's frames (on the main thread) and feeds the scan.</summary>
    private sealed class FrameReceiver(ArScanViewHandler handler, ARSCNView view) : ARSessionDelegate
    {
        private static readonly TimeSpan StatusInterval = TimeSpan.FromMilliseconds(250);

        private readonly WeakReference<ArScanViewHandler> _handler = new(handler);
        private readonly WeakReference<ARSCNView> _view = new(view);
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private TimeSpan _lastStatus;
        private int _integrating;
        private int _capturingPhoto;
        private volatile string? _message;

        public override void DidUpdateFrame(ARSession session, ARFrame frame)
        {
            // An ARFrame not released at once holds ARKit's camera pool: ARKit then stops delivering frames.
            using (frame)
            {
                if (!_handler.TryGetTarget(out var handler) || handler.CurrentView is not { } arView) return;
                var scan = arView.Session;
                bool tracking = frame.Camera.TrackingState == ARTrackingState.Normal;
                try
                {
                    if (tracking && scan is not null)
                    {
                        if (scan.State == LiveScanState.WaitingForTarget) TryPickTarget(session, frame, scan);
                        else if (scan.State == LiveScanState.Recording)
                        {
                            TryIntegrate(frame, scan);
                            TryCapturePhoto(frame, scan);
                        }
                    }
                }
                catch (Exception ex)
                {
                    _message = ex.Message;
                }

                if (_clock.Elapsed - _lastStatus < StatusInterval) return;
                _lastStatus = _clock.Elapsed;
                arView.ReportStatus(new ArScanStatus(tracking ? "Tracking" : "Move the phone slowly",
                    scan?.State ?? LiveScanState.Idle, scan?.PointCount ?? 0, scan?.FrameCount ?? 0,
                    scan?.PhotoCount ?? 0, _message));
                _message = null;
            }
        }

        public override void DidFail(ARSession session, NSError error) => _message = error.LocalizedDescription;

        /// <summary>The target is the surface under the crosshair (screen centre); the table is the highest
        /// horizontal plane ARKit found below it.</summary>
        private void TryPickTarget(ARSession session, ARFrame frame, LiveScanSession scan)
        {
            if (!_view.TryGetTarget(out var view)) return;
            var centre = new CGPoint(view.Bounds.GetMidX(), view.Bounds.GetMidY());
            using var query = view.CreateRaycastQuery(centre, ARRaycastTarget.EstimatedPlane, ARRaycastTargetAlignment.Any);
            var hit = query is null ? null : session.Raycast(query).FirstOrDefault();
            if (hit is null)
            {
                _message = "Aim the crosshair at the piece";
                return;
            }
            var target = ArKitReader.Translation(hit.WorldTransform);

            float? tableHeight = null;
            foreach (var plane in frame.Anchors.OfType<ARPlaneAnchor>())
            {
                if (plane.Alignment != ARPlaneAnchorAlignment.Horizontal) continue;
                float y = ArKitReader.Translation(plane.Transform).Y;
                if (y < target.Y - 0.005f && (tableHeight is null || y > tableHeight)) tableHeight = y;
            }
            scan.SetTarget(target, tableHeight);
        }

        private void TryIntegrate(ARFrame frame, LiveScanSession scan)
        {
            if (Interlocked.CompareExchange(ref _integrating, 1, 0) != 0) return; // previous frame still integrating
            (Scanner.Capture.DepthFrame Frame, byte[] Confidence)? depth = null;
            try
            {
                if (scan.ShouldIntegrate(frame.Timestamp)) depth = ArKitReader.ReadDepth(frame);
            }
            finally
            {
                if (depth is null) Volatile.Write(ref _integrating, 0);
            }
            if (depth is not { } read) return;

            Task.Run(() =>
            {
                try
                {
                    scan.Integrate(read.Frame, read.Confidence);
                }
                catch (Exception ex)
                {
                    _message = $"Integration failed: {ex.Message}";
                }
                finally
                {
                    Volatile.Write(ref _integrating, 0);
                }
            });
        }

        /// <summary>Takes a photo when the scan asks for one; the copy and JPEG run on the thread pool.</summary>
        private void TryCapturePhoto(ARFrame frame, LiveScanSession scan)
        {
            if (Interlocked.CompareExchange(ref _capturingPhoto, 1, 0) != 0) return; // previous photo still encoding
            ArKitPhoto? photo = null;
            bool reserved = false;
            try
            {
                reserved = scan.ShouldCapturePhoto(frame.Timestamp);
                photo = reserved ? ArKitReader.TakePhoto(frame) : null;
            }
            catch (Exception ex)
            {
                _message = $"Could not take a photo: {ex.Message}";
            }
            if (photo is null)
            {
                if (reserved) scan.ReleasePhotoReservation();
                Volatile.Write(ref _capturingPhoto, 0);
                return;
            }

            scan.TrackPhotoCapture(Task.Run(() =>
            {
                try
                {
                    var preview = ArKitReader.Preview(photo);
                    if (!scan.AddPhoto(ArKitReader.EncodeJpeg(photo), photo.Metadata, preview))
                        scan.ReleasePhotoReservation();
                }
                catch (Exception ex)
                {
                    _message = $"Could not save a photo: {ex.Message}";
                    scan.ReleasePhotoReservation();
                }
                finally
                {
                    photo.Pixels.Dispose();
                    Volatile.Write(ref _capturingPhoto, 0);
                }
            }));
        }
    }

    /// <summary>Draws each of ARKit's mesh anchors as a wireframe where it stands.</summary>
    private sealed class MeshOverlay : ARSCNViewDelegate
    {
        private static readonly SCNMaterial Material = CreateMaterial();

        public override SCNNode GetNode(ISCNSceneRenderer renderer, ARAnchor anchor) =>
            anchor is ARMeshAnchor mesh ? SCNNode.FromGeometry(Geometry(mesh)) : new SCNNode();

        public override void DidUpdateNode(ISCNSceneRenderer renderer, SCNNode node, ARAnchor anchor)
        {
            if (anchor is ARMeshAnchor mesh) node.Geometry = Geometry(mesh);
        }

        private static SCNGeometry Geometry(ARMeshAnchor anchor)
        {
            var geometry = anchor.Geometry;
            var vertices = ArKitReader.ReadVertices(geometry.Vertices)
                .Select(v => new SCNVector3(v.X, v.Y, v.Z)).ToArray();
            var indices = ArKitReader.ReadTriangles(geometry.Faces);
            var bytes = new byte[indices.Length * sizeof(int)];
            Buffer.BlockCopy(indices, 0, bytes, 0, bytes.Length);
            using var data = NSData.FromArray(bytes);
            var element = SCNGeometryElement.FromData(data, SCNGeometryPrimitiveType.Triangles, indices.Length / 3, sizeof(int));
            var result = SCNGeometry.Create([SCNGeometrySource.FromVertices(vertices)], [element]);
            result.FirstMaterial = Material;
            return result;
        }

        private static SCNMaterial CreateMaterial()
        {
            var material = new SCNMaterial { FillMode = SCNFillMode.Lines, DoubleSided = true };
            material.Diffuse.Contents = UIColor.FromRGBA(0.3f, 0.9f, 1f, 0.6f);
            material.LightingModelName = SCNLightingModel.Constant;
            return material;
        }
    }
}
