using AVFoundation;
using CoreFoundation;
using CoreGraphics;
using CoreImage;
using CoreMedia;
using CoreVideo;
using Foundation;
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.Capture;
using Scanner.Capture.ArCore;
using UIKit;

namespace Scanner.App.iOS.Handlers;

/// <summary>
/// The front TrueDepth camera: its picture on screen, and synchronized depth and camera images handed to
/// <see cref="TurntableView"/>. Depth is the camera's unfiltered depth (holes stay holes, rather than being filled
/// with guesses), at the highest resolution the chosen video format offers.
/// </summary>
public sealed class TurntableViewHandler : ViewHandler<TurntableView, TurntableViewHandler.PreviewView>
{
    public static readonly IPropertyMapper<TurntableView, TurntableViewHandler> PropertyMapper =
        new PropertyMapper<TurntableView, TurntableViewHandler>(ViewMapper);

    public static readonly CommandMapper<TurntableView, TurntableViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(TurntableView.Start)] = (handler, _, _) => handler.StartCapture(),
            [nameof(TurntableView.Stop)] = (handler, _, _) => handler.StopCapture(),
        };

    private readonly DispatchQueue _queue = new("scan3d.truedepth");
    private AVCaptureSession? _session;
    private AVCaptureDataOutputSynchronizer? _synchronizer;
    private Receiver? _receiver;

    public TurntableViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    protected override PreviewView CreatePlatformView() => new();

    protected override void ConnectHandler(PreviewView platformView)
    {
        base.ConnectHandler(platformView);
        if (VirtualView.IsRunRequested) StartCapture();
    }

    protected override void DisconnectHandler(PreviewView platformView)
    {
        StopCapture();
        base.DisconnectHandler(platformView);
    }

    private void StartCapture()
    {
        if (_session is { Running: true }) return;
        try
        {
            _session ??= CreateSession();
            var session = _session;
            _queue.DispatchAsync(session.StartRunning); // blocks until the camera runs: never on the main thread
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Scan3D: TrueDepth camera failed: {ex}");
        }
    }

    private void StopCapture()
    {
        var session = _session;
        if (session is not null) _queue.DispatchAsync(session.StopRunning);
    }

    private AVCaptureSession CreateSession()
    {
        var device = AVCaptureDevice.GetDefaultDevice(AVCaptureDeviceType.BuiltInTrueDepthCamera, AVMediaTypes.Video,
                         AVCaptureDevicePosition.Front)
                     ?? throw new NotSupportedException("This device has no TrueDepth camera.");
        var session = new AVCaptureSession();
        session.BeginConfiguration();
        session.SessionPreset = AVCaptureSession.PresetInputPriority;
        var input = AVCaptureDeviceInput.FromDevice(device, out var error)
                    ?? throw new InvalidOperationException(error?.LocalizedDescription ?? "The TrueDepth camera could not be opened.");
        session.AddInput(input);

        // The largest video format up to 1920 wide that comes with float depth; then its largest depth format.
        var format = device.Formats
            .Where(f => f.SupportedDepthDataFormats.Any(IsFloatDepth))
            .Select(f => (Format: f, Size: ((CMVideoFormatDescription)f.FormatDescription).Dimensions))
            .Where(f => f.Size.Width <= 1920)
            .OrderByDescending(f => f.Size.Width * f.Size.Height)
            .Select(f => f.Format)
            .FirstOrDefault() ?? throw new NotSupportedException("The TrueDepth camera offers no depth format.");
        var depthFormat = format.SupportedDepthDataFormats.Where(IsFloatDepth)
            .OrderByDescending(f => ((CMVideoFormatDescription)f.FormatDescription).Dimensions.Width)
            .First();

        var video = new AVCaptureVideoDataOutput
        {
            AlwaysDiscardsLateVideoFrames = true,
            WeakVideoSettings = new CVPixelBufferAttributes { PixelFormatType = CVPixelFormatType.CV420YpCbCr8BiPlanarFullRange }.Dictionary,
        };
        var depth = new AVCaptureDepthDataOutput { FilteringEnabled = false, AlwaysDiscardsLateDepthData = true };
        session.AddOutput(video);
        session.AddOutput(depth);
        depth.ConnectionFromMediaType(AVMediaTypes.DepthData.GetConstant()!)!.Enabled = true;

        if (!device.LockForConfiguration(out error))
            throw new InvalidOperationException(error?.LocalizedDescription ?? "The TrueDepth camera is busy.");
        try
        {
            device.ActiveFormat = format;
            device.ActiveDepthDataFormat = depthFormat;
        }
        finally
        {
            device.UnlockForConfiguration();
        }
        session.CommitConfiguration();

        _receiver = new Receiver(this, video, depth);
        _synchronizer = new AVCaptureDataOutputSynchronizer([video, depth]);
        _synchronizer.SetDelegate(_receiver, _queue);
        PlatformView.Layer.Session = session;
        return session;
    }

    private static bool IsFloatDepth(AVCaptureDeviceFormat f) =>
        f.FormatDescription is CMVideoFormatDescription d && d.MediaSubType == (uint)CVPixelFormatType.DepthFloat32;

    /// <summary>The camera picture, filling the view.</summary>
    public sealed class PreviewView : UIView
    {
        [Export("layerClass")]
        public static ObjCRuntime.Class LayerClass() => new(typeof(AVCaptureVideoPreviewLayer));

        public PreviewView() => Layer.VideoGravity = AVLayerVideoGravity.ResizeAspectFill;

        public new AVCaptureVideoPreviewLayer Layer => (AVCaptureVideoPreviewLayer)base.Layer;
    }

    private sealed class Receiver(TurntableViewHandler handler, AVCaptureVideoDataOutput video, AVCaptureDepthDataOutput depth)
        : AVCaptureDataOutputSynchronizerDelegate
    {
        private static readonly Lazy<CIContext> s_context = new(() => CIContext.Create());
        private readonly WeakReference<TurntableViewHandler> _handler = new(handler);

        public override void DidOutputSynchronizedDataCollection(AVCaptureDataOutputSynchronizer synchronizer,
            AVCaptureSynchronizedDataCollection collection)
        {
            if (!_handler.TryGetTarget(out var handler) || ((IElementHandler)handler).VirtualView is not TurntableView view) return;
            if (collection.GetSynchronizedData(depth) is not AVCaptureSynchronizedDepthData { DepthDataWasDropped: false } depthData
                || collection.GetSynchronizedData(video) is not AVCaptureSynchronizedSampleBufferData { SampleBufferWasDropped: false } videoData
                || videoData.SampleBuffer.GetImageBuffer() is not CVPixelBuffer pixels)
                return;

            var data = depthData.DepthData.ConvertToDepthDataType(CVPixelFormatType.DepthFloat32);
            if (data.CameraCalibrationData is not { } calibration) return;
            var map = data.DepthDataMap;
            int width = (int)map.Width, height = (int)map.Height;
            var k = calibration.IntrinsicMatrix;
            var reference = calibration.IntrinsicMatrixReferenceDimensions;
            var depthIntrinsics = ArCoreConversions.ScaleIntrinsics(k.M11, k.M22, k.M13, k.M23,
                (int)reference.Width, (int)reference.Height, width, height);
            var photoIntrinsics = ArCoreConversions.ScaleIntrinsics(k.M11, k.M22, k.M13, k.M23,
                (int)reference.Width, (int)reference.Height, (int)pixels.Width, (int)pixels.Height);

            var metres = new float[width * height];
            map.Lock(CVPixelBufferLock.ReadOnly);
            try
            {
                int stride = (int)map.BytesPerRow;
                unsafe
                {
                    byte* start = (byte*)map.BaseAddress;
                    for (int row = 0; row < height; row++)
                    {
                        var source = new ReadOnlySpan<float>(start + row * stride, width);
                        for (int u = 0; u < width; u++)
                        {
                            float d = source[u];
                            metres[row * width + u] = float.IsFinite(d) && d > 0 ? d : 0;
                        }
                    }
                }
            }
            finally
            {
                map.Unlock(CVPixelBufferLock.ReadOnly);
            }

            double seconds = depthData.Timestamp.Seconds;
            var frame = new DepthFrame(depthIntrinsics, metres, System.Numerics.Matrix4x4.Identity, seconds);
            // Held for the page: encoding happens on its worker, and only for the frames it keeps a photo of.
            var held = pixels;
            view.Raise(new DepthCapture(frame, photoIntrinsics, () => EncodeJpeg(held), held.Dispose));
        }

        private static byte[] EncodeJpeg(CVPixelBuffer pixels)
        {
            using var image = new CIImage(pixels);
            using var colorSpace = CGColorSpace.CreateSrgb() ?? throw new InvalidOperationException("No sRGB colour space.");
            using var options = new NSDictionary();
            using var jpeg = s_context.Value.GetJpegRepresentation(image, colorSpace, options)
                             ?? throw new InvalidOperationException("The camera image could not be encoded.");
            return jpeg.ToArray();
        }
    }
}
