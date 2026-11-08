using Scanner.Capture;

namespace Scanner.App.Controls;

/// <summary>
/// One synchronized depth image and camera image from a still depth camera. The camera image is held until
/// <see cref="Release"/>: take the photo (if wanted) with <see cref="EncodeJpeg"/> first, then release it.
/// </summary>
/// <param name="Depth">Metres, in the camera image's own (sensor) orientation; the pose is identity.</param>
/// <param name="PhotoIntrinsics">The intrinsics of the JPEG <see cref="EncodeJpeg"/> writes (not rotated).</param>
/// <param name="Preview">The camera image's luma, averaged down to about 960 pixels wide, with its intrinsics (the pose
/// is identity): for finding the turntable's dot ring. Call it before <see cref="Release"/>.</param>
public sealed record DepthCapture(DepthFrame Depth, CameraIntrinsics PhotoIntrinsics, Func<byte[]> EncodeJpeg,
    Func<PhotoView> Preview, Action Release);

/// <summary>Which depth camera a <see cref="TurntableView"/> opens.</summary>
public enum DepthCamera
{
    /// <summary>The front TrueDepth camera: the screen faces the piece.</summary>
    FrontTrueDepth,

    /// <summary>The rear wide camera with the LiDAR: the back of the phone faces the piece.</summary>
    BackLidar,
}

/// <summary>A still depth camera's live picture (TrueDepth or LiDAR on iPhone), delivering <see cref="DepthCapture"/>s.</summary>
public sealed class TurntableView : View
{
    /// <summary>The camera to open; set it before the view is shown.</summary>
    public DepthCamera Camera { get; init; } = DepthCamera.FrontTrueDepth;

    internal bool IsRunRequested { get; private set; }

    /// <summary>Raised on the capture thread. A handler that is still busy with the previous capture should release
    /// this one at once: the camera holds only a few images.</summary>
    public event EventHandler<DepthCapture>? Captured;

    public void Start()
    {
        IsRunRequested = true;
        Handler?.Invoke(nameof(Start));
    }

    public void Stop()
    {
        IsRunRequested = false;
        Handler?.Invoke(nameof(Stop));
    }

    internal void Raise(DepthCapture capture)
    {
        if (Captured is { } handler) handler(this, capture);
        else capture.Release();
    }
}
