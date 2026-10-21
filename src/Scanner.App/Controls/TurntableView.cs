using Scanner.Capture;

namespace Scanner.App.Controls;

/// <summary>
/// One synchronized depth image and camera image from a still depth camera. The camera image is held until
/// <see cref="Release"/>: take the photo (if wanted) with <see cref="EncodeJpeg"/> first, then release it.
/// </summary>
/// <param name="Depth">Metres, in the camera image's own (sensor) orientation; the pose is identity.</param>
/// <param name="PhotoIntrinsics">The intrinsics of the JPEG <see cref="EncodeJpeg"/> writes (not rotated).</param>
public sealed record DepthCapture(DepthFrame Depth, CameraIntrinsics PhotoIntrinsics, Func<byte[]> EncodeJpeg, Action Release);

/// <summary>The front depth camera's live picture (TrueDepth on iPhone), delivering <see cref="DepthCapture"/>s.</summary>
public sealed class TurntableView : View
{
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
