using Scanner.Capture;
using Scanner.Capture.Sessions;

namespace Scanner.App.Controls;

/// <summary>A camera photo: its small grayscale copy (sensor orientation, with its own intrinsics) and the full picture
/// on demand. <see cref="Metadata"/>'s pose is the AR session's and means nothing to a turntable scan.</summary>
public sealed record PhotoCapture(PhotoView Preview, ScanPhotoData Metadata, Func<byte[]> EncodeJpeg);

/// <summary>The back camera's live picture, offering a photo a few times a second (Android, through ARCore).</summary>
public sealed class PhotoCameraView : View
{
    internal bool IsRunRequested { get; private set; }

    /// <summary>Raised on a worker thread; the next photo is offered only after the handler returns.</summary>
    public event EventHandler<PhotoCapture>? PhotoTaken;

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

    /// <summary>Raised on the main thread when the camera cannot start, with a sentence for the user.</summary>
    public event EventHandler<string>? Failed;

    internal void Raise(PhotoCapture photo) => PhotoTaken?.Invoke(this, photo);

    internal void RaiseFailed(string message) => Failed?.Invoke(this, message);
}
