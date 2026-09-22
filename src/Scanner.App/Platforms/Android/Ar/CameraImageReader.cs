using Android.Graphics;
using Google.AR.Core;
using Google.AR.Core.Exceptions;
using Java.Nio;
using Scanner.Capture.ArCore;
using Scanner.Capture.Sessions;
using AndroidImage = Android.Media.Image;
using ArCamera = Google.AR.Core.Camera; // Android.Graphics.Camera is a different thing entirely
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Ar;

/// <summary>
/// One camera image copied out of ARCore's pool, still in the sensor's own YUV layout.
/// <see cref="Metadata"/> already carries the rotation that <see cref="CameraImageReader.EncodeJpeg"/> will apply.
/// </summary>
internal sealed record CameraImage(byte[] Nv21, int Width, int Height, ScanPhotoData Metadata);

/// <summary>
/// Reads the ARCore CPU camera image and turns it into an upright JPEG.
/// <see cref="TryRead"/> runs on the GL thread and only copies bytes; <see cref="EncodeJpeg"/> does the
/// expensive work and must run on a worker thread.
/// </summary>
/// <remarks>
/// This is the CPU image, not the GPU texture drawn on screen, so its field of view can differ from the
/// preview (commonly 4:3 against a 16:9 texture) and its resolution is whatever the camera config offers —
/// often well below the preview. That is the trade for images that come with their own intrinsics: the same
/// ones ARCore reports for this stream, which is what makes the pictures usable for texturing later rather
/// than only for looking at.
/// </remarks>
internal static class CameraImageReader
{
    private const int JpegQuality = 88;

    /// <summary>Image-normalized (0,0) and (1,0): the ends of the image's own x axis. GL thread only, and read
    /// by ARCore rather than written, so one shared array is safe.</summary>
    private static readonly float[] ImageAxisEnds = [0f, 0f, 1f, 0f];

    /// <summary>Copies the camera image belonging to <paramref name="frame"/>, or null if ARCore has none yet.</summary>
    public static CameraImage? TryRead(ArFrame frame, ArCamera camera, double timestampSeconds)
    {
        AndroidImage? image = null;
        try
        {
            image = frame.AcquireCameraImage()!;
            int width = image.Width;
            int height = image.Height;
            // The 4:2:0 chroma planes are half size in both directions, so odd dimensions have no NV21 encoding.
            // No camera produces them; dropping the photo is still better than writing a skewed one.
            if ((width & 1) != 0 || (height & 1) != 0) return null;

            byte[] nv21 = ToNv21(image, width, height);

            // Sensor-aligned pose and the CPU stream's own intrinsics: both describe the image as read out, which
            // is the orientation nv21 is in. The rotation below is recorded, not applied to either.
            var pose = new float[16];
            camera.Pose!.ToMatrix(pose, 0);
            var imageIntrinsics = camera.ImageIntrinsics!;
            float[] focal = imageIntrinsics.GetFocalLength()!;
            float[] principal = imageIntrinsics.GetPrincipalPoint()!;
            int[] size = imageIntrinsics.GetImageDimensions()!;
            var intrinsics = ArCoreConversions.ScaleIntrinsics(focal[0], focal[1], principal[0], principal[1],
                size[0], size[1], width, height);

            var metadata = new ScanPhotoData(intrinsics, ArCoreConversions.CameraToWorldFromGlPose(pose),
                timestampSeconds, UprightRotation(frame));
            return new CameraImage(nv21, width, height, metadata);
        }
        catch (NotYetAvailableException)
        {
            return null; // normal while the camera stream warms up
        }
        catch (ResourceExhaustedException)
        {
            return null; // ARCore's image pool is busy this frame; the next photo interval tries again
        }
        finally
        {
            Release(image);
        }
    }

    /// <summary>Encodes the image as an upright JPEG. Worker thread only: a JPEG is tens of milliseconds.</summary>
    public static byte[] EncodeJpeg(CameraImage image)
    {
        using var yuv = new YuvImage(image.Nv21, ImageFormatType.Nv21, image.Width, image.Height, null);
        using var stream = new MemoryStream();
        using (var bounds = new Android.Graphics.Rect(0, 0, image.Width, image.Height))
            yuv.CompressToJpeg(bounds, JpegQuality, stream);

        byte[] jpeg = stream.ToArray();
        return image.Metadata.RotationDegrees == 0 ? jpeg : Rotate(jpeg, image.Metadata.RotationDegrees);
    }

    /// <summary>
    /// How far the sensor image has to be turned clockwise to match the screen, asked of ARCore rather than
    /// derived from the sensor orientation and the display rotation: ARCore already composes both to draw the
    /// camera background, so this cannot disagree with what the user was looking at while they scanned.
    /// </summary>
    private static int UprightRotation(ArFrame frame)
    {
        var view = new float[4];
        frame.TransformCoordinates2d(Coordinates2d.ImageNormalized!, ImageAxisEnds, Coordinates2d.ViewNormalized!, view);
        return PhotoOrientation.DegreesFromImageAxis(view[2] - view[0], view[3] - view[1]);
    }

    private static byte[] Rotate(byte[] jpeg, int degreesClockwise)
    {
        using var source = BitmapFactory.DecodeByteArray(jpeg, 0, jpeg.Length)
            ?? throw new InvalidOperationException("The camera image could not be decoded for rotation.");
        using var transform = new Matrix();
        transform.PostRotate(degreesClockwise);
        // Throwing rather than returning the unrotated bytes: the metadata already says the picture is rotated,
        // and a JPEG that silently disagrees with it is worse than no photo at all.
        using var rotated = Bitmap.CreateBitmap(source, 0, 0, source.Width, source.Height, transform, filter: false)
            ?? throw new InvalidOperationException("The camera image could not be rotated.");
        using var stream = new MemoryStream();
        rotated.Compress(Bitmap.CompressFormat.Jpeg!, JpegQuality, stream);
        return stream.ToArray();
    }

    private static byte[] ToNv21(AndroidImage image, int width, int height)
    {
        var planes = image.GetPlanes()!;
        var nv21 = new byte[width * height * 3 / 2];
        CopyPlane(planes[0]!, width, height, nv21, 0, 1);
        // NV21 puts the chroma after the luma as interleaved V, U pairs at half resolution - hence plane 2 first.
        int chromaWidth = width / 2;
        int chromaHeight = height / 2;
        CopyPlane(planes[2]!, chromaWidth, chromaHeight, nv21, width * height, 2);
        CopyPlane(planes[1]!, chromaWidth, chromaHeight, nv21, width * height + 1, 2);
        return nv21;
    }

    /// <summary>Copies one plane, honouring its row and pixel strides: a camera plane is padded more often than not,
    /// and both chroma planes are commonly interleaved already (pixel stride 2).</summary>
    private static void CopyPlane(AndroidImage.Plane plane, int width, int height,
        byte[] destination, int offset, int destinationStride)
    {
        byte[] source = Copy(plane.Buffer!);
        int rowStride = plane.RowStride;
        int pixelStride = plane.PixelStride;

        // The luma plane is usually contiguous within each row, and it is by far the largest: a 1080p one is two
        // million iterations of the loop below, on the GL thread, which is a visible hitch in the camera preview.
        if (destinationStride == 1 && pixelStride == 1)
        {
            for (int v = 0; v < height; v++) Array.Copy(source, v * rowStride, destination, offset + v * width, width);
            return;
        }

        int d = offset;
        for (int v = 0; v < height; v++)
        {
            int row = v * rowStride;
            for (int u = 0; u < width; u++, d += destinationStride)
                destination[d] = source[row + u * pixelStride];
        }
    }

    private static byte[] Copy(ByteBuffer buffer)
    {
        buffer.Rewind();
        var bytes = new byte[buffer.Remaining()];
        buffer.Get(bytes);
        return bytes;
    }

    private static void Release(AndroidImage? image)
    {
        if (image is null) return;
        // Same discipline as DepthFrameReader: Close releases ARCore's pool slot, Dispose only the managed peer,
        // and neither may throw out of a finally and replace the exception in flight.
        try
        {
            image.Close();
        }
        catch (Exception ex)
        {
            Android.Util.Log.Warn("Scan3D", $"Closing the ARCore camera image failed: {ex.Message}");
        }

        try
        {
            image.Dispose();
        }
        catch (Exception ex)
        {
            Android.Util.Log.Warn("Scan3D", $"Disposing the ARCore camera image failed: {ex.Message}");
        }
    }
}
