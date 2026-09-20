using Google.AR.Core;
using Google.AR.Core.Exceptions;
using Java.Nio;
using Scanner.Capture;
using Scanner.Capture.ArCore;
using AndroidImage = Android.Media.Image;
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Ar;

/// <summary>Why one attempt to read depth for a camera frame did or did not produce a frame.</summary>
internal enum DepthReadOutcome
{
    /// <summary>A depth image was read and back-projected.</summary>
    Read,

    /// <summary>ARCore has no depth image yet (normal while its depth estimate warms up).</summary>
    NotAvailable,

    /// <summary>The depth image is the one already consumed for an earlier camera frame.</summary>
    Stale,

    /// <summary>Fresh depth that the integration throttle declined.</summary>
    Declined,
}

/// <summary>Result of <see cref="DepthFrameReader.Read"/>; <see cref="Frame"/> is set only for <see cref="DepthReadOutcome.Read"/>.</summary>
internal readonly record struct DepthRead(DepthReadOutcome Outcome, DepthFrame? Frame, byte[]? Confidence);

/// <summary>Copies ARCore raw depth (uint16 mm) and raw confidence into a <see cref="DepthFrame"/>. GL thread only.</summary>
internal sealed class DepthFrameReader
{
    private const string LogTag = "Scan3D";

    /// <summary>Timestamp of the depth image last consumed; ARCore timestamps are positive, so -1 means "none yet".</summary>
    private long _lastDepthTimestamp = -1;

    /// <summary>
    /// Reads the depth image belonging to <paramref name="frame"/>, if it is a new one and
    /// <paramref name="accept"/> (called with the frame timestamp in seconds, only for fresh depth) accepts it.
    /// </summary>
    public DepthRead Read(ArFrame frame, Camera camera, Func<double, bool> accept)
    {
        // ARCore images must be closed explicitly: Dispose() only drops the managed peer, and unclosed images
        // exhaust ARCore's image pool (ResourceExhaustedException) after a few frames.
        AndroidImage? depthImage = null;
        AndroidImage? confidenceImage = null;
        try
        {
            depthImage = frame.AcquireRawDepthImage16Bits()!;
            long timestamp = frame.Timestamp;
            long depthTimestamp = depthImage.Timestamp;

            // Fast path: the depth image carries this camera frame's timestamp, so the pose below is exactly its own.
            // Fallback: ARCore documents freshness as "differs from the previously acquired depth image", which is
            // the only test that holds on a device whose raw depth images do not carry the camera frame timestamp.
            // Without it the equality alone would reject every frame and the scan would record nothing, silently.
            if (depthTimestamp != timestamp && depthTimestamp == _lastDepthTimestamp)
                return new DepthRead(DepthReadOutcome.Stale, null, null);
            _lastDepthTimestamp = depthTimestamp;

            // The frame timestamp, not the depth one: it is the timestamp of the camera pose used below.
            double seconds = timestamp / 1e9;
            if (!accept(seconds)) return new DepthRead(DepthReadOutcome.Declined, null, null);

            confidenceImage = frame.AcquireRawDepthConfidenceImage()!;
            int width = depthImage.Width;
            int height = depthImage.Height;
            ushort[] millimeters = ReadUInt16(depthImage, width, height);
            byte[] confidence = ReadBytes(confidenceImage, width, height);

            // Sensor-aligned camera pose (not the display-oriented one): the depth image is in sensor orientation.
            var pose = new float[16];
            camera.Pose!.ToMatrix(pose, 0);

            // The depth image covers the same field of view as the GPU camera texture (ARCore maps depth pixels
            // through TEXTURE_NORMALIZED coordinates), so its intrinsics are the texture intrinsics scaled to the
            // depth resolution. The CPU image intrinsics can have a different field of view (e.g. 4:3 vs 16:9).
            var textureIntrinsics = camera.TextureIntrinsics!;
            float[] focal = textureIntrinsics.GetFocalLength()!;
            float[] principal = textureIntrinsics.GetPrincipalPoint()!;
            int[] size = textureIntrinsics.GetImageDimensions()!;
            var intrinsics = ArCoreConversions.ScaleIntrinsics(focal[0], focal[1], principal[0], principal[1],
                size[0], size[1], width, height);

            var depthFrame = ArCoreConversions.DepthFrameFromMillimeters(millimeters, intrinsics,
                ArCoreConversions.CameraToWorldFromGlPose(pose), seconds);
            return new DepthRead(DepthReadOutcome.Read, depthFrame, confidence);
        }
        catch (NotYetAvailableException)
        {
            return new DepthRead(DepthReadOutcome.NotAvailable, null, null);
        }
        finally
        {
            Release(confidenceImage);
            Release(depthImage);
        }
    }

    private static void Release(AndroidImage? image)
    {
        if (image is null) return;
        // Release runs in a finally: neither step may throw, or it would replace the exception in flight.
        try
        {
            image.Close();
        }
        catch (Exception ex)
        {
            Android.Util.Log.Warn(LogTag, $"Closing an ARCore image failed: {ex.Message}");
        }

        try
        {
            image.Dispose();
        }
        catch (Exception ex)
        {
            Android.Util.Log.Warn(LogTag, $"Disposing an ARCore image failed: {ex.Message}");
        }
    }

    private static ushort[] ReadUInt16(AndroidImage image, int width, int height)
    {
        var plane = image.GetPlanes()![0]!;
        byte[] bytes = Copy(plane.Buffer!);
        int rowStride = plane.RowStride;
        int pixelStride = plane.PixelStride;
        var result = new ushort[width * height];
        for (int v = 0; v < height; v++)
        for (int u = 0; u < width; u++)
        {
            int i = v * rowStride + u * pixelStride;
            result[v * width + u] = (ushort)(bytes[i] | (bytes[i + 1] << 8)); // little-endian uint16 millimetres
        }
        return result;
    }

    private static byte[] ReadBytes(AndroidImage image, int width, int height)
    {
        var plane = image.GetPlanes()![0]!;
        byte[] bytes = Copy(plane.Buffer!);
        int rowStride = plane.RowStride;
        int pixelStride = plane.PixelStride;
        var result = new byte[width * height];
        for (int v = 0; v < height; v++)
        for (int u = 0; u < width; u++)
            result[v * width + u] = bytes[v * rowStride + u * pixelStride];
        return result;
    }

    private static byte[] Copy(ByteBuffer buffer)
    {
        buffer.Rewind();
        var bytes = new byte[buffer.Remaining()];
        buffer.Get(bytes);
        return bytes;
    }
}
