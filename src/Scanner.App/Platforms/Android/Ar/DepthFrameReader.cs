using Google.AR.Core;
using Java.Nio;
using Scanner.Capture;
using Scanner.Capture.ArCore;
using AndroidImage = Android.Media.Image;
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Ar;

/// <summary>Copies ARCore raw depth (uint16 mm) and raw confidence into a <see cref="DepthFrame"/>. GL thread only.</summary>
internal static class DepthFrameReader
{
    /// <summary>
    /// Returns null when no fresh depth image matches this camera frame, or when <paramref name="accept"/>
    /// (called with the frame timestamp in seconds, only for fresh depth) declines it.
    /// </summary>
    /// <exception cref="Google.AR.Core.Exceptions.NotYetAvailableException">Depth is not available yet.</exception>
    public static (DepthFrame Frame, byte[] Confidence)? TryRead(ArFrame frame, Camera camera, Func<double, bool> accept)
    {
        // ARCore images must be closed explicitly: Dispose() only drops the managed peer, and unclosed images
        // exhaust ARCore's image pool (ResourceExhaustedException) after a few frames.
        AndroidImage? depthImage = null;
        AndroidImage? confidenceImage = null;
        try
        {
            depthImage = frame.AcquireRawDepthImage16Bits()!;
            long timestamp = frame.Timestamp;
            if (depthImage.Timestamp != timestamp) return null; // stale depth: its pose would not match this frame
            double seconds = timestamp / 1e9;
            if (!accept(seconds)) return null;

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
            return (depthFrame, confidence);
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
        try
        {
            image.Close();
        }
        finally
        {
            image.Dispose();
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
