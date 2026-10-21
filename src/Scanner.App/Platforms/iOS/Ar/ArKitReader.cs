using System.Numerics;
using System.Runtime.InteropServices;
using ARKit;
using CoreGraphics;
using CoreImage;
using CoreVideo;
using Foundation;
using ImageIO;
using Scanner.Capture;
using Scanner.Capture.ArCore;
using Scanner.Capture.Sessions;
using Scanner.Core.Meshing;

namespace Scanner.App.iOS.Ar;

/// <summary>A camera image held for encoding: ARKit's own pixel buffer (one at a time, so its pool never runs dry)
/// and the photo's metadata in sensor orientation.</summary>
internal sealed record ArKitPhoto(CVPixelBuffer Pixels, ScanPhotoData Metadata);

/// <summary>
/// Converts ARKit frames to the pipeline's types. ARKit's camera axes are OpenGL's (+X right, +Y up, −Z forward,
/// relative to the landscape sensor image), the same as ARCore's, so <see cref="ArCoreConversions"/> applies.
/// </summary>
internal static class ArKitReader
{
    /// <summary>The app is portrait-only, and ARKit's image is always the landscape sensor image (home button right):
    /// upright is a quarter turn clockwise.</summary>
    private const int UprightRotation = 90;

    private const int PreviewWidth = 960;

    /// <summary>ARKit's confidence levels (low, medium, high) on the 0–255 scale of ARCore's, which the depth filter
    /// uses: medium and high pass its default threshold of 128.</summary>
    private static readonly byte[] ConfidenceScale = [0, 160, 255];

    private static readonly Lazy<CIContext> s_context = new(() => CIContext.Create());

    /// <summary>Column-major elements of a simd matrix, the layout <see cref="ArCoreConversions.CameraToWorldFromGlPose"/> reads.</summary>
    public static float[] ColumnMajor(NMatrix4 m) =>
    [
        m.M11, m.M21, m.M31, m.M41,
        m.M12, m.M22, m.M32, m.M42,
        m.M13, m.M23, m.M33, m.M43,
        m.M14, m.M24, m.M34, m.M44,
    ];

    public static Vector3 Translation(NMatrix4 m) => new(m.M14, m.M24, m.M34);

    public static Matrix4x4 CameraToWorld(ARCamera camera) => ArCoreConversions.CameraToWorldFromGlPose(ColumnMajor(camera.Transform));

    /// <summary>The camera's intrinsics scaled to an image of <paramref name="width"/>×<paramref name="height"/>
    /// with the same field of view (the depth map and the camera image share it).</summary>
    public static CameraIntrinsics Intrinsics(ARCamera camera, int width, int height)
    {
        var k = camera.Intrinsics;
        var size = camera.ImageResolution;
        return ArCoreConversions.ScaleIntrinsics(k.M11, k.M22, k.M13, k.M23, (int)size.Width, (int)size.Height, width, height);
    }

    /// <summary>The LiDAR depth of <paramref name="frame"/> and its confidence, or null before it starts.</summary>
    public static (DepthFrame Frame, byte[] Confidence)? ReadDepth(ARFrame frame)
    {
        using var data = frame.SceneDepth;
        if (data?.DepthMap is not { } depthMap) return null;
        int width = (int)depthMap.Width, height = (int)depthMap.Height;
        var depth = new float[width * height];
        CopyRows(depthMap, 0, width * sizeof(float), height, MemoryMarshal.AsBytes(depth.AsSpan()));

        var confidence = new byte[width * height];
        if (data.ConfidenceMap is { } confidenceMap)
        {
            CopyRows(confidenceMap, 0, width, height, confidence);
            for (int i = 0; i < confidence.Length; i++) confidence[i] = ConfidenceScale[Math.Min(confidence[i], (byte)2)];
        }
        else Array.Fill(confidence, (byte)255);

        var depthFrame = new DepthFrame(Intrinsics(frame.Camera, width, height), depth, CameraToWorld(frame.Camera),
            frame.Timestamp);
        return (depthFrame, confidence);
    }

    /// <summary>Holds on to the camera image of <paramref name="frame"/> for <see cref="Preview"/> and <see cref="EncodeJpeg"/>.</summary>
    public static ArKitPhoto? TakePhoto(ARFrame frame)
    {
        var pixels = frame.CapturedImage;
        if (pixels is null) return null;
        int width = (int)pixels.Width, height = (int)pixels.Height;
        var metadata = new ScanPhotoData(Intrinsics(frame.Camera, width, height), CameraToWorld(frame.Camera),
            frame.Timestamp, UprightRotation);
        return new ArKitPhoto(pixels, metadata);
    }

    /// <summary>The luma plane averaged down to about <see cref="PreviewWidth"/> pixels wide. Worker thread.</summary>
    public static PhotoView Preview(ArKitPhoto photo)
    {
        var pixels = photo.Pixels;
        int width = (int)pixels.GetWidthOfPlane(0), height = (int)pixels.GetHeightOfPlane(0);
        var luma = new byte[width * height];
        CopyRows(pixels, 0, width, height, luma);
        int factor = Math.Max(1, width / PreviewWidth);
        var small = GrayImage.Downscale(luma, width, height, factor);
        var k = photo.Metadata.Intrinsics;
        var intrinsics = new CameraIntrinsics(small.Width, small.Height, k.Fx / factor, k.Fy / factor,
            (k.Cx + 0.5f) / factor - 0.5f, (k.Cy + 0.5f) / factor - 0.5f);
        return new PhotoView(small, intrinsics, photo.Metadata.CameraToWorld);
    }

    /// <summary>The photo as an upright JPEG. Worker thread.</summary>
    public static byte[] EncodeJpeg(ArKitPhoto photo)
    {
        using var image = new CIImage(photo.Pixels);
        using var upright = image.CreateByApplyingOrientation(CGImagePropertyOrientation.Right);
        using var colorSpace = CGColorSpace.CreateSrgb() ?? throw new InvalidOperationException("No sRGB colour space.");
        using var options = new NSDictionary();
        using var jpeg = s_context.Value.GetJpegRepresentation(upright, colorSpace, options)
            ?? throw new InvalidOperationException("The camera image could not be encoded.");
        return jpeg.ToArray();
    }

    /// <summary>The mesh ARKit has built of the room, in world metres.</summary>
    public static List<MeshChunk> ReadMesh(IEnumerable<ARAnchor> anchors)
    {
        var chunks = new List<MeshChunk>();
        foreach (var anchor in anchors.OfType<ARMeshAnchor>())
        {
            var geometry = anchor.Geometry;
            var local = ReadVertices(geometry.Vertices);
            var m = anchor.Transform;
            var world = new Vector3[local.Length];
            for (int i = 0; i < local.Length; i++)
            {
                var v = local[i];
                world[i] = new Vector3(
                    m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z + m.M14,
                    m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z + m.M24,
                    m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z + m.M34);
            }
            chunks.Add(new MeshChunk(world, ReadTriangles(geometry.Faces)));
        }
        return chunks;
    }

    /// <summary>Vertices of a mesh anchor in its own frame (three floats each).</summary>
    public static Vector3[] ReadVertices(ARGeometrySource source)
    {
        int count = (int)source.Count, stride = (int)source.Stride, offset = (int)source.Offset;
        var result = new Vector3[count];
        unsafe
        {
            byte* start = (byte*)source.Buffer.Contents + offset;
            for (int i = 0; i < count; i++)
            {
                float* v = (float*)(start + i * stride);
                result[i] = new Vector3(v[0], v[1], v[2]);
            }
        }
        return result;
    }

    /// <summary>Triangle corner indices of a mesh anchor.</summary>
    public static int[] ReadTriangles(ARGeometryElement faces)
    {
        int count = (int)faces.Count * (int)faces.IndexCountPerPrimitive;
        int bytesPerIndex = (int)faces.BytesPerIndex;
        var result = new int[count];
        unsafe
        {
            byte* start = (byte*)faces.Buffer.Contents;
            for (int i = 0; i < count; i++)
                result[i] = bytesPerIndex == 2 ? ((ushort*)start)[i] : (int)((uint*)start)[i];
        }
        return result;
    }

    /// <summary>Copies <paramref name="height"/> rows of <paramref name="rowBytes"/> from one plane of a pixel buffer,
    /// dropping the row padding.</summary>
    private static void CopyRows(CVPixelBuffer buffer, int plane, int rowBytes, int height, Span<byte> destination)
    {
        buffer.Lock(CVPixelBufferLock.ReadOnly);
        try
        {
            bool planar = buffer.IsPlanar;
            nint address = planar ? buffer.GetBaseAddress(plane) : buffer.BaseAddress;
            int stride = (int)(planar ? buffer.GetBytesPerRowOfPlane(plane) : buffer.BytesPerRow);
            unsafe
            {
                for (int row = 0; row < height; row++)
                    new ReadOnlySpan<byte>((byte*)address + row * stride, rowBytes)
                        .CopyTo(destination.Slice(row * rowBytes, rowBytes));
            }
        }
        finally
        {
            buffer.Unlock(CVPixelBufferLock.ReadOnly);
        }
    }
}
