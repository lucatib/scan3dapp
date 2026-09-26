using System.Numerics;

namespace Scanner.Capture;

/// <summary>8-bit grayscale image, row-major, one byte per pixel (the luma plane of a camera image).</summary>
public sealed class GrayImage
{
    public GrayImage(int width, int height, byte[] pixels)
    {
        if (width <= 0 || height <= 0) throw new ArgumentOutOfRangeException(nameof(width), "The image must not be empty.");
        if (pixels.Length != width * height) throw new ArgumentException("Pixel buffer size does not match the image size.", nameof(pixels));
        Width = width;
        Height = height;
        Pixels = pixels;
    }

    public int Width { get; }
    public int Height { get; }
    public byte[] Pixels { get; }

    public byte this[int x, int y] => Pixels[y * Width + x];

    /// <summary>Averages <paramref name="factor"/>×<paramref name="factor"/> blocks of a larger image; partial blocks at
    /// the right and bottom edges are dropped, which keeps pixel centres on a plain scale of the source grid.</summary>
    public static GrayImage Downscale(ReadOnlySpan<byte> pixels, int width, int height, int factor)
    {
        if (factor < 1) throw new ArgumentOutOfRangeException(nameof(factor));
        if (pixels.Length < width * height) throw new ArgumentException("Pixel buffer is smaller than the image.", nameof(pixels));
        int w = width / factor, h = height / factor, area = factor * factor;
        var result = new byte[w * h];
        for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
        {
            int sum = 0;
            for (int dy = 0; dy < factor; dy++)
            {
                int row = (y * factor + dy) * width + x * factor;
                for (int dx = 0; dx < factor; dx++) sum += pixels[row + dx];
            }
            result[y * w + x] = (byte)((sum + area / 2) / area);
        }
        return new GrayImage(w, h, result);
    }
}

/// <summary>A photo ready for photogrammetry: its pixels, the pinhole intrinsics of exactly those pixels, and the
/// camera→world pose (OpenCV camera axes, metres, row-vector convention).</summary>
public sealed record PhotoView(GrayImage Image, CameraIntrinsics Intrinsics, Matrix4x4 CameraToWorld);
