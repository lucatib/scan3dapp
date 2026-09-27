using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

public static class PhotoQuality
{
    /// <summary>A photo whose sharpness is below this fraction of the scan's median is treated as blurred. Motion blur
    /// took the close pass of a book scan to 0.1-0.5 of the median; steady photos stayed within 0.6-1.6.</summary>
    public const float MinRelativeSharpness = 0.5f;

    /// <summary>
    /// Mean squared gradient over the central half of the photo. Motion blur flattens gradients along the motion, so
    /// a blurred photo scores far below a steady one of the same scene. Only comparable between photos of one scan:
    /// the value also grows with how much texture is in view.
    /// </summary>
    public static float Sharpness(GrayImage image)
    {
        double sum = 0;
        int count = 0;
        for (int y = Math.Max(1, image.Height / 4); y < Math.Min(image.Height - 1, 3 * image.Height / 4); y++)
        for (int x = Math.Max(1, image.Width / 4); x < Math.Min(image.Width - 1, 3 * image.Width / 4); x++)
        {
            int gx = image[x + 1, y] - image[x - 1, y], gy = image[x, y + 1] - image[x, y - 1];
            sum += gx * gx + gy * gy;
            count++;
        }
        return count == 0 ? 0 : (float)(sum / count);
    }
}
