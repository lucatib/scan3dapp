namespace Scanner.Capture.Tests;

public class GrayImageTests
{
    [Fact]
    public void Downscale_averages_each_block()
    {
        byte[] pixels =
        [
            0, 4, 10, 10,
            8, 12, 10, 10,
            100, 100, 1, 2,
            100, 100, 3, 5,
        ];

        var small = GrayImage.Downscale(pixels, 4, 4, 2);

        Assert.Equal(2, small.Width);
        Assert.Equal(2, small.Height);
        Assert.Equal(new byte[] { 6, 10, 100, 3 }, small.Pixels);
    }

    // A camera image whose size is not a multiple of the factor loses the partial blocks at the right and bottom.
    [Fact]
    public void Downscale_drops_partial_blocks()
    {
        var small = GrayImage.Downscale(new byte[5 * 3], 5, 3, 2);

        Assert.Equal(2, small.Width);
        Assert.Equal(1, small.Height);
    }

    [Fact]
    public void Constructor_rejects_a_buffer_of_the_wrong_size()
    {
        Assert.Throws<ArgumentException>(() => new GrayImage(3, 3, new byte[8]));
    }
}

public class PhotoViewTests
{
    // Pixel centres sit at integers, so a centre x of the source lands at (x + 0.5) / factor - 0.5.
    [Fact]
    public void Downscale_scales_the_intrinsics_with_the_pixels()
    {
        var view = new PhotoView(new GrayImage(4, 2, new byte[8]), new CameraIntrinsics(4, 2, 10, 12, 1.5f, 0.5f),
            System.Numerics.Matrix4x4.CreateTranslation(1, 2, 3));

        var small = view.Downscale(2);

        Assert.Equal(new CameraIntrinsics(2, 1, 5, 6, 0.5f, 0f), small.Intrinsics);
        Assert.Equal(2, small.Image.Width);
        Assert.Equal(view.CameraToWorld, small.CameraToWorld);
    }
}
