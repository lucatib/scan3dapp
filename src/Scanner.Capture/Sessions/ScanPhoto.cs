using System.Numerics;

namespace Scanner.Capture.Sessions;

/// <summary>
/// One camera photo taken during a scan. The picture is <c>photos/NNNNNN.jpg</c> and this record sits beside it
/// as <c>photos/NNNNNN.json</c>.
/// </summary>
/// <param name="Index">1-based, and the number in both file names.</param>
/// <param name="TimestampSeconds">Camera frame timestamp, on the same clock as <see cref="DepthFrame.TimestampSeconds"/>,
/// so a photo can be matched to the depth frames taken around it.</param>
/// <param name="Intrinsics">As the camera reported them, for the UNROTATED sensor image. Width and Height are
/// therefore swapped relative to the stored JPEG whenever <see cref="RotationDegrees"/> is 90 or 270; use
/// <see cref="PhotoOrientation"/> to move between the two.</param>
/// <param name="CameraToWorld">Row-major 4x4 with OpenCV camera axes, likewise describing the unrotated sensor image.</param>
/// <param name="RotationDegrees">Clockwise rotation applied to the pixels when the JPEG was written, so that it
/// displays upright. Recording it instead of pre-rotating the intrinsics keeps every number in this record a
/// measurement: the one derived thing is the picture itself, where a mistake is visible to the eye rather than
/// silently skewing a projection nobody looks at.</param>
public sealed record ScanPhoto(
    int Index,
    double TimestampSeconds,
    CameraIntrinsics Intrinsics,
    float[] CameraToWorld,
    int RotationDegrees)
{
    /// <summary>The pose as a matrix. A method rather than a property because it can fail: the array comes
    /// straight out of a JSON file and nothing on the way in guarantees its length.</summary>
    /// <exception cref="InvalidDataException">The stored array is not 16 values long.</exception>
    public Matrix4x4 ToPose()
    {
        float[] m = CameraToWorld;
        if (m.Length != 16) throw new InvalidDataException($"A photo pose needs 16 values, not {m.Length}.");
        return new Matrix4x4(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
    }

    public static float[] Elements(Matrix4x4 m) =>
    [
        m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
        m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44,
    ];
}

/// <summary>
/// Moves pinhole intrinsics and a camera pose between the sensor image and the upright JPEG stored beside them.
/// A quarter turn of the pixels is also a quarter turn of the camera's own X and Y axes, so the two must always
/// be rotated together: rotating only the intrinsics projects onto a transposed image without failing anywhere.
/// </summary>
public static class PhotoOrientation
{
    /// <summary>Intrinsics for the image rotated clockwise by 0, 90, 180 or 270 degrees.</summary>
    public static CameraIntrinsics RotateIntrinsics(CameraIntrinsics k, int degreesClockwise) =>
        Quarters(degreesClockwise) switch
        {
            0 => k,
            // (x, y) -> (height - 1 - y, x): the rows of the source become the columns of the result.
            1 => new CameraIntrinsics(k.Height, k.Width, k.Fy, k.Fx, k.Height - 1 - k.Cy, k.Cx),
            2 => new CameraIntrinsics(k.Width, k.Height, k.Fx, k.Fy, k.Width - 1 - k.Cx, k.Height - 1 - k.Cy),
            _ => new CameraIntrinsics(k.Height, k.Width, k.Fy, k.Fx, k.Cy, k.Width - 1 - k.Cx),
        };

    /// <summary>The same rotation applied to a camera→world pose, so that it still pairs with
    /// <see cref="RotateIntrinsics"/> over the same angle.</summary>
    public static Matrix4x4 RotateCameraToWorld(Matrix4x4 cameraToWorld, int degreesClockwise) =>
        AxisChange(Quarters(degreesClockwise)) * cameraToWorld;

    /// <summary>The rotation to apply to the sensor image so that it displays the way <paramref name="imageAxis"/>
    /// says it is being shown: the direction, in view coordinates (x right, y down), that the image's own +x axis
    /// points on screen. Anything that is not within 45 degrees of an axis returns 0.</summary>
    public static int DegreesFromImageAxis(float viewDx, float viewDy)
    {
        if (MathF.Abs(viewDx) >= MathF.Abs(viewDy)) return viewDx >= 0 ? 0 : 180;
        return viewDy >= 0 ? 90 : 270; // +x running down the screen means the picture needs a clockwise quarter turn
    }

    private static int Quarters(int degreesClockwise)
    {
        if (degreesClockwise % 90 != 0)
            throw new ArgumentOutOfRangeException(nameof(degreesClockwise), degreesClockwise,
                "A photo can only be rotated by a whole number of quarter turns.");
        return ((degreesClockwise / 90) % 4 + 4) % 4;
    }

    // Rows are the rotated camera's axes written in the unrotated camera's frame; row-vector convention, so this
    // pre-multiplies the pose. Image +x runs along camera +X and image +y along camera +Y, so the pixel mapping
    // above fixes these outright: a clockwise quarter turn sends +x to -Y and +y to +X.
    private static Matrix4x4 AxisChange(int quarters) => quarters switch
    {
        0 => Matrix4x4.Identity,
        1 => new Matrix4x4(0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        2 => new Matrix4x4(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        _ => new Matrix4x4(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
    };
}
