using System.Numerics;

namespace Scanner.Capture.ArCore;

/// <summary>Conversions from ARCore conventions to the pipeline conventions (metres, OpenCV camera, row-vector matrices).</summary>
public static class ArCoreConversions
{
    /// <summary>
    /// Converts an ARCore camera pose (column-major 4x4 from <c>Pose.ToMatrix</c>, OpenGL camera axes:
    /// +X right, +Y up, −Z forward, relative to the image readout) into a camera→world matrix with
    /// OpenCV camera axes (+X right, +Y down, +Z forward). The world frame is unchanged (ARCore world, +Y up).
    /// </summary>
    public static Matrix4x4 CameraToWorldFromGlPose(ReadOnlySpan<float> columnMajor)
    {
        if (columnMajor.Length != 16) throw new ArgumentException("Expected a 4x4 matrix (16 values).", nameof(columnMajor));
        var m = columnMajor;
        // Column j of the GL matrix is row j of a row-vector matrix; flipping camera Y and Z negates rows 2 and 3.
        return new Matrix4x4(
            m[0], m[1], m[2], 0,
            -m[4], -m[5], -m[6], 0,
            -m[8], -m[9], -m[10], 0,
            m[12], m[13], m[14], 1);
    }

    /// <summary>Scales pinhole intrinsics from one image resolution to another with the same aspect ratio
    /// (pixel centres at integer coordinates).</summary>
    public static CameraIntrinsics ScaleIntrinsics(float fx, float fy, float cx, float cy,
        int imageWidth, int imageHeight, int targetWidth, int targetHeight)
    {
        float sx = (float)targetWidth / imageWidth;
        float sy = (float)targetHeight / imageHeight;
        return new CameraIntrinsics(targetWidth, targetHeight,
            fx * sx, fy * sy, (cx + 0.5f) * sx - 0.5f, (cy + 0.5f) * sy - 0.5f);
    }

    /// <summary>Builds a <see cref="DepthFrame"/> from a packed row-major uint16 millimetre depth image (0 = invalid).</summary>
    public static DepthFrame DepthFrameFromMillimeters(ushort[] millimeters, CameraIntrinsics intrinsics,
        Matrix4x4 cameraToWorld, double timestampSeconds)
    {
        if (millimeters.Length != intrinsics.Width * intrinsics.Height)
            throw new ArgumentException("Depth buffer size does not match the intrinsics.", nameof(millimeters));
        var depth = new float[millimeters.Length];
        for (int i = 0; i < depth.Length; i++) depth[i] = millimeters[i] * 0.001f;
        return new DepthFrame(intrinsics, depth, cameraToWorld, timestampSeconds);
    }
}
