namespace Scanner.Capture;

/// <summary>Pinhole intrinsics in pixels, OpenCV convention (x right, y down, z forward).</summary>
public readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy);
