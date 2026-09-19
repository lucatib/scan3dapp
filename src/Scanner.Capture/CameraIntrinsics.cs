namespace Scanner.Capture;

/// <summary>Intrinseci pinhole in pixel, convenzione OpenCV (x destra, y giù, z avanti).</summary>
public readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy);
