using System.Numerics;

namespace Scanner.App.Controls;

/// <summary>Orbit camera around the centre of a point cloud (world +Y up).</summary>
public sealed class OrbitCamera
{
    private const float RotateRadiansPerPixel = 0.008f;
    private const float MaxPitch = 1.5f;

    public Vector3 Target { get; private set; }
    public float Radius { get; private set; } = 0.1f;
    public float Distance { get; private set; } = 0.3f;
    public float Yaw { get; private set; } = 0.6f;
    public float Pitch { get; private set; } = 0.4f;

    /// <summary>Centres the camera on the bounding box of <paramref name="points"/> and resets the orientation.</summary>
    public void Frame(IReadOnlyList<Vector3> points)
    {
        if (points.Count == 0) return;
        var min = new Vector3(float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity);
        foreach (var p in points)
        {
            min = Vector3.Min(min, p);
            max = Vector3.Max(max, p);
        }
        Target = (min + max) / 2;
        Radius = MathF.Max(0.01f, Vector3.Distance(min, max) / 2);
        Distance = Radius * 2.5f;
        Yaw = 0.6f;
        Pitch = 0.4f;
    }

    public void Rotate(float dxPixels, float dyPixels)
    {
        Yaw -= dxPixels * RotateRadiansPerPixel;
        Pitch = Math.Clamp(Pitch + dyPixels * RotateRadiansPerPixel, -MaxPitch, MaxPitch);
    }

    public void Zoom(float scaleFactor)
    {
        if (scaleFactor <= 0) return;
        Distance = Math.Clamp(Distance / scaleFactor, Radius * 0.2f, Radius * 20f);
    }

    /// <summary>View-projection matrix as a column-major float[16] for GL.</summary>
    public float[] ViewProjection(float aspect)
    {
        var direction = new Vector3(MathF.Cos(Pitch) * MathF.Sin(Yaw), MathF.Sin(Pitch), MathF.Cos(Pitch) * MathF.Cos(Yaw));
        var eye = Target + direction * Distance;
        var view = Matrix4x4.CreateLookAt(eye, Target, Vector3.UnitY);
        var projection = Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4, MathF.Max(aspect, 0.01f),
            Distance * 0.01f, Distance + Radius * 10);
        var m = view * projection;
        // A row-vector System.Numerics matrix stored row by row is exactly GL's column-major layout of its transpose.
        return
        [
            m.M11, m.M12, m.M13, m.M14,
            m.M21, m.M22, m.M23, m.M24,
            m.M31, m.M32, m.M33, m.M34,
            m.M41, m.M42, m.M43, m.M44,
        ];
    }
}
