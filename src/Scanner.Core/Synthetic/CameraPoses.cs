using System.Numerics;

namespace Scanner.Core.Synthetic;

public static class CameraPoses
{
    /// <summary>Camera→world pose (OpenCV convention) looking at <paramref name="target"/> from <paramref name="eye"/>.</summary>
    public static Matrix4x4 LookAt(Vector3 eye, Vector3 target)
    {
        var forward = Vector3.Normalize(target - eye);
        var up = MathF.Abs(forward.Z) > 0.99f ? Vector3.UnitY : Vector3.UnitZ;
        var right = Vector3.Normalize(Vector3.Cross(forward, up));
        var down = Vector3.Cross(forward, right);
        return new Matrix4x4(
            right.X, right.Y, right.Z, 0,
            down.X, down.Y, down.Z, 0,
            forward.X, forward.Y, forward.Z, 0,
            eye.X, eye.Y, eye.Z, 1);
    }

    /// <summary><paramref name="count"/> poses uniformly distributed on a sphere, all facing the center.</summary>
    public static IReadOnlyList<Matrix4x4> FibonacciSphere(int count, float radius, Vector3 target)
    {
        float golden = MathF.PI * (3f - MathF.Sqrt(5f));
        var poses = new List<Matrix4x4>(count);
        for (int i = 0; i < count; i++)
        {
            float y = 1f - 2f * (i + 0.5f) / count;
            float r = MathF.Sqrt(1f - y * y);
            float phi = golden * i;
            var direction = new Vector3(MathF.Cos(phi) * r, y, MathF.Sin(phi) * r);
            poses.Add(LookAt(target + direction * radius, target));
        }
        return poses;
    }
}
