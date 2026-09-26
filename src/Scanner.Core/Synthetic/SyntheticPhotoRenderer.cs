using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Synthetic;

/// <summary>
/// Renders grayscale photos of an SDF scene painted with a solid (3D) texture: a surface point has the same shade
/// in every view, as on a real textured object, which is what multi-view stereo relies on.
/// </summary>
public static class SyntheticPhotoRenderer
{
    private const int MaxSteps = 256;
    private const float MaxDistance = 2f;
    private const float HitEpsilon = 1e-5f;

    /// <summary>Renders the photo and its ground-truth depth (metres along the optical axis, 0 where nothing is hit).
    /// The background is black.</summary>
    public static (GrayImage Image, float[] Depth) Render(ISdf sdf, CameraIntrinsics k, Matrix4x4 cameraToWorld,
        Func<Vector3, float>? texture = null)
    {
        texture ??= SolidNoise;
        var pixels = new byte[k.Width * k.Height];
        var depth = new float[k.Width * k.Height];
        var eye = cameraToWorld.Translation;

        Parallel.For(0, k.Height, v =>
        {
            for (int u = 0; u < k.Width; u++)
            {
                var rayCamera = new Vector3((u - k.Cx) / k.Fx, (v - k.Cy) / k.Fy, 1f);
                var direction = Vector3.Normalize(Vector3.TransformNormal(rayCamera, cameraToWorld));
                float t = 0f;
                for (int i = 0; i < MaxSteps && t < MaxDistance; i++)
                {
                    float d = sdf.Distance(eye + direction * t);
                    if (d < HitEpsilon)
                    {
                        int index = v * k.Width + u;
                        depth[index] = t / rayCamera.Length();
                        pixels[index] = (byte)Math.Clamp(MathF.Round(texture(eye + direction * t)), 0, 255);
                        break;
                    }
                    t += d;
                }
            }
        });
        return (new GrayImage(k.Width, k.Height, pixels), depth);
    }

    /// <summary>Smooth value noise at 4, 9 and 20 mm, like print or grain on a surface; shades around mid-gray.</summary>
    public static float SolidNoise(Vector3 p) =>
        128f + 55f * Noise(p / 0.004f) + 40f * Noise(p / 0.009f + new Vector3(17.3f, 5.1f, 9.7f))
             + 30f * Noise(p / 0.02f + new Vector3(3.7f, 11.9f, 23.1f));

    // Trilinear interpolation of pseudo-random lattice values in [-1, 1], with smoothstep weights.
    private static float Noise(Vector3 p)
    {
        int x0 = (int)MathF.Floor(p.X), y0 = (int)MathF.Floor(p.Y), z0 = (int)MathF.Floor(p.Z);
        float fx = Smooth(p.X - x0), fy = Smooth(p.Y - y0), fz = Smooth(p.Z - z0);
        float Lerp(float a, float b, float t) => a + (b - a) * t;
        float x00 = Lerp(Hash(x0, y0, z0), Hash(x0 + 1, y0, z0), fx);
        float x10 = Lerp(Hash(x0, y0 + 1, z0), Hash(x0 + 1, y0 + 1, z0), fx);
        float x01 = Lerp(Hash(x0, y0, z0 + 1), Hash(x0 + 1, y0, z0 + 1), fx);
        float x11 = Lerp(Hash(x0, y0 + 1, z0 + 1), Hash(x0 + 1, y0 + 1, z0 + 1), fx);
        return Lerp(Lerp(x00, x10, fy), Lerp(x01, x11, fy), fz);
    }

    private static float Smooth(float t) => t * t * (3f - 2f * t);

    private static float Hash(int x, int y, int z)
    {
        uint h = unchecked((uint)(x * 73856093) ^ (uint)(y * 19349663) ^ (uint)(z * 83492791));
        h ^= h >> 13;
        h = unchecked(h * 0x5bd1e995);
        h ^= h >> 15;
        return (h & 0xFFFF) / 32767.5f - 1f;
    }
}
