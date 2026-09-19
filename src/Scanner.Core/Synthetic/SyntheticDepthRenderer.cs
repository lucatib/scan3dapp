using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Synthetic;

/// <summary>Genera mappe di profondità per sphere tracing su una SDF.</summary>
public static class SyntheticDepthRenderer
{
    private const int MaxSteps = 256;
    private const float MaxDistance = 2f;
    private const float HitEpsilon = 1e-5f;

    public static DepthFrame Render(ISdf sdf, CameraIntrinsics k, Matrix4x4 cameraToWorld,
        float noiseSigma = 0f, int seed = 0, double timestampSeconds = 0)
    {
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
                        depth[v * k.Width + u] = t / rayCamera.Length();
                        break;
                    }
                    t += d;
                }
            }
        });

        if (noiseSigma > 0)
        {
            var rng = new Random(seed);
            for (int i = 0; i < depth.Length; i++)
                if (depth[i] > 0) depth[i] += noiseSigma * Gaussian(rng);
        }

        return new DepthFrame(k, depth, cameraToWorld, timestampSeconds);
    }

    private static float Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
