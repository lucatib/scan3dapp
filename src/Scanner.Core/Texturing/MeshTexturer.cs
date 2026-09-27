using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Sessions;
using Scanner.Core.Meshing;
using Scanner.Core.Photogrammetry;

namespace Scanner.Core.Texturing;

/// <summary>A photo as a texture source: its session index and the camera of the upright JPEG.</summary>
public sealed record TextureCamera(int PhotoIndex, CameraIntrinsics Intrinsics, Matrix4x4 CameraToWorld)
{
    /// <summary>The camera of the stored, upright picture: the recorded sensor camera turned the way its pixels were.</summary>
    public static TextureCamera FromPhoto(ScanPhoto photo) => new(photo.Index,
        PhotoOrientation.RotateIntrinsics(photo.Intrinsics, photo.RotationDegrees),
        PhotoOrientation.RotateCameraToWorld(photo.ToPose(), photo.RotationDegrees));

    /// <summary>The camera of the stored picture for a photo whose pose was refined: <paramref name="sensorView"/> is
    /// in sensor orientation, at any resolution (texture coordinates are normalized).</summary>
    public static TextureCamera FromView(ScanPhoto photo, PhotoView sensorView) => new(photo.Index,
        PhotoOrientation.RotateIntrinsics(sensorView.Intrinsics, photo.RotationDegrees),
        PhotoOrientation.RotateCameraToWorld(sensorView.CameraToWorld, photo.RotationDegrees));
}

/// <summary>
/// Cuts every triangle's texture from the one photo that shows it best: it must face that camera, lie inside its
/// picture and not be hidden behind other parts of the mesh there; among those, the photo seeing it most head-on and
/// from closest wins (the most texture pixels per square millimetre).
/// </summary>
public static class MeshTexturer
{
    /// <summary>Resolution of the visibility buffers, pixels across the longer side.</summary>
    private const int BufferSize = 320;

    /// <summary>Triangles seen at a grazing angle smear their photo; below this cosine a camera is not used.</summary>
    private const float MinFacing = 0.25f;

    public static TexturedModel Texture(TriangleMesh mesh, IReadOnlyList<TextureCamera> cameras)
    {
        int triangles = mesh.TriangleCount;
        var positions = mesh.Positions.ToArray();
        var indices = mesh.Indices.ToArray();
        var buffers = new float[cameras.Count][];
        Parallel.For(0, cameras.Count, c => buffers[c] = DepthBuffer(positions, indices, cameras[c]));

        var photos = new int[triangles];
        var uvs = new Vector2[indices.Length];
        Parallel.For(0, triangles, t =>
        {
            int a = indices[3 * t], b = indices[3 * t + 1], d = indices[3 * t + 2];
            var centroid = (positions[a] + positions[b] + positions[d]) / 3;
            var normal = mesh.Normals[a] + mesh.Normals[b] + mesh.Normals[d];
            if (normal.LengthSquared() < 1e-12f) return;
            normal = Vector3.Normalize(normal);

            int best = -1;
            float bestScore = 0;
            for (int c = 0; c < cameras.Count; c++)
            {
                var camera = cameras[c];
                var toCamera = camera.CameraToWorld.Translation - centroid;
                float distance = toCamera.Length();
                float facing = Vector3.Dot(normal, toCamera / distance);
                if (facing < MinFacing) continue;
                if (!Inside(camera, positions[a]) || !Inside(camera, positions[b]) || !Inside(camera, positions[d])) continue;
                if (!Visible(buffers[c], camera, centroid)) continue;
                float score = facing * camera.Intrinsics.Fx / distance;
                if (score > bestScore)
                {
                    bestScore = score;
                    best = c;
                }
            }
            if (best < 0) return;
            var chosen = cameras[best];
            photos[t] = chosen.PhotoIndex;
            for (int k = 0; k < 3; k++)
                uvs[3 * t + k] = Uv(chosen, positions[indices[3 * t + k]]);
        });
        return new TexturedModel(positions, indices, uvs, photos);
    }

    private static Vector2 Uv(TextureCamera camera, Vector3 world)
    {
        var k = camera.Intrinsics;
        Pinhole.Project(k, Pinhole.ToCamera(camera.CameraToWorld, world), out float u, out float v);
        return new Vector2((u + 0.5f) / k.Width, (v + 0.5f) / k.Height);
    }

    private static bool Inside(TextureCamera camera, Vector3 world)
    {
        var k = camera.Intrinsics;
        return Pinhole.Project(k, Pinhole.ToCamera(camera.CameraToWorld, world), out float u, out float v)
               && u >= 0 && v >= 0 && u <= k.Width - 1 && v <= k.Height - 1;
    }

    private static float Scale(CameraIntrinsics k) => (float)BufferSize / Math.Max(k.Width, k.Height);

    /// <summary>The nearest mesh depth per buffer pixel, rasterizing every triangle at reduced resolution.</summary>
    private static float[] DepthBuffer(Vector3[] positions, int[] indices, TextureCamera camera)
    {
        var k = camera.Intrinsics;
        float s = Scale(k);
        int w = Math.Max(1, (int)(k.Width * s)), h = Math.Max(1, (int)(k.Height * s));
        var depth = new float[w * h];
        Array.Fill(depth, float.PositiveInfinity);
        var projected = new Vector3[positions.Length];
        for (int i = 0; i < positions.Length; i++)
        {
            var p = Pinhole.ToCamera(camera.CameraToWorld, positions[i]);
            projected[i] = p.Z > 1e-4f
                ? new Vector3((k.Fx * p.X / p.Z + k.Cx) * s, (k.Fy * p.Y / p.Z + k.Cy) * s, p.Z)
                : new Vector3(float.NaN);
        }
        for (int t = 0; t < indices.Length; t += 3)
        {
            var p0 = projected[indices[t]];
            var p1 = projected[indices[t + 1]];
            var p2 = projected[indices[t + 2]];
            if (float.IsNaN(p0.X) || float.IsNaN(p1.X) || float.IsNaN(p2.X)) continue;
            int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(p0.X, MathF.Min(p1.X, p2.X))));
            int x1 = Math.Min(w - 1, (int)MathF.Ceiling(MathF.Max(p0.X, MathF.Max(p1.X, p2.X))));
            int y0 = Math.Max(0, (int)MathF.Floor(MathF.Min(p0.Y, MathF.Min(p1.Y, p2.Y))));
            int y1 = Math.Min(h - 1, (int)MathF.Ceiling(MathF.Max(p0.Y, MathF.Max(p1.Y, p2.Y))));
            // Triangles are a few buffer pixels at most: fill their bounding box with the nearest corner depth.
            float z = MathF.Min(p0.Z, MathF.Min(p1.Z, p2.Z));
            for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (z < depth[y * w + x]) depth[y * w + x] = z;
        }
        return depth;
    }

    private static bool Visible(float[] buffer, TextureCamera camera, Vector3 world)
    {
        var k = camera.Intrinsics;
        float s = Scale(k);
        int w = Math.Max(1, (int)(k.Width * s)), h = Math.Max(1, (int)(k.Height * s));
        var p = Pinhole.ToCamera(camera.CameraToWorld, world);
        if (!Pinhole.Project(k, p, out float u, out float v)) return false;
        int x = Math.Clamp((int)(u * s), 0, w - 1), y = Math.Clamp((int)(v * s), 0, h - 1);
        return p.Z <= buffer[y * w + x] + MathF.Max(0.005f, 0.02f * p.Z);
    }
}
