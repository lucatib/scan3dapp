using System.Globalization;
using System.Numerics;
using System.Text;

namespace Scanner.Capture.Sessions;

/// <summary>
/// A triangle mesh textured with patches of the session's own photos: every triangle names the photo it is cut
/// from (<see cref="ScanPhoto.Index"/>, 0 for none) and carries one texture coordinate per corner, normalized to the
/// upright JPEG (u right, v down, 0..1).
/// </summary>
/// <param name="Positions">Vertices, metres, world frame (+Y up).</param>
/// <param name="Indices">Three vertex indices per triangle.</param>
/// <param name="Uvs">One texture coordinate per entry of <paramref name="Indices"/>.</param>
/// <param name="TrianglePhotos">One photo index per triangle.</param>
public sealed record TexturedModel(Vector3[] Positions, int[] Indices, Vector2[] Uvs, int[] TrianglePhotos)
{
    public int TriangleCount => Indices.Length / 3;
}

/// <summary>
/// Reads and writes <see cref="TexturedModel"/> as Wavefront OBJ + MTL in the session folder. The materials point at
/// <c>photos/NNNNNN.jpg</c>, so the folder opens as a textured model in Blender, MeshLab or Fusion as it is.
/// </summary>
public static class TexturedModelFile
{
    public const string Model = "model.obj";
    public const string Materials = "model.mtl";
    private const string Untextured = "untextured";

    public static bool Exists(string directory) => File.Exists(Path.Combine(directory, Model));

    public static void Write(string directory, TexturedModel model)
    {
        var c = CultureInfo.InvariantCulture;
        var obj = new StringBuilder();
        obj.Append("# Scan3D textured model, units: metres\nmtllib ").Append(Materials).Append('\n');
        foreach (var p in model.Positions)
            obj.Append(c, $"v {p.X:R} {p.Y:R} {p.Z:R}\n");
        // OBJ texture v runs up from the bottom of the image.
        foreach (var uv in model.Uvs)
            obj.Append(c, $"vt {uv.X:R} {1 - uv.Y:R}\n");

        var photos = model.TrianglePhotos.Distinct().Order().ToArray();
        foreach (int photo in photos)
        {
            obj.Append("usemtl ").Append(MaterialName(photo)).Append('\n');
            for (int t = 0; t < model.TriangleCount; t++)
            {
                if (model.TrianglePhotos[t] != photo) continue;
                obj.Append('f');
                for (int k = 0; k < 3; k++)
                    obj.Append(c, $" {model.Indices[3 * t + k] + 1}/{3 * t + k + 1}");
                obj.Append('\n');
            }
        }

        var mtl = new StringBuilder();
        foreach (int photo in photos)
        {
            mtl.Append("newmtl ").Append(MaterialName(photo)).Append('\n');
            if (photo > 0) mtl.Append("Kd 1 1 1\nmap_Kd photos/").Append(photo.ToString("D6", c)).Append(".jpg\n\n");
            else mtl.Append("Kd 0.6 0.6 0.6\n\n");
        }

        File.WriteAllText(Path.Combine(directory, Materials), mtl.ToString());
        File.WriteAllText(Path.Combine(directory, Model), obj.ToString());
    }

    /// <summary>Reads a model written by <see cref="Write"/> (not arbitrary OBJ: triangles with v/vt pairs only).</summary>
    public static TexturedModel Read(string directory)
    {
        var positions = new List<Vector3>();
        var textureCoordinates = new List<Vector2>();
        var indices = new List<int>();
        var uvs = new List<Vector2>();
        var photos = new List<int>();
        int photo = 0;
        foreach (var line in File.ReadLines(Path.Combine(directory, Model)))
        {
            var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 0) continue;
            switch (parts[0])
            {
                case "v":
                    positions.Add(new Vector3(Float(parts[1]), Float(parts[2]), Float(parts[3])));
                    break;
                case "vt":
                    textureCoordinates.Add(new Vector2(Float(parts[1]), 1 - Float(parts[2])));
                    break;
                case "usemtl":
                    photo = parts[1].StartsWith("photo_", StringComparison.Ordinal)
                        ? int.Parse(parts[1]["photo_".Length..], CultureInfo.InvariantCulture) : 0;
                    break;
                case "f":
                    if (parts.Length != 4) throw new InvalidDataException("Only triangles are supported.");
                    for (int k = 1; k <= 3; k++)
                    {
                        var pair = parts[k].Split('/');
                        indices.Add(int.Parse(pair[0], CultureInfo.InvariantCulture) - 1);
                        uvs.Add(textureCoordinates[int.Parse(pair[1], CultureInfo.InvariantCulture) - 1]);
                    }
                    photos.Add(photo);
                    break;
            }
        }
        return new TexturedModel(positions.ToArray(), indices.ToArray(), uvs.ToArray(), photos.ToArray());
    }

    private static string MaterialName(int photo) =>
        photo > 0 ? "photo_" + photo.ToString("D6", CultureInfo.InvariantCulture) : Untextured;

    private static float Float(string s) => float.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
}
