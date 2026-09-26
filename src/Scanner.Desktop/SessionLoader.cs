using System.IO.Compression;
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Sessions;
using StbImageSharp;

namespace Scanner.Desktop;

/// <summary>A scan session opened for processing: its folder, manifest, target and decoded photos.</summary>
public sealed record LoadedSession(string Directory, ScanManifest Manifest, Vector3? Target, IReadOnlyList<PhotoView> Photos);

public static class SessionLoader
{
    /// <summary>The session folder for <paramref name="path"/>: the path itself for a folder, or a .scan archive
    /// unzipped into <paramref name="extractTo"/> (a new temporary folder when null).</summary>
    public static string Resolve(string path, string? extractTo = null)
    {
        if (Directory.Exists(path)) return path;
        if (!File.Exists(path)) throw new FileNotFoundException("No session folder or .scan file at this path.", path);
        extractTo ??= Path.Combine(Path.GetTempPath(), "scan3d-" + Path.GetFileNameWithoutExtension(path) + "-" + Guid.NewGuid().ToString("N")[..8]);
        ZipFile.ExtractToDirectory(path, extractTo, overwriteFiles: true);
        return extractTo;
    }

    /// <summary>Reads the manifest and decodes every photo to grayscale, averaged down by <paramref name="downscale"/>.</summary>
    public static LoadedSession Load(string directory, int downscale = 2)
    {
        var manifest = ScanSessionReader.ReadManifest(directory);
        Vector3? target = manifest.Target is { Length: 3 } t ? new Vector3(t[0], t[1], t[2]) : null;
        var photos = ScanSessionReader.ReadPhotos(directory);
        var views = new PhotoView[photos.Count];
        Parallel.For(0, photos.Count, i => views[i] = LoadPhoto(photos[i].Photo, photos[i].ImagePath).Downscale(downscale));
        return new LoadedSession(directory, manifest, target, views);
    }

    /// <summary>
    /// The picture is stored upright while its intrinsics and pose describe the sensor image, so both are turned the
    /// way the pixels were (PhotoOrientation) instead of turning the pixels back: every photo then pairs its pixels
    /// with the camera that took them.
    /// </summary>
    public static PhotoView LoadPhoto(ScanPhoto photo, string imagePath)
    {
        var image = ImageResult.FromMemory(File.ReadAllBytes(imagePath), ColorComponents.Grey);
        var intrinsics = PhotoOrientation.RotateIntrinsics(photo.Intrinsics, photo.RotationDegrees);
        var pose = PhotoOrientation.RotateCameraToWorld(photo.ToPose(), photo.RotationDegrees);
        if (image.Width != intrinsics.Width || image.Height != intrinsics.Height)
            throw new InvalidDataException($"{Path.GetFileName(imagePath)} is {image.Width}x{image.Height} but its metadata "
                                           + $"describes {intrinsics.Width}x{intrinsics.Height} after a {photo.RotationDegrees} degree turn.");
        return new PhotoView(new GrayImage(image.Width, image.Height, image.Data), intrinsics, pose);
    }
}
