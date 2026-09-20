using System.Globalization;
using System.Text.Json;
using Scanner.Capture.Sessions;

namespace Scanner.App.Services;

/// <summary>Scan sessions stored as folders under the app data directory; the folder name is the session id.</summary>
public sealed class SessionStore
{
    /// <summary>Written by ScanSessionWriter.Complete; its presence is what makes a folder a finished session.</summary>
    private const string ManifestFile = "manifest.json";

    public SessionStore() : this(Path.Combine(FileSystem.Current.AppDataDirectory, "sessions"))
    {
    }

    public SessionStore(string root)
    {
        Root = root;
        Directory.CreateDirectory(root);
        Sweep();
    }

    public string Root { get; }

    public (string Id, string Directory) CreateNew()
    {
        string baseId = DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture);
        string id = baseId;
        for (int n = 2; Directory.Exists(DirectoryOf(id)); n++) id = $"{baseId}-{n}";
        return (id, DirectoryOf(id));
    }

    public string DirectoryOf(string id) => Path.Combine(Root, id);

    /// <summary>Completed sessions (those with a manifest), newest first.</summary>
    public IReadOnlyList<ScanManifest> List() =>
        Directory.GetDirectories(Root)
            .Select(TryReadManifest)
            .OfType<ScanManifest>()
            .OrderByDescending(m => m.CreatedUtc)
            .ToList();

    /// <summary>Deletes a session folder on the thread pool: it holds one file per frame, and the caller is the UI thread.</summary>
    public void Delete(string id)
    {
        string directory = DirectoryOf(id);
        _ = Task.Run(() => TryDelete(directory));
    }

    /// <summary>
    /// Drops folders left without a manifest by an abandoned scan (a delete that lost a race with a frame write, or a
    /// process killed mid-scan): they are invisible to <see cref="List"/> and would never be reclaimed otherwise.
    /// Runs on the thread pool; no scan can be in progress while the store is being constructed.
    /// </summary>
    private void Sweep()
    {
        string[] directories;
        try
        {
            directories = Directory.GetDirectories(Root);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            return;
        }

        _ = Task.Run(() =>
        {
            foreach (string directory in directories)
                if (!File.Exists(Path.Combine(directory, ManifestFile)))
                    TryDelete(directory);
        });
    }

    private static void TryDelete(string directory)
    {
        try
        {
            if (Directory.Exists(directory)) Directory.Delete(directory, recursive: true);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            // Best effort: a frame may still be being written. The folder has no manifest, so List() ignores it and
            // the next sweep retries. Swallowed here because this runs detached on the thread pool.
        }
    }

    private static ScanManifest? TryReadManifest(string directory)
    {
        try
        {
            return File.Exists(Path.Combine(directory, ManifestFile))
                ? ScanSessionReader.ReadManifest(directory)
                : null;
        }
        catch (Exception ex) when (ex is IOException or JsonException or InvalidDataException)
        {
            return null;
        }
    }
}
