using System.Globalization;
using System.Text.Json;
using Scanner.Capture.Sessions;

namespace Scanner.App.Services;

/// <summary>Scan sessions stored as folders under the app data directory; the folder name is the session id.</summary>
public sealed class SessionStore
{
    public SessionStore() : this(Path.Combine(FileSystem.Current.AppDataDirectory, "sessions"))
    {
    }

    public SessionStore(string root)
    {
        Root = root;
        Directory.CreateDirectory(root);
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

    public void Delete(string id)
    {
        try
        {
            Directory.Delete(DirectoryOf(id), recursive: true);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            // Best effort: a frame may still be being written; the folder has no manifest and is ignored by List().
        }
    }

    private static ScanManifest? TryReadManifest(string directory)
    {
        try
        {
            return File.Exists(Path.Combine(directory, "manifest.json")) ? ScanSessionReader.ReadManifest(directory) : null;
        }
        catch (Exception ex) when (ex is IOException or JsonException or InvalidDataException)
        {
            return null;
        }
    }
}
