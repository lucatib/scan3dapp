# Android Scanner App (point cloud + 3D preview) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put an Android app in the user's hands that scans a physical piece with ARCore depth, accumulates a live point cloud shown over the camera feed, isolates the piece from the table, shows it in an interactive 3D preview, saves the session, and exports PLY / `.scan`.

**Architecture:** Platform-independent capture logic lives in `Scanner.Capture` (pure .NET, unit-tested on PC): ARCore→OpenCV pose conversion, depth back-projection, voxel point accumulation, object isolation, session storage, PLY I/O. The MAUI app (`Scanner.App`, TFMs `net10.0-android` and `net10.0-windows10.0.19041.0`) hosts two native Android `GLSurfaceView`s through MAUI handlers: the AR scan view (camera background + point overlay, drives the ARCore session) and the 3D point-cloud viewer (orbit camera). Rendering uses Android's GLES 3.0 API directly.

**Tech Stack:** .NET 10, .NET MAUI, ARCore .NET binding (chosen by the spike, see `docs/arcore-binding-notes.md`), Android GLES30, xUnit.

**Spec:** `docs/superpowers/specs/2026-09-19-scan3d-design.md` (sections 3, 7). Scope for this plan was narrowed by the user on 2026-09-19: Android first, Windows builds but only lists sessions; the 3D preview on Android uses native GLES instead of Silk.NET (Silk.NET stays the plan for the desktop viewer).

## Global Constraints

- All code, comments, docs, UI strings and commit messages in **English**.
- Libraries `net10.0`; app TFMs `net10.0-android;net10.0-windows10.0.19041.0`. `Directory.Build.props` enforces Nullable, ImplicitUsings, TreatWarningsAsErrors.
- `Scanner.Capture` has **no** platform or MAUI dependency.
- Internal units: **metres**. World frame = ARCore world (**+Y up**, gravity aligned). Camera frame for `DepthFrame.CameraToWorld` = OpenCV (+X right, +Y down, +Z forward); `Matrix4x4` is row-vector (`Vector3.Transform(p, m)`).
- Depth: ARCore raw depth (uint16 millimetres) + raw confidence (byte 0–255) when available; smoothed depth otherwise.
- Capture defaults: depth integration at ~5 Hz, depth range 0.10–1.50 m, min confidence 128, point voxel 5 mm, scan region radius 0.30 m around the target.
- Work on `main`; stage explicit paths only (never `git add -A`); every commit ends with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- The app project is **not** in `Scan3D.slnx` until Task 8; build it by path: `dotnet build src/Scanner.App -f net10.0-android`.

## File Structure

```
src/Scanner.Capture/
  ArCore/ArCoreConversions.cs        pose (column-major GL) → OpenCV camera→world; intrinsics scaling; mm → DepthFrame
  PointClouds/DepthBackProjector.cs  DepthFrame (+confidence) → world points, with DepthFilter / ScanRegion
  PointClouds/VoxelPointAccumulator.cs  thread-safe voxel-averaged cloud
  PointClouds/ObjectIsolator.cs      remove support plane, keep cluster nearest the target
  PointClouds/Ply.cs                 PlyWriter / PlyReader (binary little-endian xyz)
  Sessions/FrameCodec.cs             binary frame file format
  Sessions/ScanSession.cs            ScanManifest, ScanSessionWriter, ScanSessionReader, ScanArchive
tests/Scanner.Capture.Tests/          one test file per component
src/Scanner.App/
  Platforms/Android/Ar/…             ARCore session host, camera background renderer, point renderer, AR GLSurfaceView
  Platforms/Android/Viewer/…         orbit camera + point-cloud GLSurfaceView
  Controls/ArScanView.cs, Controls/PointCloudView.cs   cross-platform MAUI views (+ handlers)
  Pages/SessionsPage, ScanPage, PreviewPage (+ view models)
  Services/SessionStore.cs           session folders under app data
```

---

### Task 1: Spike — MAUI app skeleton + ARCore binding

Done by the spike agent (commit on `main`, notes in `docs/arcore-binding-notes.md`). Later tasks MUST use the exact API names recorded in that notes file.

---

### Task 2: Pose/intrinsics conversion, depth back-projection, voxel accumulator, PLY

**Files:**
- Create: `tests/Scanner.Capture.Tests/Scanner.Capture.Tests.csproj` (via CLI)
- Create: `src/Scanner.Capture/ArCore/ArCoreConversions.cs`
- Create: `src/Scanner.Capture/PointClouds/DepthBackProjector.cs`, `VoxelPointAccumulator.cs`, `Ply.cs`
- Test: `tests/Scanner.Capture.Tests/ArCoreConversionsTests.cs`, `DepthBackProjectorTests.cs`, `VoxelPointAccumulatorTests.cs`, `PlyTests.cs`

**Interfaces:**
- Consumes: `CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy)`, `DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)` with `DepthAt(u, v)` (already in `Scanner.Capture`).
- Produces:
  - `static Matrix4x4 ArCoreConversions.CameraToWorldFromGlPose(ReadOnlySpan<float> columnMajor)`
  - `static CameraIntrinsics ArCoreConversions.ScaleIntrinsics(float fx, float fy, float cx, float cy, int imageWidth, int imageHeight, int targetWidth, int targetHeight)`
  - `static DepthFrame ArCoreConversions.DepthFrameFromMillimeters(ushort[] millimeters, CameraIntrinsics intrinsics, Matrix4x4 cameraToWorld, double timestampSeconds)`
  - `readonly record struct ScanRegion(Vector3 Center, float Radius)` with `bool Contains(Vector3 p)`
  - `sealed record DepthFilter(float MinDepth = 0.10f, float MaxDepth = 1.50f, byte MinConfidence = 128, ScanRegion? Region = null)`
  - `static void DepthBackProjector.Project(DepthFrame frame, byte[]? confidence, DepthFilter filter, List<Vector3> output)`
  - `sealed class VoxelPointAccumulator(float voxelSize)`: `float VoxelSize`, `int CellCount`, `void AddRange(IEnumerable<Vector3> points)`, `Vector3[] Snapshot(int minObservations = 1)`, `void Clear()` — thread-safe.
  - `static class PlyWriter { void Write(Stream stream, IReadOnlyList<Vector3> points); }`, `static class PlyReader { Vector3[] Read(Stream stream); }`

- [ ] **Step 1: Create the test project**

```bash
dotnet new xunit -n Scanner.Capture.Tests -o tests/Scanner.Capture.Tests -f net10.0
rm tests/Scanner.Capture.Tests/UnitTest1.cs
dotnet sln add tests/Scanner.Capture.Tests
dotnet add tests/Scanner.Capture.Tests reference src/Scanner.Capture
```

Verify the csproj has `<Using Include="Xunit" />`.

- [ ] **Step 2: Write the failing tests**

`tests/Scanner.Capture.Tests/ArCoreConversionsTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.ArCore;

namespace Scanner.Capture.Tests;

public class ArCoreConversionsTests
{
    private static readonly float[] Identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

    [Fact]
    public void Identity_gl_pose_looks_down_negative_world_z_with_image_down_as_world_down()
    {
        var m = ArCoreConversions.CameraToWorldFromGlPose(Identity);

        Assert.Equal(-Vector3.UnitZ, Vector3.TransformNormal(Vector3.UnitZ, m));
        Assert.Equal(-Vector3.UnitY, Vector3.TransformNormal(Vector3.UnitY, m));
        Assert.Equal(Vector3.UnitX, Vector3.TransformNormal(Vector3.UnitX, m));
    }

    [Fact]
    public void Translation_is_preserved()
    {
        float[] pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1];

        var m = ArCoreConversions.CameraToWorldFromGlPose(pose);

        Assert.Equal(new Vector3(1, 2, 3), Vector3.Transform(Vector3.Zero, m));
    }

    [Fact]
    public void Camera_rotated_90_degrees_about_y_looks_down_negative_world_x()
    {
        // Column-major Ry(+90°): columns (0,0,-1), (0,1,0), (1,0,0).
        float[] pose = [0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1];

        var m = ArCoreConversions.CameraToWorldFromGlPose(pose);

        Assert.True(Vector3.Distance(-Vector3.UnitX, Vector3.TransformNormal(Vector3.UnitZ, m)) < 1e-6f);
    }

    [Fact]
    public void Wrong_length_throws()
    {
        Assert.Throws<ArgumentException>(() => ArCoreConversions.CameraToWorldFromGlPose(new float[12]));
    }

    [Fact]
    public void Intrinsics_scale_to_depth_resolution_with_pixel_center_convention()
    {
        var k = ArCoreConversions.ScaleIntrinsics(500f, 500f, 319.5f, 239.5f, 640, 480, 160, 120);

        Assert.Equal(160, k.Width);
        Assert.Equal(120, k.Height);
        Assert.Equal(125f, k.Fx, 4);
        Assert.Equal(125f, k.Fy, 4);
        Assert.Equal(79.5f, k.Cx, 4);
        Assert.Equal(59.5f, k.Cy, 4);
    }

    [Fact]
    public void Millimeters_become_meters_and_zero_stays_invalid()
    {
        var k = new CameraIntrinsics(2, 1, 1, 1, 0, 0);

        var frame = ArCoreConversions.DepthFrameFromMillimeters([1500, 0], k, Matrix4x4.Identity, 2.5);

        Assert.Equal(1.5f, frame.Depth[0], 6);
        Assert.Equal(0f, frame.Depth[1]);
        Assert.Equal(2.5, frame.TimestampSeconds);
    }
}
```

`tests/Scanner.Capture.Tests/DepthBackProjectorTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class DepthBackProjectorTests
{
    private static readonly CameraIntrinsics K = new(4, 3, 2f, 2f, 1.5f, 1f);

    private static DepthFrame FlatFrame(float depth, Matrix4x4 pose) =>
        new(K, Enumerable.Repeat(depth, K.Width * K.Height).ToArray(), pose, 0);

    [Fact]
    public void Principal_point_pixel_projects_on_the_optical_axis()
    {
        var frame = FlatFrame(1f, Matrix4x4.CreateTranslation(0, 0, 5));
        frame.Depth[1 * K.Width + 1] = 0; // u=1,v=1 invalid, only check u=… below
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, null, new DepthFilter(), points);

        Assert.Equal(11, points.Count);
        // Pixel (u=3, v=1): x = (3 - 1.5) / 2 * 1 = 0.75, y = 0, z = 1, then translated by +5 on Z.
        Assert.Contains(points, p => Vector3.Distance(p, new Vector3(0.75f, 0f, 6f)) < 1e-6f);
    }

    [Fact]
    public void Depth_range_and_confidence_filter_points()
    {
        var frame = FlatFrame(1f, Matrix4x4.Identity);
        frame.Depth[0] = 0.05f;  // too close
        frame.Depth[1] = 2.0f;   // too far
        var confidence = Enumerable.Repeat((byte)255, 12).ToArray();
        confidence[2] = 10;      // low confidence
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, confidence, new DepthFilter(), points);

        Assert.Equal(9, points.Count);
    }

    [Fact]
    public void Region_keeps_only_points_inside_the_sphere()
    {
        var frame = FlatFrame(1f, Matrix4x4.Identity);
        var region = new ScanRegion(new Vector3(0, 0, 1), 0.3f);
        var points = new List<Vector3>();

        DepthBackProjector.Project(frame, null, new DepthFilter(Region: region), points);

        Assert.NotEmpty(points);
        Assert.All(points, p => Assert.True(region.Contains(p)));
        Assert.True(points.Count < 12);
    }
}
```

`tests/Scanner.Capture.Tests/VoxelPointAccumulatorTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class VoxelPointAccumulatorTests
{
    [Fact]
    public void Points_in_the_same_voxel_are_averaged()
    {
        var acc = new VoxelPointAccumulator(0.005f);

        acc.AddRange([new Vector3(0.001f, 0.001f, 0.001f), new Vector3(0.003f, 0.003f, 0.003f)]);

        var snapshot = acc.Snapshot();
        Assert.Single(snapshot);
        Assert.True(Vector3.Distance(snapshot[0], new Vector3(0.002f)) < 1e-6f);
    }

    [Fact]
    public void Negative_coordinates_use_floor_cells()
    {
        var acc = new VoxelPointAccumulator(0.005f);

        acc.AddRange([new Vector3(-0.001f, 0, 0), new Vector3(0.001f, 0, 0)]);

        Assert.Equal(2, acc.CellCount);
    }

    [Fact]
    public void Min_observations_filters_single_hits()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        acc.AddRange([Vector3.Zero, Vector3.Zero, new Vector3(0.1f, 0, 0)]);

        Assert.Equal(2, acc.Snapshot().Length);
        Assert.Single(acc.Snapshot(minObservations: 2));
    }

    [Fact]
    public void Clear_empties_the_cloud()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        acc.AddRange([Vector3.One]);

        acc.Clear();

        Assert.Equal(0, acc.CellCount);
        Assert.Empty(acc.Snapshot());
    }

    [Fact]
    public async Task Concurrent_adds_and_snapshots_do_not_throw()
    {
        var acc = new VoxelPointAccumulator(0.005f);
        var writer = Task.Run(() =>
        {
            for (int i = 0; i < 200; i++)
                acc.AddRange(Enumerable.Range(0, 100).Select(j => new Vector3(i * 0.01f, j * 0.01f, 0)));
        });
        var reader = Task.Run(() =>
        {
            for (int i = 0; i < 200; i++) _ = acc.Snapshot();
        });

        await Task.WhenAll(writer, reader);

        Assert.Equal(20000, acc.CellCount);
    }
}
```

`tests/Scanner.Capture.Tests/PlyTests.cs`:

```csharp
using System.Numerics;
using System.Text;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class PlyTests
{
    [Fact]
    public void Writes_binary_little_endian_header_and_vertices()
    {
        using var stream = new MemoryStream();

        PlyWriter.Write(stream, [new Vector3(1, 2, 3), new Vector3(-1, 0.5f, 0)]);

        var bytes = stream.ToArray();
        var text = Encoding.ASCII.GetString(bytes);
        int headerEnd = text.IndexOf("end_header\n", StringComparison.Ordinal) + "end_header\n".Length;
        Assert.StartsWith("ply\nformat binary_little_endian 1.0\n", text);
        Assert.Contains("element vertex 2\n", text);
        Assert.Equal(headerEnd + 2 * 12, bytes.Length);
        Assert.Equal(1f, BitConverter.ToSingle(bytes, headerEnd));
        Assert.Equal(0.5f, BitConverter.ToSingle(bytes, headerEnd + 16));
    }

    [Fact]
    public void Round_trips_through_reader()
    {
        Vector3[] points = [new(1, 2, 3), new(-0.25f, 0.125f, 9)];
        using var stream = new MemoryStream();
        PlyWriter.Write(stream, points);
        stream.Position = 0;

        var read = PlyReader.Read(stream);

        Assert.Equal(points, read);
    }

    [Fact]
    public void Reader_rejects_non_ply()
    {
        using var stream = new MemoryStream(Encoding.ASCII.GetBytes("hello"));

        Assert.Throws<InvalidDataException>(() => PlyReader.Read(stream));
    }
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `dotnet test tests/Scanner.Capture.Tests`
Expected: FAIL to compile — namespaces `Scanner.Capture.ArCore` / `Scanner.Capture.PointClouds` do not exist.

- [ ] **Step 4: Implement**

`src/Scanner.Capture/ArCore/ArCoreConversions.cs`:

```csharp
using System.Numerics;

namespace Scanner.Capture.ArCore;

/// <summary>Conversions from ARCore conventions to the pipeline conventions (metres, OpenCV camera, row-vector matrices).</summary>
public static class ArCoreConversions
{
    /// <summary>
    /// Converts an ARCore camera pose (column-major 4x4 from <c>Pose.ToMatrix</c>, OpenGL camera axes:
    /// +X right, +Y up, −Z forward, relative to the image readout) into a camera→world matrix with
    /// OpenCV camera axes (+X right, +Y down, +Z forward). The world frame is unchanged (ARCore world, +Y up).
    /// </summary>
    public static Matrix4x4 CameraToWorldFromGlPose(ReadOnlySpan<float> columnMajor)
    {
        if (columnMajor.Length != 16) throw new ArgumentException("Expected a 4x4 matrix (16 values).", nameof(columnMajor));
        var m = columnMajor;
        // Column j of the GL matrix is row j of a row-vector matrix; flipping camera Y and Z negates rows 2 and 3.
        return new Matrix4x4(
            m[0], m[1], m[2], 0,
            -m[4], -m[5], -m[6], 0,
            -m[8], -m[9], -m[10], 0,
            m[12], m[13], m[14], 1);
    }

    /// <summary>Scales pinhole intrinsics from one image resolution to another with the same aspect ratio
    /// (pixel centres at integer coordinates).</summary>
    public static CameraIntrinsics ScaleIntrinsics(float fx, float fy, float cx, float cy,
        int imageWidth, int imageHeight, int targetWidth, int targetHeight)
    {
        float sx = (float)targetWidth / imageWidth;
        float sy = (float)targetHeight / imageHeight;
        return new CameraIntrinsics(targetWidth, targetHeight,
            fx * sx, fy * sy, (cx + 0.5f) * sx - 0.5f, (cy + 0.5f) * sy - 0.5f);
    }

    /// <summary>Builds a <see cref="DepthFrame"/> from a packed row-major uint16 millimetre depth image (0 = invalid).</summary>
    public static DepthFrame DepthFrameFromMillimeters(ushort[] millimeters, CameraIntrinsics intrinsics,
        Matrix4x4 cameraToWorld, double timestampSeconds)
    {
        if (millimeters.Length != intrinsics.Width * intrinsics.Height)
            throw new ArgumentException("Depth buffer size does not match the intrinsics.", nameof(millimeters));
        var depth = new float[millimeters.Length];
        for (int i = 0; i < depth.Length; i++) depth[i] = millimeters[i] * 0.001f;
        return new DepthFrame(intrinsics, depth, cameraToWorld, timestampSeconds);
    }
}
```

`src/Scanner.Capture/PointClouds/DepthBackProjector.cs`:

```csharp
using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Sphere in world space that bounds the piece being scanned.</summary>
public readonly record struct ScanRegion(Vector3 Center, float Radius)
{
    public bool Contains(Vector3 p) => Vector3.DistanceSquared(p, Center) <= Radius * Radius;
}

/// <summary>Which depth pixels become points. Confidence is 0–255 (ARCore raw depth confidence).</summary>
public sealed record DepthFilter(float MinDepth = 0.10f, float MaxDepth = 1.50f, byte MinConfidence = 128, ScanRegion? Region = null);

public static class DepthBackProjector
{
    /// <summary>Back-projects every accepted depth pixel into world space and appends it to <paramref name="output"/>.</summary>
    public static void Project(DepthFrame frame, byte[]? confidence, DepthFilter filter, List<Vector3> output)
    {
        var k = frame.Intrinsics;
        if (confidence is not null && confidence.Length != frame.Depth.Length)
            throw new ArgumentException("Confidence buffer size does not match the depth buffer.", nameof(confidence));

        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
        {
            int i = v * k.Width + u;
            float d = frame.Depth[i];
            if (d < filter.MinDepth || d > filter.MaxDepth) continue;
            if (confidence is not null && confidence[i] < filter.MinConfidence) continue;

            var cameraPoint = new Vector3((u - k.Cx) / k.Fx * d, (v - k.Cy) / k.Fy * d, d);
            var world = Vector3.Transform(cameraPoint, frame.CameraToWorld);
            if (filter.Region is { } region && !region.Contains(world)) continue;
            output.Add(world);
        }
    }
}
```

`src/Scanner.Capture/PointClouds/VoxelPointAccumulator.cs`:

```csharp
using System.Numerics;
using System.Runtime.InteropServices;

namespace Scanner.Capture.PointClouds;

/// <summary>Accumulates points into a voxel grid; each occupied voxel yields the mean of its points. Thread-safe.</summary>
public sealed class VoxelPointAccumulator
{
    private readonly Dictionary<(int, int, int), Cell> _cells = new();
    private readonly object _gate = new();

    public VoxelPointAccumulator(float voxelSize)
    {
        if (voxelSize <= 0) throw new ArgumentOutOfRangeException(nameof(voxelSize));
        VoxelSize = voxelSize;
    }

    public float VoxelSize { get; }

    public int CellCount
    {
        get { lock (_gate) return _cells.Count; }
    }

    public void AddRange(IEnumerable<Vector3> points)
    {
        lock (_gate)
        {
            foreach (var p in points)
            {
                var key = ((int)MathF.Floor(p.X / VoxelSize), (int)MathF.Floor(p.Y / VoxelSize), (int)MathF.Floor(p.Z / VoxelSize));
                ref var cell = ref CollectionsMarshal.GetValueRefOrAddDefault(_cells, key, out _);
                cell.Sum += p;
                cell.Count++;
            }
        }
    }

    public Vector3[] Snapshot(int minObservations = 1)
    {
        lock (_gate)
        {
            var result = new List<Vector3>(_cells.Count);
            foreach (var cell in _cells.Values)
                if (cell.Count >= minObservations) result.Add(cell.Sum / cell.Count);
            return result.ToArray();
        }
    }

    public void Clear()
    {
        lock (_gate) _cells.Clear();
    }

    private struct Cell
    {
        public Vector3 Sum;
        public int Count;
    }
}
```

`src/Scanner.Capture/PointClouds/Ply.cs`:

```csharp
using System.Buffers.Binary;
using System.Numerics;
using System.Text;

namespace Scanner.Capture.PointClouds;

/// <summary>Writes point clouds as binary little-endian PLY (float x, y, z in metres).</summary>
public static class PlyWriter
{
    public static void Write(Stream stream, IReadOnlyList<Vector3> points)
    {
        var header = "ply\n"
                     + "format binary_little_endian 1.0\n"
                     + "comment Scan3D point cloud, units: metres\n"
                     + $"element vertex {points.Count}\n"
                     + "property float x\n"
                     + "property float y\n"
                     + "property float z\n"
                     + "end_header\n";
        stream.Write(Encoding.ASCII.GetBytes(header));

        Span<byte> vertex = stackalloc byte[12];
        foreach (var p in points)
        {
            BinaryPrimitives.WriteSingleLittleEndian(vertex[..4], p.X);
            BinaryPrimitives.WriteSingleLittleEndian(vertex[4..8], p.Y);
            BinaryPrimitives.WriteSingleLittleEndian(vertex[8..], p.Z);
            stream.Write(vertex);
        }
    }
}

/// <summary>Reads the PLY files produced by <see cref="PlyWriter"/> (binary little-endian, float x, y, z only).</summary>
public static class PlyReader
{
    public static Vector3[] Read(Stream stream)
    {
        int count = -1;
        if (ReadLine(stream) != "ply") throw new InvalidDataException("Not a PLY file.");
        if (ReadLine(stream) != "format binary_little_endian 1.0") throw new InvalidDataException("Only binary little-endian PLY is supported.");
        while (true)
        {
            var line = ReadLine(stream) ?? throw new InvalidDataException("Unexpected end of PLY header.");
            if (line == "end_header") break;
            if (line.StartsWith("element vertex ", StringComparison.Ordinal)) count = int.Parse(line["element vertex ".Length..]);
        }
        if (count < 0) throw new InvalidDataException("PLY header has no vertex element.");

        var points = new Vector3[count];
        Span<byte> vertex = stackalloc byte[12];
        for (int i = 0; i < count; i++)
        {
            stream.ReadExactly(vertex);
            points[i] = new Vector3(
                BinaryPrimitives.ReadSingleLittleEndian(vertex[..4]),
                BinaryPrimitives.ReadSingleLittleEndian(vertex[4..8]),
                BinaryPrimitives.ReadSingleLittleEndian(vertex[8..]));
        }
        return points;
    }

    private static string? ReadLine(Stream stream)
    {
        var builder = new StringBuilder();
        while (true)
        {
            int b = stream.ReadByte();
            if (b < 0) return builder.Length == 0 ? null : builder.ToString();
            if (b == '\n') return builder.ToString();
            builder.Append((char)b);
            if (builder.Length > 256) throw new InvalidDataException("PLY header line too long.");
        }
    }
}
```

Note on `DepthBackProjectorTests.Principal_point_pixel_projects_on_the_optical_axis`: 12 pixels, one set to 0 depth → 11 points.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `dotnet test tests/Scanner.Capture.Tests`
Expected: PASS (17 tests).

- [ ] **Step 6: Commit**

```bash
git add tests/Scanner.Capture.Tests src/Scanner.Capture/ArCore src/Scanner.Capture/PointClouds Scan3D.slnx
git commit -m "feat(capture): ARCore conversions, depth back-projection, voxel accumulator, PLY I/O" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Object isolation (remove the table, keep the piece)

**Files:**
- Create: `src/Scanner.Capture/PointClouds/ObjectIsolator.cs`
- Test: `tests/Scanner.Capture.Tests/ObjectIsolatorTests.cs`

**Interfaces:**
- Consumes: nothing beyond `System.Numerics`.
- Produces: `static Vector3[] ObjectIsolator.Isolate(IReadOnlyList<Vector3> points, Vector3 target, float? supportPlaneHeight, float linkDistance, float planeMargin = 0.004f)` — world +Y up; drops points with `Y <= supportPlaneHeight + planeMargin`, links points closer than ~`linkDistance` (cell grid, 26-neighbourhood), returns the cluster nearest to `target`.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Capture.Tests/ObjectIsolatorTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Tests;

public class ObjectIsolatorTests
{
    private const float Step = 0.005f;

    // A 4 cm cube of points resting on a 40 cm table at y = 0, plus a second box 20 cm away.
    private static List<Vector3> Scene()
    {
        var points = new List<Vector3>();
        for (float x = -0.2f; x <= 0.2f; x += Step)
        for (float z = -0.2f; z <= 0.2f; z += Step)
            points.Add(new Vector3(x, 0, z));
        AddBox(points, new Vector3(0, 0.02f + Step, 0), 0.02f);
        AddBox(points, new Vector3(0.15f, 0.02f + Step, 0.15f), 0.02f);
        return points;
    }

    private static void AddBox(List<Vector3> points, Vector3 center, float half)
    {
        for (float x = -half; x <= half; x += Step)
        for (float y = -half; y <= half; y += Step)
        for (float z = -half; z <= half; z += Step)
            points.Add(center + new Vector3(x, y, z));
    }

    [Fact]
    public void Removes_table_and_keeps_only_the_piece_near_the_target()
    {
        var result = ObjectIsolator.Isolate(Scene(), target: new Vector3(0, 0.03f, 0), supportPlaneHeight: 0f, linkDistance: 0.01f);

        Assert.NotEmpty(result);
        Assert.All(result, p =>
        {
            Assert.True(p.Y > 0.004f);
            Assert.True(MathF.Abs(p.X) <= 0.021f && MathF.Abs(p.Z) <= 0.021f);
        });
    }

    [Fact]
    public void Without_plane_height_the_table_links_everything()
    {
        var scene = Scene();

        var result = ObjectIsolator.Isolate(scene, target: new Vector3(0, 0.03f, 0), supportPlaneHeight: null, linkDistance: 0.01f);

        Assert.Equal(scene.Count, result.Length);
    }

    [Fact]
    public void Picks_the_other_box_when_the_target_is_there()
    {
        var result = ObjectIsolator.Isolate(Scene(), target: new Vector3(0.15f, 0.03f, 0.15f), supportPlaneHeight: 0f, linkDistance: 0.01f);

        Assert.NotEmpty(result);
        Assert.All(result, p => Assert.True(p.X > 0.1f && p.Z > 0.1f));
    }

    [Fact]
    public void Empty_input_returns_empty()
    {
        Assert.Empty(ObjectIsolator.Isolate([], Vector3.Zero, 0f, 0.01f));
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `dotnet test tests/Scanner.Capture.Tests --filter "FullyQualifiedName~ObjectIsolatorTests"`
Expected: FAIL to compile — `ObjectIsolator` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Capture/PointClouds/ObjectIsolator.cs`:

```csharp
using System.Numerics;

namespace Scanner.Capture.PointClouds;

/// <summary>Separates the scanned piece from its surroundings: drops the support plane, then keeps the
/// connected cluster of points nearest to the target point.</summary>
public static class ObjectIsolator
{
    public static Vector3[] Isolate(IReadOnlyList<Vector3> points, Vector3 target, float? supportPlaneHeight,
        float linkDistance, float planeMargin = 0.004f)
    {
        if (linkDistance <= 0) throw new ArgumentOutOfRangeException(nameof(linkDistance));
        var kept = supportPlaneHeight is { } height
            ? points.Where(p => p.Y > height + planeMargin).ToList()
            : points.ToList();
        if (kept.Count == 0) return [];

        var cells = new Dictionary<(int, int, int), List<int>>();
        for (int i = 0; i < kept.Count; i++)
        {
            var key = CellOf(kept[i], linkDistance);
            if (!cells.TryGetValue(key, out var list)) cells[key] = list = [];
            list.Add(i);
        }

        var visited = new HashSet<(int, int, int)>();
        List<(int, int, int)>? best = null;
        float bestDistance = float.PositiveInfinity;
        foreach (var start in cells.Keys)
        {
            if (!visited.Add(start)) continue;
            var component = new List<(int, int, int)>();
            var queue = new Queue<(int, int, int)>();
            queue.Enqueue(start);
            float nearest = float.PositiveInfinity;
            while (queue.Count > 0)
            {
                var cell = queue.Dequeue();
                component.Add(cell);
                foreach (int i in cells[cell]) nearest = MathF.Min(nearest, Vector3.DistanceSquared(kept[i], target));
                foreach (var neighbour in Neighbours(cell))
                    if (cells.ContainsKey(neighbour) && visited.Add(neighbour)) queue.Enqueue(neighbour);
            }
            if (nearest < bestDistance)
            {
                bestDistance = nearest;
                best = component;
            }
        }

        return best!.SelectMany(cell => cells[cell]).Select(i => kept[i]).ToArray();
    }

    private static (int, int, int) CellOf(Vector3 p, float size) =>
        ((int)MathF.Floor(p.X / size), (int)MathF.Floor(p.Y / size), (int)MathF.Floor(p.Z / size));

    private static IEnumerable<(int, int, int)> Neighbours((int X, int Y, int Z) c)
    {
        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            if (dx != 0 || dy != 0 || dz != 0) yield return (c.X + dx, c.Y + dy, c.Z + dz);
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `dotnet test tests/Scanner.Capture.Tests --filter "FullyQualifiedName~ObjectIsolatorTests"`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/Scanner.Capture/PointClouds/ObjectIsolator.cs tests/Scanner.Capture.Tests/ObjectIsolatorTests.cs
git commit -m "feat(capture): isolate the scanned piece from the support plane" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Scan session storage (frames, manifest, `.scan` archive)

**Files:**
- Create: `src/Scanner.Capture/Sessions/FrameCodec.cs`, `src/Scanner.Capture/Sessions/ScanSession.cs`
- Test: `tests/Scanner.Capture.Tests/ScanSessionTests.cs`

**Interfaces:**
- Consumes: `DepthFrame`, `CameraIntrinsics`.
- Produces:
  - `static class FrameCodec { void Write(Stream s, DepthFrame frame, byte[]? confidence); (DepthFrame Frame, byte[]? Confidence) Read(Stream s); }` — depth stored as uint16 millimetres (rounded, clamped to 65535).
  - `sealed record ScanManifest(int Version, string Id, DateTimeOffset CreatedUtc, string Device, int FrameCount, float[]? Target, float? SupportPlaneHeight, int PointCount)`
  - `sealed class ScanSessionWriter(string directory, string id, string device)`: `int FrameCount`, `void AppendFrame(DepthFrame frame, byte[]? confidence)`, `void Complete(Vector3? target, float? supportPlaneHeight, IReadOnlyList<Vector3> points)` (writes `points.ply` and `manifest.json`).
  - `static class ScanSessionReader { ScanManifest ReadManifest(string dir); Vector3[] ReadPoints(string dir); IEnumerable<(DepthFrame Frame, byte[]? Confidence)> ReadFrames(string dir); }`
  - `static class ScanArchive { void Export(string sessionDirectory, string scanFilePath); }`
  - File layout: `<dir>/manifest.json`, `<dir>/points.ply`, `<dir>/frames/000001.frame` …

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Capture.Tests/ScanSessionTests.cs`:

```csharp
using System.IO.Compression;
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Tests;

public sealed class ScanSessionTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "scan3d-tests-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_root)) Directory.Delete(_root, recursive: true);
    }

    private static DepthFrame Frame(float depth, double time) =>
        new(new CameraIntrinsics(3, 2, 100f, 101f, 1f, 0.5f), [depth, 0f, 0.5f, 1.2346f, 70f, depth],
            Matrix4x4.CreateTranslation(1, 2, 3), time);

    [Fact]
    public void Frame_codec_round_trips_with_millimetre_precision()
    {
        using var stream = new MemoryStream();
        var frame = Frame(0.8f, 1.25);
        byte[] confidence = [1, 2, 3, 4, 5, 6];

        FrameCodec.Write(stream, frame, confidence);
        stream.Position = 0;
        var (read, readConfidence) = FrameCodec.Read(stream);

        Assert.Equal(frame.Intrinsics, read.Intrinsics);
        Assert.Equal(frame.CameraToWorld, read.CameraToWorld);
        Assert.Equal(1.25, read.TimestampSeconds);
        Assert.Equal(1.235f, read.Depth[3], 4);
        Assert.Equal(65.535f, read.Depth[4], 3); // clamped to uint16 millimetres
        Assert.Equal(0f, read.Depth[1]);
        Assert.Equal(confidence, readConfidence);
    }

    [Fact]
    public void Frame_codec_supports_missing_confidence_and_rejects_garbage()
    {
        using var stream = new MemoryStream();
        FrameCodec.Write(stream, Frame(1f, 0), null);
        stream.Position = 0;

        Assert.Null(FrameCodec.Read(stream).Confidence);
        Assert.Throws<InvalidDataException>(() => FrameCodec.Read(new MemoryStream([1, 2, 3, 4, 5, 6, 7, 8])));
    }

    [Fact]
    public void Session_writes_frames_points_and_manifest()
    {
        var dir = Path.Combine(_root, "s1");
        var writer = new ScanSessionWriter(dir, "s1", "Pixel 8");
        writer.AppendFrame(Frame(0.5f, 0), null);
        writer.AppendFrame(Frame(0.6f, 0.2), [9, 9, 9, 9, 9, 9]);

        writer.Complete(new Vector3(0.1f, 0.2f, 0.3f), 0.05f, [Vector3.One, Vector3.Zero]);

        var manifest = ScanSessionReader.ReadManifest(dir);
        Assert.Equal(1, manifest.Version);
        Assert.Equal("s1", manifest.Id);
        Assert.Equal("Pixel 8", manifest.Device);
        Assert.Equal(2, manifest.FrameCount);
        Assert.Equal(2, manifest.PointCount);
        Assert.Equal(new[] { 0.1f, 0.2f, 0.3f }, manifest.Target);
        Assert.Equal(0.05f, manifest.SupportPlaneHeight);
        Assert.Equal(new[] { Vector3.One, Vector3.Zero }, ScanSessionReader.ReadPoints(dir));
        var frames = ScanSessionReader.ReadFrames(dir).ToList();
        Assert.Equal(2, frames.Count);
        Assert.Equal(0.2, frames[1].Frame.TimestampSeconds);
        Assert.NotNull(frames[1].Confidence);
    }

    [Fact]
    public void Archive_contains_the_whole_session()
    {
        var dir = Path.Combine(_root, "s2");
        var writer = new ScanSessionWriter(dir, "s2", "test");
        writer.AppendFrame(Frame(0.5f, 0), null);
        writer.Complete(null, null, [Vector3.One]);
        var archive = Path.Combine(_root, "s2.scan");

        ScanArchive.Export(dir, archive);

        using var zip = ZipFile.OpenRead(archive);
        var names = zip.Entries.Select(e => e.FullName.Replace('\\', '/')).ToList();
        Assert.Contains("manifest.json", names);
        Assert.Contains("points.ply", names);
        Assert.Contains("frames/000001.frame", names);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `dotnet test tests/Scanner.Capture.Tests --filter "FullyQualifiedName~ScanSessionTests"`
Expected: FAIL to compile — `Scanner.Capture.Sessions` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Capture/Sessions/FrameCodec.cs`:

```csharp
using System.Numerics;
using System.Text;

namespace Scanner.Capture.Sessions;

/// <summary>
/// Binary depth-frame file: magic "S3DF", version, width, height, fx, fy, cx, cy, timestamp,
/// camera→world matrix (16 floats, row-major M11..M44), confidence flag, depth as uint16 millimetres,
/// optional confidence bytes. Little-endian.
/// </summary>
public static class FrameCodec
{
    private const int Version = 1;
    private static readonly byte[] Magic = Encoding.ASCII.GetBytes("S3DF");

    public static void Write(Stream stream, DepthFrame frame, byte[]? confidence)
    {
        var k = frame.Intrinsics;
        if (confidence is not null && confidence.Length != frame.Depth.Length)
            throw new ArgumentException("Confidence buffer size does not match the depth buffer.", nameof(confidence));

        using var writer = new BinaryWriter(stream, Encoding.ASCII, leaveOpen: true);
        writer.Write(Magic);
        writer.Write(Version);
        writer.Write(k.Width);
        writer.Write(k.Height);
        writer.Write(k.Fx);
        writer.Write(k.Fy);
        writer.Write(k.Cx);
        writer.Write(k.Cy);
        writer.Write(frame.TimestampSeconds);
        foreach (float value in Elements(frame.CameraToWorld)) writer.Write(value);
        writer.Write(confidence is not null);
        foreach (float d in frame.Depth)
            writer.Write((ushort)Math.Clamp(MathF.Round(d * 1000f), 0f, ushort.MaxValue));
        if (confidence is not null) writer.Write(confidence);
    }

    public static (DepthFrame Frame, byte[]? Confidence) Read(Stream stream)
    {
        using var reader = new BinaryReader(stream, Encoding.ASCII, leaveOpen: true);
        try
        {
            if (!reader.ReadBytes(4).AsSpan().SequenceEqual(Magic)) throw new InvalidDataException("Not a Scan3D frame file.");
            int version = reader.ReadInt32();
            if (version != Version) throw new InvalidDataException($"Unsupported frame version {version}.");

            var k = new CameraIntrinsics(reader.ReadInt32(), reader.ReadInt32(),
                reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            double timestamp = reader.ReadDouble();
            var m = new float[16];
            for (int i = 0; i < 16; i++) m[i] = reader.ReadSingle();
            bool hasConfidence = reader.ReadBoolean();

            int count = checked(k.Width * k.Height);
            var depth = new float[count];
            for (int i = 0; i < count; i++) depth[i] = reader.ReadUInt16() * 0.001f;
            byte[]? confidence = hasConfidence ? reader.ReadBytes(count) : null;
            if (confidence is not null && confidence.Length != count) throw new InvalidDataException("Truncated confidence data.");

            var pose = new Matrix4x4(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
            return (new DepthFrame(k, depth, pose, timestamp), confidence);
        }
        catch (EndOfStreamException ex)
        {
            throw new InvalidDataException("Truncated frame file.", ex);
        }
    }

    private static IEnumerable<float> Elements(Matrix4x4 m) =>
    [
        m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
        m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44,
    ];
}
```

`src/Scanner.Capture/Sessions/ScanSession.cs`:

```csharp
using System.IO.Compression;
using System.Numerics;
using System.Text.Json;
using Scanner.Capture.PointClouds;

namespace Scanner.Capture.Sessions;

/// <summary>Session metadata stored as manifest.json. Target is the world point the user aimed at (x, y, z).</summary>
public sealed record ScanManifest(
    int Version,
    string Id,
    DateTimeOffset CreatedUtc,
    string Device,
    int FrameCount,
    float[]? Target,
    float? SupportPlaneHeight,
    int PointCount);

internal static class SessionPaths
{
    public const string Manifest = "manifest.json";
    public const string Points = "points.ply";
    public const string Frames = "frames";

    public static string Frame(string directory, int index) => Path.Combine(directory, Frames, $"{index:D6}.frame");

    public static readonly JsonSerializerOptions Json = new() { WriteIndented = true };
}

/// <summary>Writes a scan session folder: frames as they arrive, then points and manifest on completion.</summary>
public sealed class ScanSessionWriter
{
    private readonly string _directory;
    private readonly string _id;
    private readonly string _device;
    private readonly DateTimeOffset _created = DateTimeOffset.UtcNow;

    public ScanSessionWriter(string directory, string id, string device)
    {
        _directory = directory;
        _id = id;
        _device = device;
        Directory.CreateDirectory(Path.Combine(directory, SessionPaths.Frames));
    }

    public int FrameCount { get; private set; }

    public void AppendFrame(DepthFrame frame, byte[]? confidence)
    {
        using var file = File.Create(SessionPaths.Frame(_directory, FrameCount + 1));
        FrameCodec.Write(file, frame, confidence);
        FrameCount++;
    }

    public void Complete(Vector3? target, float? supportPlaneHeight, IReadOnlyList<Vector3> points)
    {
        using (var file = File.Create(Path.Combine(_directory, SessionPaths.Points)))
            PlyWriter.Write(file, points);

        var manifest = new ScanManifest(1, _id, _created, _device, FrameCount,
            target is { } t ? [t.X, t.Y, t.Z] : null, supportPlaneHeight, points.Count);
        File.WriteAllText(Path.Combine(_directory, SessionPaths.Manifest), JsonSerializer.Serialize(manifest, SessionPaths.Json));
    }
}

public static class ScanSessionReader
{
    public static ScanManifest ReadManifest(string directory) =>
        JsonSerializer.Deserialize<ScanManifest>(File.ReadAllText(Path.Combine(directory, SessionPaths.Manifest)), SessionPaths.Json)
        ?? throw new InvalidDataException("Empty manifest.");

    public static Vector3[] ReadPoints(string directory)
    {
        using var file = File.OpenRead(Path.Combine(directory, SessionPaths.Points));
        return PlyReader.Read(file);
    }

    public static IEnumerable<(DepthFrame Frame, byte[]? Confidence)> ReadFrames(string directory)
    {
        var files = Directory.GetFiles(Path.Combine(directory, SessionPaths.Frames), "*.frame").Order(StringComparer.Ordinal);
        foreach (var path in files)
        {
            using var file = File.OpenRead(path);
            yield return FrameCodec.Read(file);
        }
    }
}

public static class ScanArchive
{
    /// <summary>Zips a session folder into a portable .scan file (overwrites an existing file).</summary>
    public static void Export(string sessionDirectory, string scanFilePath)
    {
        if (File.Exists(scanFilePath)) File.Delete(scanFilePath);
        ZipFile.CreateFromDirectory(sessionDirectory, scanFilePath, CompressionLevel.Fastest, includeBaseDirectory: false);
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `dotnet test tests/Scanner.Capture.Tests`
Expected: PASS (all Capture tests).

- [ ] **Step 5: Commit**

```bash
git add src/Scanner.Capture/Sessions tests/Scanner.Capture.Tests/ScanSessionTests.cs
git commit -m "feat(capture): scan session storage and .scan archive" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Live scan session (state machine, throttling, accumulation, completion)

**Files:**
- Create: `src/Scanner.Capture/Live/LiveScanSession.cs`
- Test: `tests/Scanner.Capture.Tests/LiveScanSessionTests.cs`

**Interfaces:**
- Consumes: `ScanSessionWriter` (Task 4), `VoxelPointAccumulator`, `DepthBackProjector`, `DepthFilter`, `ScanRegion`, `ObjectIsolator` (Tasks 2–3), `DepthFrame`.
- Produces (namespace `Scanner.Capture.Live`):
  - `enum LiveScanState { Idle, WaitingForTarget, Recording, Paused, Completed }`
  - `sealed record LiveScanOptions(float VoxelSize = 0.005f, float RegionRadius = 0.30f, double IntegrationIntervalSeconds = 0.2, DepthFilter? Filter = null, int MinIsolatedPoints = 200)`
  - `sealed record LiveScanResult(Vector3[] AllPoints, Vector3[] PiecePoints, bool Isolated)`
  - `sealed class LiveScanSession(ScanSessionWriter writer, LiveScanOptions? options = null)` with `LiveScanOptions Options`, `LiveScanState State`, `Vector3? Target`, `float? SupportPlaneHeight`, `int PointCount`, `int FrameCount`, `void RequestStart()`, `void SetTarget(Vector3 target, float? supportPlaneHeight)`, `void Pause()`, `bool ShouldIntegrate(double timestampSeconds)`, `int Integrate(DepthFrame frame, byte[]? confidence)`, `Vector3[] SnapshotPoints()`, `LiveScanResult Complete()`.
  - Threading contract: `Integrate` runs on one worker thread at a time; `SnapshotPoints` may run concurrently on the render thread; state methods run on the UI/render thread. All are safe to call concurrently.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Capture.Tests/LiveScanSessionTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;
using Scanner.Capture.Live;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Tests;

public sealed class LiveScanSessionTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "scan3d-live-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, recursive: true);
    }

    private LiveScanSession NewSession(LiveScanOptions? options = null) =>
        new(new ScanSessionWriter(_dir, "live", "test"), options);

    // 20x20 pixels, every pixel 1 m away on the camera's optical axis direction; identity pose.
    private static DepthFrame FlatFrame(double time)
    {
        var k = new CameraIntrinsics(20, 20, 20f, 20f, 9.5f, 9.5f);
        return new DepthFrame(k, Enumerable.Repeat(1f, 400).ToArray(), Matrix4x4.Identity, time);
    }

    [Fact]
    public void Start_waits_for_target_then_records_with_throttling()
    {
        var session = NewSession();

        session.RequestStart();
        Assert.Equal(LiveScanState.WaitingForTarget, session.State);
        Assert.False(session.ShouldIntegrate(0));

        session.SetTarget(new Vector3(0, 0, 1), 0.5f);
        Assert.Equal(LiveScanState.Recording, session.State);
        Assert.Equal(new Vector3(0, 0, 1), session.Target);
        Assert.Equal(0.5f, session.SupportPlaneHeight);
        Assert.True(session.ShouldIntegrate(10.0));
        Assert.False(session.ShouldIntegrate(10.1));
        Assert.True(session.ShouldIntegrate(10.25));
    }

    [Fact]
    public void Pause_and_resume_keep_the_target()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(Vector3.Zero, null);

        session.Pause();
        Assert.Equal(LiveScanState.Paused, session.State);
        Assert.False(session.ShouldIntegrate(1));

        session.RequestStart();
        Assert.Equal(LiveScanState.Recording, session.State);
        Assert.Equal(Vector3.Zero, session.Target);
    }

    [Fact]
    public void Pause_while_waiting_for_target_returns_to_idle()
    {
        var session = NewSession();
        session.RequestStart();

        session.Pause();

        Assert.Equal(LiveScanState.Idle, session.State);
    }

    [Fact]
    public void SetTarget_outside_waiting_state_throws()
    {
        Assert.Throws<InvalidOperationException>(() => NewSession().SetTarget(Vector3.Zero, null));
    }

    [Fact]
    public void Integrate_keeps_points_inside_the_region_and_records_the_frame()
    {
        var session = NewSession(new LiveScanOptions(RegionRadius: 0.2f));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);

        int added = session.Integrate(FlatFrame(0), null);

        Assert.True(added > 0 && added < 400);
        Assert.Equal(1, session.FrameCount);
        Assert.All(session.SnapshotPoints(), p => Assert.True(Vector3.Distance(p, new Vector3(0, 0, 1)) <= 0.2f + 0.005f));
        Assert.Equal(session.SnapshotPoints().Length, session.PointCount);
    }

    [Fact]
    public void Complete_writes_the_session_and_isolates_the_piece()
    {
        var session = NewSession(new LiveScanOptions(MinIsolatedPoints: 1));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Integrate(FlatFrame(0), null);

        var result = session.Complete();

        Assert.Equal(LiveScanState.Completed, session.State);
        Assert.True(result.Isolated);
        Assert.NotEmpty(result.PiecePoints);
        Assert.Equal(result.PiecePoints.Length, ScanSessionReader.ReadPoints(_dir).Length);
        Assert.Equal(1, ScanSessionReader.ReadManifest(_dir).FrameCount);
        Assert.Throws<InvalidOperationException>(() => session.Complete());
        Assert.Throws<InvalidOperationException>(() => session.RequestStart());
    }

    [Fact]
    public void Complete_without_enough_isolated_points_falls_back_to_all_points()
    {
        var session = NewSession(new LiveScanOptions(MinIsolatedPoints: 1_000_000));
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Integrate(FlatFrame(0), null);

        var result = session.Complete();

        Assert.False(result.Isolated);
        Assert.Equal(result.AllPoints.Length, result.PiecePoints.Length);
    }

    [Fact]
    public void Integrate_after_completion_is_ignored()
    {
        var session = NewSession();
        session.RequestStart();
        session.SetTarget(new Vector3(0, 0, 1), null);
        session.Complete();

        Assert.Equal(0, session.Integrate(FlatFrame(1), null));
        Assert.Equal(0, session.FrameCount);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `dotnet test tests/Scanner.Capture.Tests --filter "FullyQualifiedName~LiveScanSessionTests"`
Expected: FAIL to compile — `Scanner.Capture.Live` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Capture/Live/LiveScanSession.cs`:

```csharp
using System.Numerics;
using Scanner.Capture.PointClouds;
using Scanner.Capture.Sessions;

namespace Scanner.Capture.Live;

public enum LiveScanState { Idle, WaitingForTarget, Recording, Paused, Completed }

public sealed record LiveScanOptions(
    float VoxelSize = 0.005f,
    float RegionRadius = 0.30f,
    double IntegrationIntervalSeconds = 0.2,
    DepthFilter? Filter = null,
    int MinIsolatedPoints = 200);

public sealed record LiveScanResult(Vector3[] AllPoints, Vector3[] PiecePoints, bool Isolated);

/// <summary>
/// Platform-independent state of one live scan: throttles depth integration, accumulates points inside the
/// region around the target, records frames, and isolates the piece on completion.
/// <see cref="Integrate"/> runs on one worker thread at a time; <see cref="SnapshotPoints"/> may run concurrently
/// on the render thread; state methods run on the UI/render thread.
/// </summary>
public sealed class LiveScanSession
{
    private readonly ScanSessionWriter _writer;
    private readonly VoxelPointAccumulator _accumulator;
    private readonly object _gate = new();
    private LiveScanState _state = LiveScanState.Idle;
    private double _lastIntegration = double.NegativeInfinity;

    public LiveScanSession(ScanSessionWriter writer, LiveScanOptions? options = null)
    {
        _writer = writer;
        Options = options ?? new LiveScanOptions();
        _accumulator = new VoxelPointAccumulator(Options.VoxelSize);
    }

    public LiveScanOptions Options { get; }

    public LiveScanState State
    {
        get { lock (_gate) return _state; }
    }

    public Vector3? Target { get; private set; }
    public float? SupportPlaneHeight { get; private set; }
    public int PointCount => _accumulator.CellCount;

    public int FrameCount
    {
        get { lock (_gate) return _writer.FrameCount; }
    }

    /// <summary>Idle → WaitingForTarget (the renderer then picks the target); Paused → Recording.</summary>
    public void RequestStart()
    {
        lock (_gate)
        {
            _state = _state switch
            {
                LiveScanState.Idle => LiveScanState.WaitingForTarget,
                LiveScanState.Paused => LiveScanState.Recording,
                LiveScanState.WaitingForTarget or LiveScanState.Recording => _state,
                _ => throw new InvalidOperationException("The scan is already completed."),
            };
        }
    }

    /// <summary>Sets the aimed-at world point and the support plane height (world +Y up), and starts recording.</summary>
    public void SetTarget(Vector3 target, float? supportPlaneHeight)
    {
        lock (_gate)
        {
            if (_state != LiveScanState.WaitingForTarget)
                throw new InvalidOperationException($"Cannot set the target while {_state}.");
            Target = target;
            SupportPlaneHeight = supportPlaneHeight;
            _state = LiveScanState.Recording;
        }
    }

    public void Pause()
    {
        lock (_gate)
        {
            _state = _state switch
            {
                LiveScanState.Recording => LiveScanState.Paused,
                LiveScanState.WaitingForTarget => LiveScanState.Idle,
                _ => _state,
            };
        }
    }

    /// <summary>True when recording and at least <see cref="LiveScanOptions.IntegrationIntervalSeconds"/> passed since the last accepted frame.</summary>
    public bool ShouldIntegrate(double timestampSeconds)
    {
        lock (_gate)
        {
            if (_state != LiveScanState.Recording) return false;
            if (timestampSeconds - _lastIntegration < Options.IntegrationIntervalSeconds) return false;
            _lastIntegration = timestampSeconds;
            return true;
        }
    }

    /// <summary>Back-projects the frame, accumulates the points inside the region and records the frame.
    /// Returns the number of points added (0 once the scan is completed).</summary>
    public int Integrate(DepthFrame frame, byte[]? confidence)
    {
        Vector3? target;
        lock (_gate)
        {
            if (_state == LiveScanState.Completed) return 0;
            target = Target;
        }

        var filter = (Options.Filter ?? new DepthFilter()) with
        {
            Region = target is { } t ? new ScanRegion(t, Options.RegionRadius) : null,
        };
        var points = new List<Vector3>();
        DepthBackProjector.Project(frame, confidence, filter, points);

        lock (_gate)
        {
            if (_state == LiveScanState.Completed) return 0;
            _accumulator.AddRange(points);
            _writer.AppendFrame(frame, confidence);
        }
        return points.Count;
    }

    public Vector3[] SnapshotPoints() => _accumulator.Snapshot();

    /// <summary>Stops the scan, isolates the piece around the target, writes points and manifest.</summary>
    public LiveScanResult Complete()
    {
        lock (_gate)
        {
            if (_state == LiveScanState.Completed) throw new InvalidOperationException("The scan is already completed.");
            _state = LiveScanState.Completed;

            var all = _accumulator.Snapshot();
            var piece = Target is { } t
                ? ObjectIsolator.Isolate(all, t, SupportPlaneHeight, 2 * Options.VoxelSize)
                : all;
            bool isolated = Target is not null && piece.Length >= Options.MinIsolatedPoints;
            if (!isolated) piece = all;

            _writer.Complete(Target, SupportPlaneHeight, piece);
            return new LiveScanResult(all, piece, isolated);
        }
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `dotnet test tests/Scanner.Capture.Tests`
Expected: PASS (all Capture tests).

- [ ] **Step 5: Commit**

```bash
git add src/Scanner.Capture/Live tests/Scanner.Capture.Tests/LiveScanSessionTests.cs
git commit -m "feat(capture): live scan session state machine" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: AR scan screen (camera background + live point overlay, ARCore depth capture)

**Files:**
- Modify: `src/Scanner.App/Scanner.App.csproj` (project reference to `Scanner.Capture`)
- Modify: `src/Scanner.App/Platforms/Android/MainActivity.cs` (portrait lock)
- Modify: `src/Scanner.App/Platforms/Android/AndroidManifest.xml` (nothing to add if the spike already declared camera/AR/GLES 3 — verify)
- Modify: `src/Scanner.App/MauiProgram.cs`, `src/Scanner.App/AppShell.xaml` (+ `.cs`); delete `MainPage.xaml` / `MainPage.xaml.cs`
- Create: `src/Scanner.App/Controls/ArScanView.cs`
- Create: `src/Scanner.App/Services/ArPlatform.cs`, `src/Scanner.App/Services/SessionStore.cs`
- Create: `src/Scanner.App/Pages/ScanPage.cs`
- Create (Android): `src/Scanner.App/Platforms/Android/ArPlatform.Android.cs`, `Platforms/Android/Rendering/GlUtil.cs`, `Platforms/Android/Rendering/CameraBackgroundRenderer.cs`, `Platforms/Android/Rendering/PointCloudRenderer.cs`, `Platforms/Android/Ar/DepthFrameReader.cs`, `Platforms/Android/Ar/ArScanRenderer.cs`, `Platforms/Android/Handlers/ArScanViewHandler.cs`
- Create (Windows): `src/Scanner.App/Platforms/Windows/ArPlatform.Windows.cs`, `Platforms/Windows/Handlers/UnsupportedViewHandlers.cs`

**Interfaces:**
- Consumes: `LiveScanSession`, `LiveScanState` (Task 5), `ScanSessionWriter` (Task 4), `ArCoreConversions` (Task 2); ARCore binding API exactly as recorded in `docs/arcore-binding-notes.md` / `Platforms/Android/ArCoreApiProbe.cs`.
- Produces:
  - `sealed class ArScanView : View` — bindable `LiveScanSession? Session`; `event EventHandler<ArScanStatus>? StatusChanged`; `void Resume()`, `void Pause()` (forwarded to the handler through the command mapper); `internal void ReportStatus(ArScanStatus status)`.
  - `sealed record ArScanStatus(string Tracking, LiveScanState State, int PointCount, int FrameCount, string? Message)`
  - `static partial class ArPlatform { static partial Task<string?> PrepareAsync(); }` — null when scanning can start, otherwise a user-facing reason.
  - `sealed class SessionStore` — `string Root`, `(string Id, string Directory) CreateNew()`, `string DirectoryOf(string id)`, `IReadOnlyList<ScanManifest> List()`, `void Delete(string id)` (best effort). (Sharing is added in Task 8.)
  - `sealed class ScanPage : ContentPage` (DI: `SessionStore`), temporarily the Shell root.
  - Android internals: `GlUtil.CreateProgram`, `GlUtil.ToBuffer`, `CameraBackgroundRenderer { int TextureId; void Initialize(); void Draw(Frame frame); }`, `PointCloudRenderer { void Initialize(); void Upload(IReadOnlyList<Vector3> points); void Draw(float[] viewProjectionColumnMajor, float pointSize); }` (reused by Task 7).

**Naming rule:** Android code lives in namespaces `Scanner.App.Droid.*` (never `Scanner.App.Platforms.Android.*` — a namespace segment named `Android` shadows the global `Android.*` namespaces inside it). Alias ARCore's `Frame` (`using ArFrame = Google.AR.Core.Frame;`) and Android's `Image` where they collide with MAUI types, exactly as the probe does.

**Binding rule:** the code below uses the API names verified by the spike. Where it calls something the probe did not pin (`Frame.HitTest`, `HitResult.HitPose`, `Pose.Tx()/Ty()/Tz()`, `Session.GetAllTrackables`, `Plane` type/tracking/center pose, `Config.SetPlaneFindingMode`, `Frame.HasDisplayGeometryChanged`, `Frame.Timestamp`, `Image.Timestamp`, `Coordinates2d.OpenglNormalizedDeviceCoordinates`), keep the semantics and adapt only the member spelling until it compiles; record every adaptation in the task report and append it to `docs/arcore-binding-notes.md`.

- [ ] **Step 1: Reference the capture library and lock portrait**

```bash
dotnet add src/Scanner.App reference src/Scanner.Capture
```

In `MainActivity.cs` add `ScreenOrientation = ScreenOrientation.Portrait` to the `[Activity(...)]` attribute (display rotation is then always 0, which the AR renderer relies on).

- [ ] **Step 2: Cross-platform control, status, platform preparation, session store**

`src/Scanner.App/Controls/ArScanView.cs`:

```csharp
using Scanner.Capture.Live;

namespace Scanner.App.Controls;

/// <summary>Snapshot of the AR scan for the UI, raised a few times per second.</summary>
public sealed record ArScanStatus(string Tracking, LiveScanState State, int PointCount, int FrameCount, string? Message);

/// <summary>Full-screen AR camera view that feeds depth frames into <see cref="Session"/> and draws its points.</summary>
public sealed class ArScanView : View
{
    public static readonly BindableProperty SessionProperty =
        BindableProperty.Create(nameof(Session), typeof(LiveScanSession), typeof(ArScanView));

    public LiveScanSession? Session
    {
        get => (LiveScanSession?)GetValue(SessionProperty);
        set => SetValue(SessionProperty, value);
    }

    public event EventHandler<ArScanStatus>? StatusChanged;

    /// <summary>Starts or resumes the camera and AR tracking.</summary>
    public void Resume() => Handler?.Invoke(nameof(Resume));

    /// <summary>Pauses the camera and AR tracking (call when the page is hidden or the app is backgrounded).</summary>
    public void Pause() => Handler?.Invoke(nameof(Pause));

    internal void ReportStatus(ArScanStatus status) => StatusChanged?.Invoke(this, status);
}
```

`src/Scanner.App/Services/ArPlatform.cs`:

```csharp
namespace Scanner.App.Services;

public static partial class ArPlatform
{
    /// <summary>Requests camera permission and checks/installs ARCore.
    /// Returns null when scanning can start, otherwise a user-facing reason.</summary>
    public static partial Task<string?> PrepareAsync();
}
```

`src/Scanner.App/Platforms/Android/ArPlatform.Android.cs`:

```csharp
using Google.AR.Core;

namespace Scanner.App.Services;

public static partial class ArPlatform
{
    public static partial async Task<string?> PrepareAsync()
    {
        if (await Permissions.RequestAsync<Permissions.Camera>() != PermissionStatus.Granted)
            return "Camera permission is required to scan.";

        var activity = Platform.CurrentActivity;
        if (activity is null) return "The app is not ready yet, try again.";

        var availability = ArCoreApk.Instance!.CheckAvailability(activity)!;
        for (int i = 0; i < 20 && availability.IsTransient; i++)
        {
            await Task.Delay(250);
            availability = ArCoreApk.Instance!.CheckAvailability(activity)!;
        }
        if (!availability.IsSupported) return "This device does not support ARCore.";

        try
        {
            var status = ArCoreApk.Instance!.RequestInstall(activity, true)!;
            if (status == ArCoreApk.InstallStatus.InstallRequested)
                return "Install or update Google Play Services for AR, then open the scan again.";
        }
        catch (Exception ex)
        {
            return $"ARCore is not available: {ex.Message}";
        }
        return null;
    }
}
```

`src/Scanner.App/Platforms/Windows/ArPlatform.Windows.cs`:

```csharp
namespace Scanner.App.Services;

public static partial class ArPlatform
{
    public static partial Task<string?> PrepareAsync() =>
        Task.FromResult<string?>("Scanning needs an Android phone with ARCore depth support.");
}
```

`src/Scanner.App/Services/SessionStore.cs`:

```csharp
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
```

- [ ] **Step 3: GL helpers and renderers (Android)**

`src/Scanner.App/Platforms/Android/Rendering/GlUtil.cs`:

```csharp
using Android.Opengl;
using Java.Nio;

namespace Scanner.App.Droid.Rendering;

internal static class GlUtil
{
    public static int CreateProgram(string vertexSource, string fragmentSource)
    {
        int vertex = Compile(GLES30.GlVertexShader, vertexSource);
        int fragment = Compile(GLES30.GlFragmentShader, fragmentSource);
        int program = GLES30.GlCreateProgram();
        GLES30.GlAttachShader(program, vertex);
        GLES30.GlAttachShader(program, fragment);
        GLES30.GlLinkProgram(program);
        var status = new int[1];
        GLES30.GlGetProgramiv(program, GLES30.GlLinkStatus, status, 0);
        GLES30.GlDeleteShader(vertex);
        GLES30.GlDeleteShader(fragment);
        if (status[0] == 0)
        {
            string log = GLES30.GlGetProgramInfoLog(program) ?? "";
            GLES30.GlDeleteProgram(program);
            throw new InvalidOperationException($"GL program link failed: {log}");
        }
        return program;
    }

    public static FloatBuffer ToBuffer(float[] data)
    {
        var bytes = ByteBuffer.AllocateDirect(data.Length * sizeof(float))!;
        bytes.Order(ByteOrder.NativeOrder()!);
        var floats = bytes.AsFloatBuffer()!;
        floats.Put(data);
        floats.Position(0);
        return floats;
    }

    private static int Compile(int type, string source)
    {
        int shader = GLES30.GlCreateShader(type);
        GLES30.GlShaderSource(shader, source);
        GLES30.GlCompileShader(shader);
        var status = new int[1];
        GLES30.GlGetShaderiv(shader, GLES30.GlCompileStatus, status, 0);
        if (status[0] == 0)
        {
            string log = GLES30.GlGetShaderInfoLog(shader) ?? "";
            GLES30.GlDeleteShader(shader);
            throw new InvalidOperationException($"GL shader compile failed: {log}");
        }
        return shader;
    }
}
```

`src/Scanner.App/Platforms/Android/Rendering/CameraBackgroundRenderer.cs`:

```csharp
using Android.Opengl;
using Google.AR.Core;
using Java.Nio;
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Rendering;

/// <summary>Draws the ARCore camera image (external OES texture) as a full-screen background.</summary>
internal sealed class CameraBackgroundRenderer
{
    private const int TextureExternalOes = 0x8D65; // GL_TEXTURE_EXTERNAL_OES
    private static readonly float[] QuadCoords = [-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f];

    private const string VertexShader = """
        #version 300 es
        layout(location = 0) in vec2 a_Position;
        layout(location = 1) in vec2 a_TexCoord;
        out vec2 v_TexCoord;
        void main() {
            gl_Position = vec4(a_Position, 0.0, 1.0);
            v_TexCoord = a_TexCoord;
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        #extension GL_OES_EGL_image_external_essl3 : require
        precision mediump float;
        uniform samplerExternalOES u_Texture;
        in vec2 v_TexCoord;
        out vec4 o_Color;
        void main() {
            o_Color = texture(u_Texture, v_TexCoord);
        }
        """;

    private readonly float[] _texCoords = new float[8];
    private FloatBuffer? _quad;
    private FloatBuffer? _tex;
    private int _program;
    private int _textureUniform;

    public int TextureId { get; private set; }

    /// <summary>Creates GL resources. Call on the GL thread from OnSurfaceCreated.</summary>
    public void Initialize()
    {
        var textures = new int[1];
        GLES30.GlGenTextures(1, textures, 0);
        TextureId = textures[0];
        GLES30.GlBindTexture(TextureExternalOes, TextureId);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureWrapS, GLES30.GlClampToEdge);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureWrapT, GLES30.GlClampToEdge);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureMinFilter, GLES30.GlLinear);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureMagFilter, GLES30.GlLinear);

        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _textureUniform = GLES30.GlGetUniformLocation(_program, "u_Texture");
        _quad = GlUtil.ToBuffer(QuadCoords);
        _tex = GlUtil.ToBuffer(new float[8]);
    }

    public void Draw(ArFrame frame)
    {
        if (frame.HasDisplayGeometryChanged)
        {
            frame.TransformCoordinates2d(Coordinates2d.OpenglNormalizedDeviceCoordinates!, QuadCoords,
                Coordinates2d.TextureNormalized!, _texCoords);
            _tex!.Position(0);
            _tex.Put(_texCoords);
            _tex.Position(0);
        }
        if (frame.Timestamp == 0) return; // ARCore has not produced a camera image yet.

        GLES30.GlDisable(GLES30.GlDepthTest);
        GLES30.GlDepthMask(false);
        GLES30.GlUseProgram(_program);
        GLES30.GlActiveTexture(GLES30.GlTexture0);
        GLES30.GlBindTexture(TextureExternalOes, TextureId);
        GLES30.GlUniform1i(_textureUniform, 0);
        GLES30.GlVertexAttribPointer(0, 2, GLES30.GlFloat, false, 0, _quad);
        GLES30.GlVertexAttribPointer(1, 2, GLES30.GlFloat, false, 0, _tex);
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlEnableVertexAttribArray(1);
        GLES30.GlDrawArrays(GLES30.GlTriangleStrip, 0, 4);
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlDisableVertexAttribArray(1);
        GLES30.GlDepthMask(true);
        GLES30.GlEnable(GLES30.GlDepthTest);
    }
}
```

`src/Scanner.App/Platforms/Android/Rendering/PointCloudRenderer.cs`:

```csharp
using System.Numerics;
using Android.Opengl;

namespace Scanner.App.Droid.Rendering;

/// <summary>Draws a point cloud as round GL points coloured by height (world +Y). All calls on the GL thread.</summary>
internal sealed class PointCloudRenderer
{
    private const string VertexShader = """
        #version 300 es
        uniform mat4 u_ViewProjection;
        uniform float u_PointSize;
        uniform vec2 u_HeightRange;
        layout(location = 0) in vec3 a_Position;
        out vec3 v_Color;
        vec3 heightColor(float t) {
            return clamp(vec3(1.5 - abs(4.0 * t - 3.0), 1.5 - abs(4.0 * t - 2.0), 1.5 - abs(4.0 * t - 1.0)), 0.0, 1.0);
        }
        void main() {
            gl_Position = u_ViewProjection * vec4(a_Position, 1.0);
            gl_PointSize = u_PointSize;
            float span = max(u_HeightRange.y - u_HeightRange.x, 1e-4);
            v_Color = heightColor(clamp((a_Position.y - u_HeightRange.x) / span, 0.0, 1.0));
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        precision mediump float;
        in vec3 v_Color;
        out vec4 o_Color;
        void main() {
            vec2 c = gl_PointCoord * 2.0 - 1.0;
            if (dot(c, c) > 1.0) discard;
            o_Color = vec4(v_Color, 1.0);
        }
        """;

    private int _program;
    private int _buffer;
    private int _count;
    private int _viewProjectionUniform;
    private int _pointSizeUniform;
    private int _heightRangeUniform;
    private float _minY;
    private float _maxY;

    public void Initialize()
    {
        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _viewProjectionUniform = GLES30.GlGetUniformLocation(_program, "u_ViewProjection");
        _pointSizeUniform = GLES30.GlGetUniformLocation(_program, "u_PointSize");
        _heightRangeUniform = GLES30.GlGetUniformLocation(_program, "u_HeightRange");
        var buffers = new int[1];
        GLES30.GlGenBuffers(1, buffers, 0);
        _buffer = buffers[0];
        _count = 0;
    }

    public void Upload(IReadOnlyList<Vector3> points)
    {
        var data = new float[points.Count * 3];
        float minY = float.PositiveInfinity, maxY = float.NegativeInfinity;
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            data[3 * i] = p.X;
            data[3 * i + 1] = p.Y;
            data[3 * i + 2] = p.Z;
            minY = MathF.Min(minY, p.Y);
            maxY = MathF.Max(maxY, p.Y);
        }
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _buffer);
        GLES30.GlBufferData(GLES30.GlArrayBuffer, data.Length * sizeof(float), GlUtil.ToBuffer(data), GLES30.GlDynamicDraw);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
        _count = points.Count;
        _minY = points.Count > 0 ? minY : 0;
        _maxY = points.Count > 0 ? maxY : 1;
    }

    /// <param name="viewProjection">Column-major 4x4 matrix (GL convention).</param>
    public void Draw(float[] viewProjection, float pointSize)
    {
        if (_count == 0) return;
        GLES30.GlUseProgram(_program);
        GLES30.GlUniformMatrix4fv(_viewProjectionUniform, 1, false, viewProjection, 0);
        GLES30.GlUniform1f(_pointSizeUniform, pointSize);
        GLES30.GlUniform2f(_heightRangeUniform, _minY, _maxY);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _buffer);
        GLES30.GlVertexAttribPointer(0, 3, GLES30.GlFloat, false, 0, 0);
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlDrawArrays(GLES30.GlPoints, 0, _count);
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
    }
}
```

- [ ] **Step 4: Depth frame reader and AR renderer (Android)**

`src/Scanner.App/Platforms/Android/Ar/DepthFrameReader.cs`:

```csharp
using Google.AR.Core;
using Java.Nio;
using Scanner.Capture;
using Scanner.Capture.ArCore;
using AndroidImage = Android.Media.Image;
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Ar;

/// <summary>Copies ARCore raw depth (uint16 mm) and raw confidence into a <see cref="DepthFrame"/>. GL thread only.</summary>
internal static class DepthFrameReader
{
    /// <summary>Returns null when no fresh depth image matches this camera frame.</summary>
    /// <exception cref="Google.AR.Core.Exceptions.NotYetAvailableException">Depth is not available yet.</exception>
    public static (DepthFrame Frame, byte[] Confidence)? TryRead(ArFrame frame, Camera camera)
    {
        using AndroidImage depthImage = frame.AcquireRawDepthImage16Bits()!;
        using AndroidImage confidenceImage = frame.AcquireRawDepthConfidenceImage()!;
        if (depthImage.Timestamp != frame.Timestamp) return null; // stale depth: its pose would not match

        int width = depthImage.Width;
        int height = depthImage.Height;
        ushort[] millimeters = ReadUInt16(depthImage, width, height);
        byte[] confidence = ReadBytes(confidenceImage, width, height);

        var pose = new float[16];
        camera.Pose!.ToMatrix(pose, 0);
        var image = camera.ImageIntrinsics!;
        float[] focal = image.GetFocalLength()!;
        float[] principal = image.GetPrincipalPoint()!;
        int[] size = image.GetImageDimensions()!;
        var intrinsics = ArCoreConversions.ScaleIntrinsics(focal[0], focal[1], principal[0], principal[1],
            size[0], size[1], width, height);

        var depthFrame = ArCoreConversions.DepthFrameFromMillimeters(millimeters, intrinsics,
            ArCoreConversions.CameraToWorldFromGlPose(pose), frame.Timestamp / 1e9);
        return (depthFrame, confidence);
    }

    private static ushort[] ReadUInt16(AndroidImage image, int width, int height)
    {
        var plane = image.GetPlanes()![0]!;
        byte[] bytes = Copy(plane.Buffer!);
        int rowStride = plane.RowStride;
        var result = new ushort[width * height];
        for (int v = 0; v < height; v++)
        for (int u = 0; u < width; u++)
            result[v * width + u] = BitConverter.ToUInt16(bytes, v * rowStride + u * 2);
        return result;
    }

    private static byte[] ReadBytes(AndroidImage image, int width, int height)
    {
        var plane = image.GetPlanes()![0]!;
        byte[] bytes = Copy(plane.Buffer!);
        int rowStride = plane.RowStride;
        int pixelStride = plane.PixelStride;
        var result = new byte[width * height];
        for (int v = 0; v < height; v++)
        for (int u = 0; u < width; u++)
            result[v * width + u] = bytes[v * rowStride + u * pixelStride];
        return result;
    }

    private static byte[] Copy(ByteBuffer buffer)
    {
        buffer.Rewind();
        var bytes = new byte[buffer.Remaining()];
        buffer.Get(bytes);
        return bytes;
    }
}
```

`src/Scanner.App/Platforms/Android/Ar/ArScanRenderer.cs`:

```csharp
using System.Diagnostics;
using System.Numerics;
using Android.Opengl;
using Google.AR.Core;
using Google.AR.Core.Exceptions;
using Javax.Microedition.Khronos.Opengles;
using Scanner.App.Controls;
using Scanner.App.Droid.Rendering;
using Scanner.Capture.Live;
using ArFrame = Google.AR.Core.Frame;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Ar;

/// <summary>
/// GL-thread renderer for the scan screen: updates ARCore, draws the camera background and the live points,
/// picks the target when a scan starts, and hands depth frames to a worker thread for integration.
/// </summary>
internal sealed class ArScanRenderer : Java.Lang.Object, GLSurfaceView.IRenderer
{
    private const float PointSizePixels = 7f;
    private static readonly TimeSpan PointUploadInterval = TimeSpan.FromMilliseconds(300);
    private static readonly TimeSpan StatusInterval = TimeSpan.FromMilliseconds(250);

    private readonly Action<ArScanStatus> _reportStatus;
    private readonly CameraBackgroundRenderer _background = new();
    private readonly PointCloudRenderer _points = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly float[] _view = new float[16];
    private readonly float[] _projection = new float[16];
    private readonly float[] _viewProjection = new float[16];

    private volatile Session? _session;
    private volatile LiveScanSession? _scan;
    private volatile bool _geometryChanged;
    private int _width;
    private int _height;
    private int _integrating;
    private TimeSpan _lastUpload;
    private TimeSpan _lastStatus;
    private string? _message;

    public ArScanRenderer(Action<ArScanStatus> reportStatus) => _reportStatus = reportStatus;

    public void AttachSession(Session? session) => _session = session;

    public void SetScan(LiveScanSession? scan) => _scan = scan;

    public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
    {
        GLES30.GlClearColor(0f, 0f, 0f, 1f);
        _background.Initialize();
        _points.Initialize();
        _lastUpload = TimeSpan.Zero;
    }

    public void OnSurfaceChanged(IGL10? gl, int width, int height)
    {
        GLES30.GlViewport(0, 0, width, height);
        _width = width;
        _height = height;
        _geometryChanged = true;
    }

    public void OnDrawFrame(IGL10? gl)
    {
        GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
        var session = _session;
        if (session is null) return;

        var scan = _scan;
        string tracking = "Starting";
        try
        {
            if (_geometryChanged)
            {
                session.SetDisplayGeometry(0, _width, _height); // portrait-locked activity: rotation 0
                _geometryChanged = false;
            }
            session.SetCameraTextureName(_background.TextureId);
            ArFrame frame = session.Update()!;
            Camera camera = frame.Camera!;
            _background.Draw(frame);

            bool isTracking = camera.TrackingState!.Equals(TrackingState.Tracking!);
            tracking = isTracking ? "Tracking" : "Move the phone slowly";
            if (isTracking && scan is not null)
            {
                if (scan.State == LiveScanState.WaitingForTarget) TryPickTarget(session, frame, scan);
                if (scan.ShouldIntegrate(frame.Timestamp / 1e9)) TryIntegrate(frame, camera, scan);

                if (_clock.Elapsed - _lastUpload >= PointUploadInterval)
                {
                    _points.Upload(scan.SnapshotPoints());
                    _lastUpload = _clock.Elapsed;
                }
                camera.GetViewMatrix(_view, 0);
                camera.GetProjectionMatrix(_projection, 0, 0.05f, 20f);
                Android.Opengl.Matrix.MultiplyMM(_viewProjection, 0, _projection, 0, _view, 0);
                _points.Draw(_viewProjection, PointSizePixels);
            }
        }
        catch (Exception ex)
        {
            _message = ex.Message;
        }

        if (_clock.Elapsed - _lastStatus >= StatusInterval)
        {
            _lastStatus = _clock.Elapsed;
            _reportStatus(new ArScanStatus(tracking, scan?.State ?? LiveScanState.Idle,
                scan?.PointCount ?? 0, scan?.FrameCount ?? 0, _message));
            _message = null;
        }
    }

    // The target is the depth/plane hit under the screen centre (the crosshair); the support plane is the highest
    // tracked upward-facing horizontal plane below it.
    private void TryPickTarget(Session session, ArFrame frame, LiveScanSession scan)
    {
        var hit = frame.HitTest(_width / 2f, _height / 2f)?.FirstOrDefault();
        if (hit is null)
        {
            _message = "Aim the crosshair at the piece";
            return;
        }
        var pose = hit.HitPose!;
        var target = new Vector3(pose.Tx(), pose.Ty(), pose.Tz());

        float? planeHeight = null;
        foreach (var trackable in session.GetAllTrackables(Java.Lang.Class.FromType(typeof(Plane)))!)
        {
            if (trackable is not Plane plane) continue;
            if (!plane.TrackingState!.Equals(TrackingState.Tracking!)) continue;
            if (!plane.GetType_()!.Equals(Plane.Type.HorizontalUpwardFacing!)) continue;
            float y = plane.CenterPose!.Ty();
            if (y < target.Y - 0.005f && (planeHeight is null || y > planeHeight)) planeHeight = y;
        }

        if (scan.State == LiveScanState.WaitingForTarget) scan.SetTarget(target, planeHeight);
    }

    private void TryIntegrate(ArFrame frame, Camera camera, LiveScanSession scan)
    {
        if (Interlocked.CompareExchange(ref _integrating, 1, 0) != 0) return; // previous frame still integrating

        (Scanner.Capture.DepthFrame Frame, byte[] Confidence)? data;
        try
        {
            data = DepthFrameReader.TryRead(frame, camera);
        }
        catch (NotYetAvailableException)
        {
            data = null;
        }
        catch
        {
            Volatile.Write(ref _integrating, 0);
            throw;
        }

        if (data is not { } depth)
        {
            Volatile.Write(ref _integrating, 0);
            return;
        }

        Task.Run(() =>
        {
            try
            {
                scan.Integrate(depth.Frame, depth.Confidence);
            }
            catch (Exception ex)
            {
                _message = $"Integration failed: {ex.Message}";
            }
            finally
            {
                Volatile.Write(ref _integrating, 0);
            }
        });
    }
}
```

(`plane.GetType_()` is the conventional binding name for Java `Plane.getType()`, which clashes with `object.GetType()`; use whatever the binding exposes and note it.)

- [ ] **Step 5: Android handler**

`src/Scanner.App/Platforms/Android/Handlers/ArScanViewHandler.cs`:

```csharp
using Android.Opengl;
using Google.AR.Core;
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.App.Droid.Ar;

namespace Scanner.App.Droid.Handlers;

public sealed class ArScanViewHandler : ViewHandler<ArScanView, GLSurfaceView>
{
    public static readonly IPropertyMapper<ArScanView, ArScanViewHandler> PropertyMapper =
        new PropertyMapper<ArScanView, ArScanViewHandler>(ViewMapper)
        {
            [nameof(ArScanView.Session)] = (handler, view) => handler._renderer?.SetScan(view.Session),
        };

    public static readonly CommandMapper<ArScanView, ArScanViewHandler> CommandMapper =
        new(ViewCommandMapper)
        {
            [nameof(ArScanView.Resume)] = (handler, _, _) => handler.ResumeAr(),
            [nameof(ArScanView.Pause)] = (handler, _, _) => handler.PauseAr(),
        };

    private ArScanRenderer? _renderer;
    private Session? _session;

    public ArScanViewHandler() : base(PropertyMapper, CommandMapper)
    {
    }

    protected override GLSurfaceView CreatePlatformView()
    {
        var view = new GLSurfaceView(Context) { PreserveEGLContextOnPause = true };
        view.SetEGLContextClientVersion(3);
        view.SetEGLConfigChooser(8, 8, 8, 8, 16, 0);
        _renderer = new ArScanRenderer(status =>
            MainThread.BeginInvokeOnMainThread(() => VirtualView?.ReportStatus(status)));
        view.SetRenderer(_renderer);
        view.RenderMode = Rendermode.Continuously;
        return view;
    }

    protected override void ConnectHandler(GLSurfaceView platformView)
    {
        base.ConnectHandler(platformView);
        _renderer?.SetScan(VirtualView.Session);
    }

    protected override void DisconnectHandler(GLSurfaceView platformView)
    {
        PauseAr();
        _renderer?.AttachSession(null);
        _session?.Close();
        _session = null;
        base.DisconnectHandler(platformView);
    }

    private void ResumeAr()
    {
        try
        {
            _session ??= CreateSession();
            _renderer?.AttachSession(_session);
            _session.Resume();
            PlatformView.OnResume();
        }
        catch (Exception ex)
        {
            VirtualView?.ReportStatus(new ArScanStatus("Unavailable", VirtualView.Session?.State ?? default, 0, 0, ex.Message));
        }
    }

    private void PauseAr()
    {
        PlatformView?.OnPause();
        _session?.Pause();
    }

    private Session CreateSession()
    {
        var session = new Session(Context);
        if (!session.IsDepthModeSupported(Config.DepthMode.Automatic!))
        {
            session.Close();
            throw new NotSupportedException("This device does not support ARCore depth, which scanning requires.");
        }
        var config = new Config(session);
        config.SetDepthMode(Config.DepthMode.Automatic!);
        config.SetFocusMode(Config.FocusMode.Auto!);
        config.SetPlaneFindingMode(Config.PlaneFindingMode.Horizontal!);
        session.Configure(config);
        return session;
    }
}
```

- [ ] **Step 6: Windows placeholder handler**

`src/Scanner.App/Platforms/Windows/Handlers/UnsupportedViewHandlers.cs`:

```csharp
using Microsoft.Maui.Handlers;
using Microsoft.UI.Xaml.Controls;
using Scanner.App.Controls;

namespace Scanner.App.WinUI.Handlers;

/// <summary>Shows a short message where an Android-only view would be.</summary>
public sealed class ArScanViewHandler() : ViewHandler<ArScanView, TextBlock>(ViewMapper)
{
    protected override TextBlock CreatePlatformView() =>
        new() { Text = "AR scanning is available on Android only.", Margin = new Microsoft.UI.Xaml.Thickness(16) };
}
```

- [ ] **Step 7: Scan page, handler registration, Shell root**

`src/Scanner.App/Pages/ScanPage.cs`:

```csharp
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Live;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

public sealed class ScanPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly ArScanView _arView = new();
    private readonly Label _status = new() { TextColor = Colors.White, FontSize = 14 };
    private readonly Button _startPause = new() { Text = "Start" };
    private readonly Button _finish = new() { Text = "Finish", IsEnabled = false };
    private LiveScanSession? _scan;
    private string? _sessionId;

    public ScanPage(SessionStore store)
    {
        _store = store;
        Title = "Scan";
        _startPause.Clicked += OnStartPauseClicked;
        _finish.Clicked += OnFinishClicked;
        _arView.StatusChanged += OnStatusChanged;

        var crosshair = new Label
        {
            Text = "+", FontSize = 40, TextColor = Colors.White, InputTransparent = true,
            HorizontalOptions = LayoutOptions.Center, VerticalOptions = LayoutOptions.Center,
        };
        var panel = new VerticalStackLayout
        {
            Padding = 12, Spacing = 8, VerticalOptions = LayoutOptions.End,
            BackgroundColor = Color.FromRgba(0, 0, 0, 0.55),
            Children = { _status, new HorizontalStackLayout { Spacing = 12, Children = { _startPause, _finish } } },
        };
        Content = new Grid { Children = { _arView, crosshair, panel } };
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        if (_scan is null)
        {
            string? error = await ArPlatform.PrepareAsync();
            if (error is not null)
            {
                _status.Text = error;
                _startPause.IsEnabled = false;
                return;
            }
            var (id, directory) = _store.CreateNew();
            _sessionId = id;
            _scan = new LiveScanSession(new ScanSessionWriter(directory, id, DeviceInfo.Current.Model));
            _arView.Session = _scan;
            _status.Text = "Aim the crosshair at the piece and press Start.";
        }
        if (Window is { } window)
        {
            window.Stopped += OnWindowStopped;
            window.Resumed += OnWindowResumed;
        }
        _arView.Resume();
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        if (Window is { } window)
        {
            window.Stopped -= OnWindowStopped;
            window.Resumed -= OnWindowResumed;
        }
        _arView.Pause();

        // Leaving without finishing discards the unfinished session.
        if (_scan is not null && _scan.State != LiveScanState.Completed && _sessionId is not null)
        {
            _arView.Session = null;
            _scan.Pause();
            _store.Delete(_sessionId);
            _scan = null;
            _sessionId = null;
        }
    }

    private void OnWindowStopped(object? sender, EventArgs e) => _arView.Pause();

    private void OnWindowResumed(object? sender, EventArgs e) => _arView.Resume();

    private void OnStartPauseClicked(object? sender, EventArgs e)
    {
        if (_scan is null) return;
        if (_scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget) _scan.Pause();
        else _scan.RequestStart();
        UpdateButtons();
    }

    private async void OnFinishClicked(object? sender, EventArgs e)
    {
        if (_scan is null || _sessionId is null) return;
        _finish.IsEnabled = false;
        _startPause.IsEnabled = false;
        _status.Text = "Isolating the piece…";
        _scan.Pause();
        var result = await Task.Run(_scan.Complete);
        await DisplayAlertAsync("Scan complete",
            $"{result.PiecePoints.Length:N0} points{(result.Isolated ? " (piece isolated from the table)" : "")}.", "OK");
    }

    private void OnStatusChanged(object? sender, ArScanStatus status)
    {
        if (_scan is null || _scan.State == LiveScanState.Completed) return;
        _status.Text = $"{status.Tracking} · {status.State} · {status.PointCount:N0} points · {status.FrameCount} frames"
                       + (status.Message is { } message ? $"\n{message}" : "");
        UpdateButtons();
    }

    private void UpdateButtons()
    {
        if (_scan is null) return;
        bool running = _scan.State is LiveScanState.Recording or LiveScanState.WaitingForTarget;
        _startPause.Text = running ? "Pause" : _scan.FrameCount > 0 ? "Resume" : "Start";
        _finish.IsEnabled = _scan.FrameCount > 0 && _scan.State != LiveScanState.Completed;
    }
}
```

`src/Scanner.App/MauiProgram.cs` — inside `CreateMauiApp`, after `UseMauiApp<App>()`:

```csharp
builder.ConfigureMauiHandlers(handlers =>
{
#if ANDROID
    handlers.AddHandler<Scanner.App.Controls.ArScanView, Scanner.App.Droid.Handlers.ArScanViewHandler>();
#elif WINDOWS
    handlers.AddHandler<Scanner.App.Controls.ArScanView, Scanner.App.WinUI.Handlers.ArScanViewHandler>();
#endif
});
builder.Services.AddSingleton<Scanner.App.Services.SessionStore>();
builder.Services.AddTransient<Scanner.App.Pages.ScanPage>();
```

`src/Scanner.App/AppShell.xaml` — replace the template's `MainPage` `ShellContent` with:

```xml
<ShellContent Title="Scan" ContentTemplate="{DataTemplate pages:ScanPage}" Route="scan" />
```

with `xmlns:pages="clr-namespace:Scanner.App.Pages"` on the `Shell` element. Delete `MainPage.xaml` and `MainPage.xaml.cs`.

- [ ] **Step 8: Build both targets**

Run: `dotnet build src/Scanner.App -f net10.0-android -c Debug` and `dotnet build src/Scanner.App -f net10.0-windows10.0.19041.0 -c Debug`
Expected: both succeed with 0 warnings. Fix binding member spellings as described in the Binding rule; replace any API that .NET 10 MAUI marks obsolete (e.g. use `DisplayAlertAsync`) rather than suppressing the warning.

- [ ] **Step 9: Deploy if a device is attached (otherwise skip and say so)**

Run: `"/c/Program Files (x86)/Android/android-sdk/platform-tools/adb.exe" devices`
If a device is listed: `dotnet build src/Scanner.App -t:Run -f net10.0-android -c Debug` and confirm the app starts to the scan screen (camera permission prompt, then camera image). Record the outcome in the report.

- [ ] **Step 10: Commit**

```bash
git add src/Scanner.App docs/arcore-binding-notes.md
git status --short   # verify no bin/ obj/ and nothing outside these paths
git commit -m "feat(app): AR scan screen with live point overlay and ARCore depth capture" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: 3D preview of the scanned piece (orbit viewer)

**Files:**
- Create: `src/Scanner.App/Controls/PointCloudView.cs`
- Create: `src/Scanner.App/Controls/OrbitCamera.cs`
- Create (Android): `src/Scanner.App/Platforms/Android/Viewer/PointCloudGlView.cs`, `Platforms/Android/Viewer/OrbitRenderer.cs`, `Platforms/Android/Handlers/PointCloudViewHandler.cs`
- Modify (Windows): `src/Scanner.App/Platforms/Windows/Handlers/UnsupportedViewHandlers.cs` (add `PointCloudViewHandler`)
- Create: `src/Scanner.App/Pages/PreviewPage.cs`
- Modify: `src/Scanner.App/MauiProgram.cs`, `src/Scanner.App/AppShell.xaml.cs`, `src/Scanner.App/Pages/ScanPage.cs`

**Interfaces:**
- Consumes: `PointCloudRenderer` (Task 6), `SessionStore`, `ScanSessionReader`.
- Produces:
  - `sealed class PointCloudView : View` with bindable `Vector3[]? Points`.
  - `sealed class OrbitCamera` — `void Frame(IReadOnlyList<Vector3> points)`, `void Rotate(float dxPixels, float dyPixels)`, `void Zoom(float scaleFactor)`, `float[] ViewProjection(float aspect)` (column-major for GL).
  - `sealed class PreviewPage : ContentPage` — Shell route `preview`, query parameter `id` (session id).
  - `ScanPage` Finish now navigates to `preview?id=<id>` instead of showing an alert.

- [ ] **Step 1: Cross-platform view and orbit camera**

`src/Scanner.App/Controls/PointCloudView.cs`:

```csharp
using System.Numerics;

namespace Scanner.App.Controls;

/// <summary>Interactive 3D view of a point cloud: drag to orbit, pinch to zoom.</summary>
public sealed class PointCloudView : View
{
    public static readonly BindableProperty PointsProperty =
        BindableProperty.Create(nameof(Points), typeof(Vector3[]), typeof(PointCloudView));

    public Vector3[]? Points
    {
        get => (Vector3[]?)GetValue(PointsProperty);
        set => SetValue(PointsProperty, value);
    }
}
```

`src/Scanner.App/Controls/OrbitCamera.cs`:

```csharp
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
```

- [ ] **Step 2: Android GL view, renderer, handler**

`src/Scanner.App/Platforms/Android/Viewer/OrbitRenderer.cs`:

```csharp
using System.Numerics;
using Android.Opengl;
using Javax.Microedition.Khronos.Opengles;
using Scanner.App.Controls;
using Scanner.App.Droid.Rendering;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Viewer;

/// <summary>GL-thread renderer for the 3D preview. Mutate <see cref="Camera"/> only through GLSurfaceView.QueueEvent.</summary>
internal sealed class OrbitRenderer : Java.Lang.Object, GLSurfaceView.IRenderer
{
    private const float PointSizePixels = 6f;

    private readonly PointCloudRenderer _points = new();
    private Vector3[] _data = [];
    private float _aspect = 1f;

    public OrbitCamera Camera { get; } = new();

    /// <summary>GL thread.</summary>
    public void SetPoints(Vector3[]? points)
    {
        _data = points ?? [];
        _points.Upload(_data);
        Camera.Frame(_data);
    }

    public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
    {
        GLES30.GlClearColor(0.11f, 0.11f, 0.13f, 1f);
        GLES30.GlEnable(GLES30.GlDepthTest);
        _points.Initialize();
        _points.Upload(_data); // the EGL context may have been recreated
    }

    public void OnSurfaceChanged(IGL10? gl, int width, int height)
    {
        GLES30.GlViewport(0, 0, width, height);
        _aspect = height > 0 ? (float)width / height : 1f;
    }

    public void OnDrawFrame(IGL10? gl)
    {
        GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
        _points.Draw(Camera.ViewProjection(_aspect), PointSizePixels);
    }
}
```

`src/Scanner.App/Platforms/Android/Viewer/PointCloudGlView.cs`:

```csharp
using System.Numerics;
using Android.Content;
using Android.Opengl;
using Android.Views;

namespace Scanner.App.Droid.Viewer;

/// <summary>GLSurfaceView with one-finger orbit and pinch zoom.</summary>
internal sealed class PointCloudGlView : GLSurfaceView
{
    private readonly OrbitRenderer _renderer = new();
    private readonly ScaleGestureDetector _scaleDetector;
    private float _lastX;
    private float _lastY;

    public PointCloudGlView(Context context) : base(context)
    {
        SetEGLContextClientVersion(3);
        SetEGLConfigChooser(8, 8, 8, 8, 16, 0);
        SetRenderer(_renderer);
        RenderMode = Rendermode.WhenDirty;
        _scaleDetector = new ScaleGestureDetector(context, new ScaleListener(this));
    }

    public void SetPoints(Vector3[]? points)
    {
        QueueEvent(() => _renderer.SetPoints(points));
        RequestRender();
    }

    public override bool OnTouchEvent(MotionEvent? e)
    {
        if (e is null) return false;
        _scaleDetector.OnTouchEvent(e);
        if (e.PointerCount == 1 && !_scaleDetector.IsInProgress)
        {
            switch (e.ActionMasked)
            {
                case MotionEventActions.Down:
                    _lastX = e.GetX();
                    _lastY = e.GetY();
                    break;
                case MotionEventActions.Move:
                    float dx = e.GetX() - _lastX, dy = e.GetY() - _lastY;
                    _lastX = e.GetX();
                    _lastY = e.GetY();
                    QueueEvent(() => _renderer.Camera.Rotate(dx, dy));
                    RequestRender();
                    break;
            }
        }
        else if (e.ActionMasked == MotionEventActions.PointerUp && e.PointerCount == 2)
        {
            // Continue rotating smoothly with the finger that stays down.
            int remaining = e.ActionIndex == 0 ? 1 : 0;
            _lastX = e.GetX(remaining);
            _lastY = e.GetY(remaining);
        }
        return true;
    }

    private sealed class ScaleListener(PointCloudGlView owner) : ScaleGestureDetector.SimpleOnScaleGestureListener
    {
        public override bool OnScale(ScaleGestureDetector? detector)
        {
            if (detector is null) return false;
            float factor = detector.ScaleFactor;
            owner.QueueEvent(() => owner._renderer.Camera.Zoom(factor));
            owner.RequestRender();
            return true;
        }
    }
}
```

`src/Scanner.App/Platforms/Android/Handlers/PointCloudViewHandler.cs`:

```csharp
using Microsoft.Maui.Handlers;
using Scanner.App.Controls;
using Scanner.App.Droid.Viewer;

namespace Scanner.App.Droid.Handlers;

public sealed class PointCloudViewHandler : ViewHandler<PointCloudView, PointCloudGlView>
{
    public static readonly IPropertyMapper<PointCloudView, PointCloudViewHandler> PropertyMapper =
        new PropertyMapper<PointCloudView, PointCloudViewHandler>(ViewMapper)
        {
            [nameof(PointCloudView.Points)] = (handler, view) => handler.PlatformView.SetPoints(view.Points),
        };

    public PointCloudViewHandler() : base(PropertyMapper)
    {
    }

    protected override PointCloudGlView CreatePlatformView() => new(Context);
}
```

Note: `PointCloudGlView` is `internal`; if the handler's generic base requires an accessible type, make `PointCloudGlView` `public sealed` (and `OrbitRenderer` stays internal).

Append to `src/Scanner.App/Platforms/Windows/Handlers/UnsupportedViewHandlers.cs`:

```csharp
public sealed class PointCloudViewHandler() : ViewHandler<PointCloudView, TextBlock>(ViewMapper)
{
    protected override TextBlock CreatePlatformView() =>
        new() { Text = "The 3D preview is available on Android only for now.", Margin = new Microsoft.UI.Xaml.Thickness(16) };
}
```

- [ ] **Step 3: Preview page, route, navigation from the scan**

`src/Scanner.App/Pages/PreviewPage.cs`:

```csharp
using System.Numerics;
using Scanner.App.Controls;
using Scanner.App.Services;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

[QueryProperty(nameof(SessionId), "id")]
public sealed class PreviewPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly PointCloudView _view = new();
    private readonly Label _info = new() { FontSize = 14 };
    private string? _loadedId;

    public PreviewPage(SessionStore store)
    {
        _store = store;
        Title = "Preview";
        var panel = new VerticalStackLayout { Padding = 12, Spacing = 8, Children = { _info } };
        Content = new Grid
        {
            RowDefinitions = { new RowDefinition(GridLength.Star), new RowDefinition(GridLength.Auto) },
            Children = { _view, panel },
        };
        Grid.SetRow(panel, 1);
    }

    public string? SessionId { get; set; }

    /// <summary>Extra row for action buttons (Task 8).</summary>
    protected VerticalStackLayout Panel => (VerticalStackLayout)((Grid)Content).Children[1];

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        if (SessionId is null || SessionId == _loadedId) return;
        _loadedId = SessionId;
        string directory = _store.DirectoryOf(SessionId);
        _info.Text = "Loading…";
        try
        {
            var (manifest, points) = await Task.Run(() =>
                (ScanSessionReader.ReadManifest(directory), ScanSessionReader.ReadPoints(directory)));
            _view.Points = points;
            _info.Text = $"{points.Length:N0} points · {manifest.FrameCount} frames · {manifest.CreatedUtc.LocalDateTime:g}\n"
                         + "Drag to rotate, pinch to zoom.";
        }
        catch (Exception ex) when (ex is IOException or InvalidDataException or System.Text.Json.JsonException)
        {
            _info.Text = $"Could not load the scan: {ex.Message}";
        }
    }
}
```

(Remove the `Panel` helper if unused after Task 8 — Task 8 adds buttons to the same stack; keep `panel` as a field instead if that is simpler. `sealed` classes cannot have `protected` members: make it `private` or a field.)

`AppShell.xaml.cs` constructor, after `InitializeComponent()`:

```csharp
Routing.RegisterRoute("preview", typeof(Scanner.App.Pages.PreviewPage));
```

`MauiProgram.cs`: add `handlers.AddHandler<Scanner.App.Controls.PointCloudView, ...PointCloudViewHandler>();` in both platform branches and `builder.Services.AddTransient<Scanner.App.Pages.PreviewPage>();`.

`ScanPage.OnFinishClicked`: replace the `DisplayAlertAsync(...)` call with:

```csharp
_status.Text = result.Isolated ? "Piece isolated from the table." : "Could not isolate the piece; showing all points.";
await Shell.Current.GoToAsync($"preview?id={_sessionId}");
```

- [ ] **Step 4: Build both targets, deploy if a device is attached**

Run: `dotnet build src/Scanner.App -f net10.0-android -c Debug` and the windows TFM — 0 warnings. If `adb devices` lists a device, `dotnet build src/Scanner.App -t:Run -f net10.0-android -c Debug`, scan something, press Finish and confirm the preview rotates/zooms. Record the outcome.

- [ ] **Step 5: Commit**

```bash
git add src/Scanner.App
git status --short
git commit -m "feat(app): 3D orbit preview of the scanned piece" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Sessions list, share/export, delete, run instructions

**Files:**
- Create: `src/Scanner.App/Pages/SessionsPage.cs`
- Modify: `src/Scanner.App/Services/SessionStore.cs` (sharing), `src/Scanner.App/Pages/PreviewPage.cs` (buttons), `src/Scanner.App/Pages/ScanPage.cs` (relative navigation), `src/Scanner.App/AppShell.xaml` (+ `.cs`), `src/Scanner.App/MauiProgram.cs`
- Modify: `Scan3D.slnx` (add the app)
- Create: `docs/running-on-android.md`

**Interfaces:**
- Consumes: everything above.
- Produces: `enum SessionShareKind { Ply, ScanArchive }`; `Task SessionStore.ShareAsync(string id, SessionShareKind kind)`; Shell root = `SessionsPage` (route `sessions`), pushed routes `scan` and `preview`.

- [ ] **Step 1: Sharing in the store**

Add to `SessionStore.cs`:

```csharp
public enum SessionShareKind { Ply, ScanArchive }
```

(outside the class, same namespace) and inside the class:

```csharp
/// <summary>Copies the point cloud (PLY) or zips the whole session (.scan) into the cache and opens the share sheet.</summary>
public async Task ShareAsync(string id, SessionShareKind kind)
{
    string source = DirectoryOf(id);
    string file;
    if (kind == SessionShareKind.Ply)
    {
        file = Path.Combine(FileSystem.Current.CacheDirectory, $"scan3d-{id}.ply");
        File.Copy(Path.Combine(source, "points.ply"), file, overwrite: true);
    }
    else
    {
        file = Path.Combine(FileSystem.Current.CacheDirectory, $"scan3d-{id}.scan");
        await Task.Run(() => ScanArchive.Export(source, file));
    }
    await Share.Default.RequestAsync(new ShareFileRequest { Title = "Share scan", File = new ShareFile(file) });
}
```

- [ ] **Step 2: Sessions page**

`src/Scanner.App/Pages/SessionsPage.cs`:

```csharp
using Scanner.App.Services;
using Scanner.Capture.Sessions;

namespace Scanner.App.Pages;

public sealed class SessionsPage : ContentPage
{
    private readonly SessionStore _store;
    private readonly CollectionView _list;

    public SessionsPage(SessionStore store)
    {
        _store = store;
        Title = "Scans";
        ToolbarItems.Add(new ToolbarItem("New scan", null, async () => await Shell.Current.GoToAsync("scan")));

        _list = new CollectionView
        {
            SelectionMode = SelectionMode.Single,
            EmptyView = new Label
            {
                Text = "No scans yet. Tap \"New scan\", aim at the piece, press Start and walk slowly around it.",
                Margin = 24, HorizontalTextAlignment = TextAlignment.Center,
            },
            ItemTemplate = new DataTemplate(() =>
            {
                var date = new Label { FontSize = 16, FontAttributes = FontAttributes.Bold };
                date.SetBinding(Label.TextProperty, static (ScanManifest m) => m.CreatedUtc.LocalDateTime, stringFormat: "{0:g}");
                var stats = new Label { FontSize = 13 };
                stats.SetBinding(Label.TextProperty, static (ScanManifest m) => m.PointCount, stringFormat: "{0:N0} points");
                return new VerticalStackLayout { Padding = new Thickness(16, 10), Children = { date, stats } };
            }),
        };
        _list.SelectionChanged += OnSelectionChanged;
        Content = _list;
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();
        _list.SelectedItem = null;
        _list.ItemsSource = _store.List();
    }

    private async void OnSelectionChanged(object? sender, SelectionChangedEventArgs e)
    {
        if (e.CurrentSelection.FirstOrDefault() is ScanManifest manifest)
            await Shell.Current.GoToAsync($"preview?id={manifest.Id}");
    }
}
```

(If the compiled-binding `SetBinding` overload with a lambda is unavailable, use `SetBinding(Label.TextProperty, new Binding(nameof(ScanManifest.PointCount), stringFormat: "{0:N0} points"))` — no warnings allowed either way.)

- [ ] **Step 3: Preview actions**

In `PreviewPage`, add three buttons under the info label: **Share PLY** → `_store.ShareAsync(id, SessionShareKind.Ply)`, **Share .scan** → `SessionShareKind.ScanArchive`, **Delete** → `DisplayAlertAsync("Delete scan", "Delete this scan permanently?", "Delete", "Cancel")`, then `_store.Delete(id)` and `Shell.Current.GoToAsync("..")`. Disable the buttons while an action runs; show exceptions from sharing in the info label.

- [ ] **Step 4: Shell and navigation**

`AppShell.xaml`: root `ShellContent` becomes `<ShellContent Title="Scans" ContentTemplate="{DataTemplate pages:SessionsPage}" Route="sessions" />`.
`AppShell.xaml.cs`: register both routes: `Routing.RegisterRoute("scan", typeof(ScanPage)); Routing.RegisterRoute("preview", typeof(PreviewPage));`.
`MauiProgram.cs`: `builder.Services.AddTransient<SessionsPage>();`.
`ScanPage.OnFinishClicked`: navigate with `await Shell.Current.GoToAsync($"../preview?id={_sessionId}");` so Back from the preview returns to the list, not to the finished scan.

- [ ] **Step 5: Add the app to the solution and write run instructions**

```bash
dotnet sln add src/Scanner.App
```

`docs/running-on-android.md`:

```markdown
# Running Scan3D on an Android phone

Requirements: an Android phone that supports ARCore **with the Depth API**
(see Google's list of ARCore supported devices), a USB cable, and this PC with the .NET 10 SDK and MAUI workloads.

1. On the phone: Settings → About phone → tap "Build number" 7 times; then Settings → Developer options → enable **USB debugging**.
2. Connect the phone and accept the "Allow USB debugging" prompt.
3. Check it is visible: `"C:\Program Files (x86)\Android\android-sdk\platform-tools\adb.exe" devices`
4. From the repository root: `dotnet build src/Scanner.App -t:Run -f net10.0-android -c Debug`
   (first build/deploy takes a few minutes).

## Scanning a piece
1. Put the piece on a table with some texture around it, in good light.
2. Tap **New scan**, allow the camera, move the phone slowly until the status says *Tracking*.
3. Aim the crosshair at the piece and tap **Start**. Walk slowly around it at 30–60 cm, also from above.
   Coloured points appear as depth is captured.
4. Tap **Finish**: the table is removed and the 3D preview opens (drag to rotate, pinch to zoom).
5. **Share PLY** exports the point cloud (open it with MeshLab/CloudCompare); **Share .scan** exports the raw
   session for the desktop reconstruction pipeline.
```

- [ ] **Step 6: Build, deploy if possible, commit**

Run: `dotnet build src/Scanner.App -f net10.0-android -c Debug` and the windows TFM — 0 warnings. If a device is attached, deploy with `-t:Run` and exercise: list → New scan → Start → Finish → preview → Share PLY → Back → list shows the scan → Delete. Record the outcome.

```bash
git add src/Scanner.App Scan3D.slnx docs/running-on-android.md
git status --short
git commit -m "feat(app): sessions list, share/export and delete; run instructions" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```
