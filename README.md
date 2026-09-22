# Scan3D

A .NET 10 / .NET MAUI app that turns an Android phone into a depth scanner, with the goal of producing
**STEP (B-Rep) solids that open as editable bodies in CAD tools such as Autodesk Fusion** — not just point
clouds or meshes.

> **Status: work in progress.** The phone app captures and previews point clouds today. The reconstruction
> pipeline (TSDF fusion → meshing → primitive detection → B-Rep → STEP) exists as tested libraries but is
> **not yet wired into the app**. See [What works today](#what-works-today).

## What works today

### Android app (`Scanner.App`)
- **Live AR scan** using ARCore's Depth API: camera preview with a crosshair, Start / Pause / Resume / Finish.
- Pressing Start picks a **target** at the crosshair (ARCore hit test) and the **support plane** (the highest
  tracked horizontal plane below it). Only depth points within 30 cm of the target are accumulated.
- Depth frames are integrated at ~5 Hz into a 5 mm voxel point accumulator, filtered by depth range
  (0.10–1.50 m) and ARCore confidence.
- **Camera photos** are captured once per second (up to 60) with their pose and intrinsics.
- On Finish, the piece is **isolated** from the table: points on the support plane are dropped and the connected
  cluster nearest the target is kept (falls back to all points if isolation finds too few).
- **Preview page**: orbitable 3D point cloud (drag to rotate, pinch to zoom) and a photo browser.
- Every scan is saved as a session folder on the device (see [Session format](#session-format)).

The Windows target builds, but its views only show a placeholder message: scanning and the 3D preview are Android-only.

### Libraries (platform-independent, `net10.0`, unit tested)
| Project | Contents |
|---|---|
| `Scanner.Capture` | `DepthFrame` / `CameraIntrinsics`, ARCore pose and intrinsics conversions, depth back-projection, voxel point accumulator, object isolation, PLY read/write, session writer/reader, binary frame codec, `.scan` zip export, live-scan state machine |
| `Scanner.Core` | Sparse TSDF volume (8³ blocks), Surface Nets mesh extraction, sequential RANSAC for **planes and cylinders** with least-squares refinement, 3×3 linear algebra, synthetic SDF shapes + depth renderer for tests |
| `Scanner.Brep` | B-Rep model (vertices, edges, loops, faces, solid) with a validator, builders for **convex polyhedra** (from planes) and **tubes** (cylinder with optional coaxial hole), and a **STEP AP214 writer** (ISO 10303-21, millimetres) |

### Not implemented yet
- Running the reconstruction/STEP pipeline from the app, and exporting/sharing files from the phone.
- Cones, spheres, B-spline (organic) surfaces, general (non-convex) solids.
- iOS / Mac Catalyst (dropped for now — no Mac build agent), desktop reprocessing UI.

The longer-term design is in [docs/superpowers/specs/2026-09-19-scan3d-design.md](docs/superpowers/specs/2026-09-19-scan3d-design.md);
treat it as the roadmap, not a description of the current code.

## Repository layout

```
Scan3D.slnx                 Solution with the libraries and tests (the app is built separately)
Directory.Build.props       Nullable, latest C#, TreatWarningsAsErrors for every project
src/
  Scanner.Capture/          Capture model, point clouds, sessions        (no dependencies)
  Scanner.Core/             Fusion, meshing, segmentation                (→ Capture)
  Scanner.Brep/             B-Rep model, builders, STEP writer           (→ Core)
  Scanner.App/              .NET MAUI app, Android + Windows             (→ Capture)
    Platforms/Android/      ARCore session, depth/camera readers, GLES renderers
tests/
  Scanner.Capture.Tests/    xUnit
  Scanner.Core.Tests/       xUnit, incl. synthetic scans of known shapes
  Scanner.Brep.Tests/       xUnit, incl. a STEP syntax checker
docs/
  arcore-binding-notes.md   Notes on the ARCore .NET binding and its quirks
```

## Requirements

- **.NET SDK 10** (developed with 10.0.401).
- For the app: the .NET MAUI workloads (`dotnet workload install maui-android maui-windows`) and the Android SDK.
- For scanning: an **Android 7.0+ (API 24) phone that supports ARCore with the Depth API**, with
  Google Play Services for AR installed (the app prompts for it).

The ARCore binding is [`Vapolia.Google.ARCore`](https://www.nuget.org/packages/Vapolia.Google.ARCore) 1.47.1,
referenced only for the Android target.

## Build and test

Libraries and tests (any OS with the .NET 10 SDK):

```bash
dotnet test Scan3D.slnx
```

Android app — build, or build and deploy to a connected device (USB debugging enabled):

```bash
dotnet build src/Scanner.App -f net10.0-android
```

```bash
dotnet build src/Scanner.App -f net10.0-android -t:Run
```

Windows placeholder build:

```bash
dotnet build src/Scanner.App -f net10.0-windows10.0.19041.0
```

All projects build with warnings treated as errors.

## Using the app

1. Put the object on a table and open the app; grant camera permission and install ARCore services if asked.
2. Move the phone a little so ARCore detects the table, aim the crosshair at the object and press **Start**.
3. Walk slowly around the object; the status line shows tracking state, points, frames and photos.
4. Press **Finish**. The app isolates the object and opens the preview (3D cloud / photos).

Leaving the scan page before pressing Finish discards the unfinished session.

## Session format

Each scan is a folder under the app's data directory (`<AppData>/sessions/<id>/`):

| Path | Contents |
|---|---|
| `manifest.json` | Format version (2), id, creation time, device model, frame/point/photo counts, target point, support-plane height |
| `points.ply` | Isolated point cloud (metres, world frame, +Y up) |
| `frames/NNNNNN.frame` | Binary depth frame: `S3DF` magic, intrinsics, timestamp, camera→world matrix, depth as uint16 mm, optional confidence bytes (little-endian) |
| `photos/NNNNNN.jpg` + `.json` | Camera photo with its intrinsics, camera→world pose, timestamp and display rotation |

`ScanArchive.Export` zips a session folder into a portable `.scan` file. Version 1 sessions (without photos)
still load.

## License

[MIT](LICENSE) © 2026 Luca Tiburzio
