# Scan3D

A .NET 10 / .NET MAUI app that turns an Android phone or a LiDAR iPhone into a 3D scanner, with the goal of producing
**STEP (B-Rep) solids that open as editable bodies in CAD tools such as Autodesk Fusion**, not just point
clouds or meshes.

> **Status: work in progress.** The phone app scans an object on a table into a **textured, closed mesh** in a
> few seconds, with a live preview of the surface while scanning. Primitive detection, B-Rep and STEP export exist
> as tested libraries but are **not yet wired into the app**. See [What works today](#what-works-today).

## Scan modes

The app opens on a mode selection page; each way of scanning is its own mode, offered where the phone can do it.

| Mode | Phone | Geometry from | Poses from |
|---|---|---|---|
| **Handheld · AR** | Android (ARCore depth) | Photo stereo in a TSDF, space carving | ARCore, refined from the photos |
| **Handheld · LiDAR** | iPhone Pro (LiDAR) | ARKit's scene mesh, cut around the target above the table | ARKit |
| **Turntable · TrueDepth** | iPhone (front TrueDepth) | TrueDepth depth in a 1 mm TSDF | ICP of each depth frame against the piece fused so far |
| **Turntable · printed mat** | Android | Photo stereo in a TSDF, as handheld | The dots of a printed mat, in every photo (exact, true scale) |

The reconstruction pattern below is the handheld Android mode's, the one tested on a phone. The iPhone modes and the
mat turntable are tested on synthetic data only so far (see [Turntable modes](#turntable-modes)).

## Reconstruction pattern

The phones this is built for have no depth sensor: ARCore's depth is estimated from camera motion, and on real scans
it put the table up to 2 cm off and drifted over a loop around the object. So ARCore only **locates** the object;
the **photos** measure it.

| Role | Source |
|---|---|
| Where the object is (a box around it, the depth range to search, a first guess of the table) | ARCore depth points around the crosshair target |
| Camera poses | ARCore's pose of every photo as the **starting guess**, corrected from the photos themselves (bundle adjustment on matched corners, during the scan) |
| Geometry | Photo stereo (plane sweep) fused in a **TSDF voxel volume**, completed into a solid by **space carving** |
| Table | The lowest flat level of the photo surface with the object standing on it (ARCore only narrows the search to ±8 cm) |
| Appearance | **Photo patches**: every triangle is textured from the photo that sees it best |

```
ARCore ─► photo + pose (every 0.5 s) ─► corners, sharpness ─► pose refinement (every 8 photos, background)
  │                                                                  │ refined poses
  └─► depth points ─► object box, table guess                        ▼
                           │                   live: 1 depth map per photo ─► live TSDF ─► lit surface over the camera
                           ▼                                                  (fused again after every refinement)
                  Finish: 8 depth maps (refined poses) ─► TSDF ─► space carving ─► mesh cut from the table
                                                                ─► flat base ─► photo-patch texture ─► model.obj
```

### During the scan
- **Photos**: up to 90, one every 0.5 s, from the largest CPU camera image up to 1920×1080 that supports depth. Each
  photo keeps a 960 px grayscale copy in memory, its **corners** (for pose refinement) and its **sharpness** (mean
  squared gradient). Photos under half the scan's median sharpness count as motion-blurred: they are not used for
  depth, poses or texture, and the status line asks to move slower.
- **Pose refinement** runs every 8 new photos on a background worker. Up to 48 sharp photos spread over the scan are
  matched to their 3 closest partners by angle (800 corners each, epipolar search guided by the ARCore poses),
  triangulated and bundle-adjusted in two rounds, with ARCore's camera centres as soft priors (scale and gravity stay
  ARCore's). A result that turns the photos by more than 3° (median) is rejected: ARCore is off by about 1°, and a
  blurred close pass once produced a 10° "correction". Photos between refinements take the correction of the nearest
  refined photo.
- **Live surface**: each new photo becomes a depth map of the object box (its crop at most 256 px, depth hypotheses
  about one pixel of disparity apart), is cross-checked against the recent maps and fused into a 3 mm TSDF. The surface
  is shown where at least 3 depth maps saw it, cut from the table, and drawn lit over the camera image in place of the
  ARCore points. After every pose refinement the stored maps are fused again with the new poses, so layers ARCore's
  drift put side by side merge. The overlay is drawn through the newest photo's correction, back in ARCore's frame.

### On Finish (about 3.4–4.6 s on a Galaxy S23 Ultra)
1. Take the photos of the last pose refinement (the only ones with bundle-adjusted poses); refine again only if the
   last photos went unrefined.
2. Crop to the live surface's extent plus 2.5 cm (or ARCore's box, at most ±12 cm around the target), compute 8 depth
   maps (crops at most 384 px), keep the pixels two other maps confirm, and fuse them in a 3 mm TSDF.
3. **Table**: going up from the lowest level within 8 cm of ARCore's table, the first with a real share of the surface
   and part of the object above it; if the photos saw no floor, ARCore's table.
4. **Space carving**: a voxel above the table that no depth map sees through, and that two see behind a surface of
   the object, is solid. Walls stand where the photos start seeing the table beside them, unmatched patches fill, the
   underside closes. The measured TSDF surface wins where it exists, except near the table, where it is floor.
5. Extract the mesh (Surface Nets), keep the sizeable parts above the table, and flatten the base onto the table.
6. **Texture**: each triangle takes the photo that sees it most head-on and closest, unoccluded; triangles no photo
   shows clearly take the best photo facing them. Saved as `model.obj` + `model.mtl` pointing at the session's JPEGs.

If the photos give no usable surface, the scan falls back to the ARCore depth cloud cut from the table.

## What works today

### Phone app (`Scanner.App`)
- **Mode selection** start page (see [Scan modes](#scan-modes)).
- **Live AR scan** with a crosshair and Start / Pause / Resume / Finish. Start picks the **target** under the
  crosshair (ARCore hit test). Depth frames are accumulated at ~5 Hz (5 mm voxels, 0.10–1.50 m, ARCore confidence)
  once three frames agree on the surface height, and are saved.
- **Live photogrammetry** while scanning: refined poses, and the lit surface of the object over the camera image.
- **Finish** builds the textured, closed mesh described above.
- **Preview page**: the textured model (drag to rotate, pinch to zoom); the point cloud when there is no model.
- Every scan is saved as a session folder on the device (see [Session format](#session-format)).

- **iPhone** (`Platforms/iOS`, built only on a Mac or with a paired Mac): the LiDAR handheld scan (ARKit scene
  mesh plus LiDAR depth and photos into the same session format, photo-patch texture), the TrueDepth turntable, and a
  SceneKit preview of the model. Compiles; not yet run on a device.
- **Printed-mat turntable** on Android: the mat is posed in every photo, a photo is kept every 5° of turn, and Finish
  runs the voxel reconstruction and texturing on them. Not yet run on a device.

The Windows target builds, but its views only show a placeholder message: scanning and the 3D preview are phone-only.

### Turntable modes
- **TrueDepth** (`TurntableFusion`): the phone stands still, screen towards the piece about 25 cm away. The table is
  the plane with nothing behind it in the first frame; it and everything beyond 12 cm of the target are cut away, so
  only the turning piece is left. Point-to-plane ICP (`RigidAligner`) aligns each frame to the points fused so far,
  starting from the last frame's motion; frames that fit badly are skipped. The model comes out upright, standing on
  y = 0, and a photo every 15° of turn textures it.
- **Printed mat** (`TurntableMat`, `MatDetector`, `MatPose`, `MatTurntableScan`): print
  [docs/turntable-mat.svg](docs/turntable-mat.svg) at 100 % on A4 (the app can also share it). It is a ring of 36 dots,
  filled or with a hole, read as a code in which any 7 neighbours in a row (either way round) are unique. The detector
  finds dark disc-shaped blobs, the ellipse most lie on, their order around it and, from runs of 7 or more, which dots
  they are; a homography and Gauss-Newton on the reprojection give the camera pose in the mat's frame, in metres. The
  piece stands inside the dashed circle.

### Libraries (platform-independent, `net10.0`, unit tested)
| Project | Contents |
|---|---|
| `Scanner.Capture` | `DepthFrame` / `CameraIntrinsics` / `PhotoView`, ARCore pose and intrinsics conversions, depth back-projection, voxel point accumulator, support-plane fit, object isolation, PLY read/write, session writer/reader, binary frame codec, textured model OBJ/MTL read/write, `.scan` zip export, live-scan state machine |
| `Scanner.Core` | **Turntable**: printed-mat code, dot detection and pose (`TurntableMat`, `MatDetector`, `MatPose`, `MatTurntableScan`), ICP and TrueDepth fusion (`RigidAligner`, `TurntableFusion`). **Sensor mesh**: cutting the piece from a LiDAR mesh (`SensorMesh`). **Photogrammetry**: corners, guided matching, tracks, triangulation, bundle adjustment, pose refinement (`PoseRefiner`, `LivePoses`), plane-sweep stereo, depth-map cross-checking, photo sharpness, voxel reconstruction (`PhotoVoxelReconstruction`), live reconstruction (`LiveReconstruction`, `LivePhotogrammetry`). **Fusion and meshing**: sparse TSDF (8³ blocks), space carving, Surface Nets, mesh cleanup. **Texturing**: photo-patch texturing. **Segmentation**: sequential RANSAC for planes and cylinders with least-squares refinement. Synthetic SDF shapes and renderers for tests |
| `Scanner.Brep` | B-Rep model (vertices, edges, loops, faces, solid) with a validator, builders for **convex polyhedra** (from planes) and **tubes** (cylinder with optional coaxial hole), and a **STEP AP214 writer** (ISO 10303-21, millimetres) |
| `Scanner.Desktop` | `scan3d-process`: reprocesses a session or `.scan` file on a PC (the earlier point-cloud pipeline, not yet the voxel/texture one) |

### Known limits
- Shiny or transparent objects (a metal-and-glass jar) give the photos nothing to match.
- Moving fast blurs the photos, and coming very close makes the object fill the frame. Both are handled (blurred photos
  are skipped, close crops are scaled down) but cost detail.
- A thin fringe of floor can remain where few photos saw the table beside the object; the underside is untextured.
- The live surface appears about 10 s into the scan (it waits for three depth maps to agree).

### Not implemented yet
- Running primitive detection / B-Rep / STEP from the app, and exporting or sharing files from the phone (GLB is planned).
- Cones, spheres, B-spline (organic) surfaces, general (non-convex) solids.
- Full-resolution stills for the turntable modes (both use video-rate camera images, up to 1920 px), Mac Catalyst,
  desktop reprocessing UI.

The longer-term design is in [docs/superpowers/specs/2026-09-19-scan3d-design.md](docs/superpowers/specs/2026-09-19-scan3d-design.md)
and [docs/superpowers/specs/2026-09-26-photogrammetry-design.md](docs/superpowers/specs/2026-09-26-photogrammetry-design.md);
treat them as the roadmap, not a description of the current code.

## Repository layout

```
Scan3D.slnx                 Solution with the libraries and tests (the app is built separately)
Directory.Build.props       Nullable, latest C#, TreatWarningsAsErrors for every project
src/
  Scanner.Capture/          Capture model, point clouds, sessions        (no dependencies)
  Scanner.Core/             Fusion, meshing, segmentation                (→ Capture)
  Scanner.Brep/             B-Rep model, builders, STEP writer           (→ Core)
  Scanner.Desktop/          scan3d-process, PC reprocessing tool         (→ Core)
  Scanner.App/              .NET MAUI app, Android + iOS + Windows       (→ Capture, Core)
    Platforms/Android/      ARCore session, depth/camera readers, GLES renderers (camera, points, live surface,
                            textured model), the mat turntable camera
    Platforms/iOS/          ARKit LiDAR scan (mesh, depth, photos), TrueDepth capture, SceneKit preview
tests/
  Scanner.Capture.Tests/    xUnit
  Scanner.Core.Tests/       xUnit, incl. synthetic scans of known shapes
  Scanner.Brep.Tests/       xUnit, incl. a STEP syntax checker
  Scanner.Desktop.Tests/    xUnit
docs/
  arcore-binding-notes.md   Notes on the ARCore .NET binding and its quirks
  turntable-mat.svg         The printed turntable mat (A4, print at 100 %)
```

## Requirements

- **.NET SDK 10** (developed with 10.0.401).
- For the app: the .NET MAUI workloads (`dotnet workload install maui-android maui-windows`) and the Android SDK.
- For scanning: an **Android 7.0+ (API 24) phone that supports ARCore with the Depth API**, with
  Google Play Services for AR installed (the app prompts for it), or an **iPhone with LiDAR** (12 Pro or later Pro,
  iOS 15+) for the LiDAR and TrueDepth modes.
- For the iPhone build: a Mac with Xcode (or a Mac paired with Visual Studio on Windows) and the `ios` workload.

The ARCore binding is [`Vapolia.Google.ARCore`](https://www.nuget.org/packages/Vapolia.Google.ARCore) 1.47.1,
referenced only for the Android target.

## Build and test

Libraries and tests (any OS with the .NET 10 SDK):

```bash
dotnet test Scan3D.slnx
```

Android app: build a Release APK (the reconstruction is several times slower in Debug), then install it over the
existing app on a connected device (USB or Wi-Fi debugging):

```bash
dotnet build src/Scanner.App -f net10.0-android -c Release
```

```bash
adb install -r src/Scanner.App/bin/Release/net10.0-android/com.scan3d.scanner-Signed.apk
```

`install -r` updates in place and keeps the scans. `dotnet build -t:Run` in Release uninstalls the app first, which
**deletes every scan on the phone**. The app is debuggable in Release too, so sessions can be pulled for analysis:

```bash
adb exec-out run-as com.scan3d.scanner tar -cf - -C files sessions > sessions.tar
```

(`exec-out`, not `shell`: `adb shell` translates line endings and corrupts binary files.)

iPhone app: the `net10.0-ios` target is added automatically on a Mac; on Windows it needs a paired Mac and
`-p:EnableIos=true` (without a Mac, `-t:Compile` still type-checks the iOS code):

```bash
dotnet build src/Scanner.App -f net10.0-ios -p:EnableIos=true
```

Windows placeholder build:

```bash
dotnet build src/Scanner.App -f net10.0-windows10.0.19041.0
```

All projects build with warnings treated as errors.

## Using the app

Pick a mode on the start page. **Handheld** (below) works on every supported phone.

1. Put the object on a table and open the app; grant camera permission and install ARCore services if asked.
   Objects with print or texture work best; shiny or transparent ones do not.
2. Move the phone a little so ARCore tracks, aim the crosshair at the object and press **Start**.
3. Walk slowly around the object, keeping it in view, about 30–50 cm away. After about 10 s its surface appears over
   the camera image and grows as you go round. If the status line says the photos are blurred, move slower.
4. Press **Finish**. After a few seconds the textured model opens (drag to rotate, pinch to zoom).

Leaving the scan page before pressing Finish discards the unfinished session.

**Turntable · printed mat** (Android): print the mat, put the piece inside the dashed circle and the phone on a stand
with the whole ring in view (about 30 cm away, looking down at 25–40°). The status says when the mat is found; press
**Start**, turn the mat slowly or in small steps through a full turn (the status counts the degrees covered), then
**Finish**.

**Turntable · TrueDepth** (iPhone): stand the phone about 25 cm from the piece, screen towards it, crosshair on the
piece; press **Start**, turn the piece one full turn without your hand in view (a turntable is best), then **Finish**.

## Session format

Each scan is a folder under the app's data directory (`<AppData>/sessions/<id>/`):

| Path | Contents |
|---|---|
| `manifest.json` | Format version (2), id, creation time, device model, frame/point/photo counts, target point, support-plane height and fitted plane coefficients/margin |
| `model.obj` + `model.mtl` | The textured model (metres, world frame, +Y up): one material per photo used, pointing at `photos/NNNNNN.jpg`, so the folder opens as a textured mesh in Blender, MeshLab and other OBJ viewers |
| `points.ply` | The model's vertices, or the isolated ARCore depth cloud when the photos gave no model |
| `frames/NNNNNN.frame` | Binary depth frame: `S3DF` magic, intrinsics, timestamp, camera→world matrix, depth as uint16 mm, optional confidence bytes (little-endian) |
| `photos/NNNNNN.jpg` + `.json` | Camera photo with its intrinsics, camera→world pose (ARCore's, not refined), timestamp and display rotation |

The manifest `PhotoPointCount` is the number of model vertices (0 for a depth-only result); older sessions default to
zero and remain readable.

`ScanArchive.Export` zips a session folder into a portable `.scan` file. Version 1 sessions (without photos)
still load.

## License

[MIT](LICENSE) © 2026 Luca Tiburzio
