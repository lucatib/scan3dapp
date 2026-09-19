# Scan3D — Design Specification

Date: 2026-09-19
Status: draft for review

## 1. Goal

.NET MAUI app (net10) that turns a smartphone into a 3D scanner and produces
**STEP (B-Rep) files that can be imported and edited in Autodesk Fusion**, not just
point clouds or meshes. The same app runs on desktop (Windows, macOS via Mac
Catalyst) to reprocess scans at high resolution.

### In scope (v1)
- Capture on iOS (ARKit, LiDAR / scene depth) and Android (ARCore Depth API).
- Desktop import of `.scan` sessions and generic meshes (PLY/OBJ/STL).
- Processing **entirely local**, same code on all platforms,
  with different resolution profiles (mobile = draft, desktop = fine).
- Three user-selectable modes: **Mechanical**, **Organic**, **Environment**.
- Export: STEP AP214, STL, OBJ, PLY, `.scan`.
- 3D visualization with Silk.NET.

### Out of scope (v1)
- External gadgets (ToF VL53Lx via BLE, turntable with stepper motor). Remain
  possible through the `IDepthSource` / `IPoseSource` interfaces.
- Server/cloud processing.
- OpenCASCADE backend (planned for the future, desktop only, behind `IBrepBuilder`).
- Texture/color on the STEP model.
- Pure photogrammetry (without depth).

## 2. Architecture

```
Scanner.App (MAUI, net10.0-android/ios/maccatalyst/windows)
 ├─ UI MVVM (CommunityToolkit.Mvvm)
 ├─ Platforms/iOS      → ArKitDepthSource, ArKitPoseSource
 ├─ Platforms/Android  → ArCoreDepthSource, ArCorePoseSource
 └─ Viewer             → MAUI handler that hosts the Silk.NET renderer

Scanner.Capture   (net10.0)  IDepthSource, IPoseSource, ICameraSource interfaces,
                             DepthFrame model, .scan session format
Scanner.Core      (net10.0)  geometry, TSDF, marching cubes, mesh cleanup,
                             segmentation, RANSAC, primitive and B-spline fitting
Scanner.Brep      (net10.0)  B-Rep topology, IBrepBuilder, CSharpBrepBuilder,
                             StepWriter AP214, STL/OBJ/PLY export
Scanner.Render    (net10.0)  IRenderer + Silk.NET implementation (GLES 3.0)
Scanner.*.Tests   (xUnit)    tests for Core, Brep, Capture (serialization)
```

Rules:
- `Scanner.Core` and `Scanner.Brep` have **no** dependency on any platform
  or on MAUI, and are testable on a PC.
- Native implementations live only in `Scanner.App/Platforms/*`.
- Computation uses `System.Numerics` (SIMD) and `Parallel`; no GPU compute in v1.

## 3. Capture

- `IDepthSource` produces `DepthFrame`: depth map (float32, meters),
  optional confidence, intrinsics (fx, fy, cx, cy), timestamp.
- `IPoseSource` produces the camera→world pose (4x4 matrix) for a timestamp.
- iOS: `ARFrame.sceneDepth` (LiDAR, 256×192) or `smoothedSceneDepth`; pose from
  `ARCamera.transform`.
- Android: ARCore `acquireDepthImage16Bits` + confidence; pose from `Camera.getPose`.
- Integration frequency 5–10 Hz, frames discarded if tracking is not `Normal`.
- A device without depth support shows a message and only allows
  import mode.

### `.scan` session format
Zip archive:
- `manifest.json` — format version, device, mode, intrinsics, units.
- `frames/NNNNNN.depth` — compressed float16 depth + confidence.
- `frames/NNNNNN.pose` — 4x4 float32 matrix.
- `keyframes/NNNNNN.jpg` — optional photos (for future uses).

Used to reprocess at high resolution on desktop and as test data.

## 4. Processing pipeline

Common stages:
1. **TSDF fusion** on a sparse voxel grid (8³ blocks in a hash map).
   Voxel size: mobile 4–8 mm, desktop 1–2 mm (per-profile parameters).
2. **Marching Cubes** → triangle mesh.
3. **Cleanup**: removal of small components, light smoothing (Taubin),
   normal recomputation, optional decimation.
4. **Cropping**: user's 3D box; automatic removal of the support plane
   (RANSAC on the dominant plane under the object).

Stages per mode:

| Mode | Algorithm | STEP output |
|---|---|---|
| Mechanical | Region growing on normals + efficient RANSAC (plane, cylinder, cone, sphere), least-squares refinement; edges and vertices from intersection of adjacent primitives; optional snapping (parallelism, perpendicularity, coaxiality, standard radii) | Closed solid with analytic surfaces |
| Organic | Partition into quadrangular patches, B-spline fitting per patch, C0 continuity (approximated G1) at boundaries | Shell of `B_SPLINE_SURFACE_WITH_KNOTS` |
| Environment | Plane-only RANSAC; floor, ceiling, vertical walls; extruded 2D floor plan; rectangular openings | Wall/floor solid or set of surfaces |

**Fallback**: if the topology does not close, STL/OBJ are exported anyway along with a
"faceted" STEP (planar triangular faces). The app shows a quality
metric: % of area covered by primitives and RMS of the error in mm.

**Known limits**: fillets and chamfers below ~3x the voxel size become
sharp edges; continuity between organic patches is only approximate.

## 5. B-Rep and STEP export

- `IBrepBuilder` receives primitives and adjacencies, returns a B-Rep model
  (vertices, edges, loops, faces, shells, solid).
- `CSharpBrepBuilder` (v1): computes edges by analytic intersection
  (plane-plane, plane-cylinder, plane-cone, plane-sphere, coaxial
  cylinder-cylinder); unhandled intersections degrade to approximate
  polyline/B-spline edges.
- `StepWriter`: writes ISO 10303-21, AP214 schema (`AUTOMOTIVE_DESIGN`), units in mm.
  Entities: `MANIFOLD_SOLID_BREP`, `CLOSED_SHELL`, `ADVANCED_FACE`, `PLANE`,
  `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`,
  `B_SPLINE_SURFACE_WITH_KNOTS`, `EDGE_CURVE`, `LINE`, `CIRCLE`,
  `B_SPLINE_CURVE_WITH_KNOTS`, and the required product context entities.
- Internal validation before export: every edge shared by exactly
  two faces, consistent Euler characteristic, closed and oriented loops.
- Future: `OcctBrepBuilder` (OpenCASCADE) desktop only, same interface.

## 6. Rendering (Silk.NET)

- `IRenderer`: loads mesh (positions, normals, per-vertex colors, indices),
  orbit camera, selection (ray picking), edge lines, crop box gizmo,
  transparent overlay.
- Implementation: **OpenGL ES 3.0 via Silk.NET**.
  - Windows: ANGLE (D3D11) or WGL.
  - Android: native EGL.
  - iOS / Mac Catalyst: **ANGLE with Metal backend**.
- Hosted in a per-platform MAUI handler that provides the native surface.
- **Main risk**: availability and packaging of ANGLE on iOS/Catalyst.
  Mitigation: the plan's first task is a spike (triangle → mesh on the 4
  platforms). If it fails on Apple, switch to **Silk.NET.WebGPU**
  (wgpu-native) behind the same `IRenderer`.
- During capture: native camera preview (ARKit/ARCore) with the TSDF mesh
  overlay drawn by the Silk.NET renderer on a transparent layer.

## 7. UI and flow

1. **Projects**: list of scans (thumbnail, mode, date); "Import" on desktop.
2. **New scan**: mode + quality (Draft / Fine).
3. **Capture** (mobile): preview + live mesh colored by coverage;
   Start / Pause / Finish; warnings (too fast, too far, tracking lost).
4. **Crop**: 3D box, "remove support plane".
5. **Processing**: progress per stage, cancellable, in background.
6. **Result**: 3D view with primitives colored by type, error heatmap,
   statistics; parameters (RANSAC tolerance, snapping, minimum radius) with
   "rerun fitting".
7. **Export**: STEP / STL / OBJ / PLY / `.scan` via share sheet or "Save as".

## 8. Error handling

- Tracking lost or invalid depth: frame discarded, warning in the UI.
- Memory: voxel limit per profile; above threshold the voxel size is
  increased and the user is warned.
- Fitting/B-Rep failure: falls back (mesh + faceted STEP) with the
  reason shown to the user, never a crash.
- Processing cancellable via `CancellationToken` in all stages.

## 9. Testing

- **Synthetic** (`Scanner.Core.Tests`): generator of known objects (cube,
  drilled cylinder, flange, sphere, room) and depth-frame simulator
  from known poses with noise and holes. Verifies that fusion and fitting
  recover the primitives within tolerance.
- **STEP** (`Scanner.Brep.Tests`): syntactically valid file (internal parser),
  closed topology, golden file for simple cases.
- **Real regression**: real `.scan` scans in `testdata/` (Git LFS).
- **Manual**: import checklist in Fusion on a fixed set of examples;
  on-device testing for capture and UI.

## 10. Milestones

1. Silk.NET renderer spike on Windows, Android, iOS, Mac Catalyst.
2. Core on synthetic data: TSDF → mesh → RANSAC → B-Rep → STEP of a cube and
   drilled cylinder, verified in Fusion.
3. `.scan` format + desktop import + results viewer.
4. iOS capture (ARKit).
5. Android capture (ARCore).
6. Environment mode.
7. Organic mode (B-spline).
8. UI polish, export, quality profiles.
