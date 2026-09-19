# ARCore .NET binding spike — notes

Time-boxed spike to determine which ARCore .NET binding works on .NET 10 Android
and to prove, by compilation, that every ARCore API the scanner needs is
reachable from C#. Scope: `src/Scanner.App` (Android + Windows MAUI app
skeleton) plus this notes file. No device was used; nothing beyond
`dotnet build` was run.

## Chosen package

**`Vapolia.Google.ARCore` 1.47.1** (Android-only `PackageReference`, conditioned
on `$(TargetFramework.Contains('-android'))` in `Scanner.App.csproj`).

This was candidate (a) in the evaluation order and it worked on the first
attempt, so candidates (b) and (c) were never installed — per the spike's
20-minute-per-candidate time box, there was no reason to try them once (a)
built cleanly and exposed the full depth API. They are listed below only for
completeness (found via `dotnet package search ARCore`), not because they
were tried and failed.

Why it fits:
- It restores and builds warning-free against `net10.0-android` even though
  its `lib/` folder only ships `net8.0-android34.0` and `net9.0-android35.0`
  assets (see "Surprises" below) — NuGet's Android asset-compatibility
  fallback accepts the closest lower Android binding TFM.
- Its `Google.ARCore.dll` exposes the full ARCore Java API surface needed
  by the scanner: `Session`, `Config`, `Frame`, `Camera`, `Pose`,
  `CameraIntrinsics`, `Coordinates2d`, and — critically — every depth-image
  method (`AcquireDepthImage16Bits`, `AcquireRawDepthImage16Bts`,
  `AcquireRawDepthConfidenceImage`) plus `Session.IsDepthModeSupported`.
- Version 1.47.1 tracks a recent ARCore SDK (namespace `Google.AR.Core`,
  mirrors the current Java API, including newer members like
  `StreetscapeGeometryMode` and `SemanticMode` that don't exist in the older
  official binding).

### Candidates not installed (found, not tried)

| Package | Version | Why not tried |
|---|---|---|
| `Xamarin.Google.ARCore` | 1.29.0 | Official Microsoft/Xamarin package, but candidate (a) already succeeded; this is noticeably older (ARCore SDK ~1.29 vintage, predates the current Depth API additions such as raw-depth confidence and semantic modes) and was next in line only if (a) had failed. |
| `XPACE.Google.ARCore` | 1.41.0 | Third-party mirror, older than the chosen package; not needed once (a) worked. |
| `EngageSolutionsGroup.Google.AR.Core` | 1.50.48 | Third-party mirror; not needed once (a) worked. |
| `UrhoSharp.ARCore` | 1.9.67 | Bound for the UrhoSharp engine, not a general-purpose ARCore binding; wrong shape for this app. |
| `WaveEngine.ARMobile` | 2.5.0.39 | Wave Engine's own AR abstraction, not a direct ARCore binding; wrong shape for this app. |

## Project shape

- `src/Scanner.App/Scanner.App.csproj`: `TargetFrameworks` hard-set to
  `net10.0-android;net10.0-windows10.0.19041.0` (unconditional — iOS/MacCatalyst
  removed because no Mac build agent is available; `Platforms/iOS` and
  `Platforms/MacCatalyst` were deleted, not just excluded).
- `ApplicationId` = `com.scan3d.scanner`, `ApplicationTitle` = `Scan3D`.
- Android `SupportedOSPlatformVersion` raised from the template default (21)
  to **24.0** (ARCore's minimum).
- `AndroidManifest.xml` (`src/Scanner.App/Platforms/Android/AndroidManifest.xml`)
  adds:
  - `<uses-permission android:name="android.permission.CAMERA" />`
  - `<uses-feature android:name="android.hardware.camera.ar" android:required="true" />`
  - `<uses-feature android:glEsVersion="0x00030000" android:required="true" />`
  - `<meta-data android:name="com.google.ar.core" android:value="required" />`
    inside `<application>`.
- The ARCore `PackageReference` is inside an `ItemGroup` conditioned on
  `$(TargetFramework.Contains('-android'))` so it never touches the Windows
  build.

## NoWarn

**None added.** Both TFMs build with `TreatWarningsAsErrors=true` and **0
warnings** without any `<NoWarn>`. The probe file needed only two `using`
aliases to resolve name collisions (see "Surprises"); no compiler warning
needed suppressing.

## Build results (compile-only, no device)

```
dotnet build src/Scanner.App -f net10.0-android -c Debug
  -> Compilazione completata. Avvisi: 0. Errori: 0.

dotnet build src/Scanner.App -f net10.0-windows10.0.19041.0 -c Debug
  -> Compilazione completata. Avvisi: 0. Errori: 0.
```

Both were also verified as **clean rebuilds** (`bin`/`obj` deleted first),
not just incremental builds.

## Compile-verified API surface

Everything below lives in
`src/Scanner.App/Platforms/Android/ArCoreApiProbe.cs`, a static class that is
never called at runtime (not referenced from `MauiProgram`, `MainActivity`,
or any page/view). Its only job is to make the compiler prove these calls
resolve with these exact signatures against `Vapolia.Google.ARCore` 1.47.1.
All types below live in namespace `Google.AR.Core` unless noted.

### 1. Install/availability check

```csharp
ArCoreApk.Availability availability = ArCoreApk.Instance!.CheckAvailability(context)!;
bool isSupported = availability.IsSupported;

ArCoreApk.InstallStatus status = ArCoreApk.Instance!.RequestInstall(activity, true)!;
```
- `ArCoreApk.Instance` is a **static property**, not `ArCoreApk.GetInstance()`.
- `Availability` and `InstallStatus` are **nested types**:
  `Google.AR.Core.ArCoreApk.Availability` / `...ArCoreApk.InstallStatus`.
- `RequestInstall(Activity, bool)` — the second parameter is **not** named
  `userRequestedInstall` in this binding (a named-argument call with that
  name fails with CS1739); pass it positionally.

### 2. Session / Config lifecycle, depth mode, focus mode

```csharp
var session = new Session(context);          // Session(Android.Content.Context)
var config  = new Config(session);            // Config(Session)

bool depthSupported = session.IsDepthModeSupported(Config.DepthMode.Automatic!);
config.SetDepthMode(Config.DepthMode.Automatic!);   // or Config.DepthMode.Disabled!
config.SetFocusMode(Config.FocusMode.Auto!);

session.Configure(config);
session.Resume();
session.Pause();
session.Close();
```
- `Config.DepthMode` and `Config.FocusMode` are **nested classes**
  (`Google.AR.Core.Config.DepthMode`, `...Config.FocusMode`), not plain C#
  enums. They expose their values as **static properties**
  (`Config.DepthMode.Automatic`, `.Disabled`, `.RawDepthOnly`;
  `Config.FocusMode.Auto`, `.Fixed`), plus `ValueOf(string)`/`Values()` — this
  is the standard Xamarin/.NET-for-Android "Java enum as bound class"
  pattern, not a `[Flags]`-style C# `enum`.
- `Session.IsDepthModeSupported(DepthMode)` is a **method** on `Session`
  (it is not a `Config` member and not a property).
- `Config.SetDepthMode(...)` / `SetFocusMode(...)` return `Config` (fluent),
  but the probe ignores the return value since `Configure` reads state off
  the same `config` instance.

### 3. Per-frame update

```csharp
session.SetCameraTextureName(cameraTextureId);           // int
session.SetDisplayGeometry(displayRotation, width, height); // int, int, int
Google.AR.Core.Frame frame = session.Update()!;
```
- `Session.Update()` returns `Frame` — must call it `Google.AR.Core.Frame`
  or alias it (see "Surprises": collides with `Microsoft.Maui.Controls.Frame`).

### 4. Tracking state, camera pose, view/projection matrices

```csharp
Camera camera = frame.Camera!;
TrackingState trackingState = camera.TrackingState!;

if (trackingState.Equals(TrackingState.Tracking!))
{
    var poseMatrix = new float[16];
    camera.Pose!.ToMatrix(poseMatrix, 0);              // Pose.ToMatrix(float[], int)

    var viewMatrix = new float[16];
    camera.GetViewMatrix(viewMatrix, 0);                // Camera.GetViewMatrix(float[], int)

    var projectionMatrix = new float[16];
    camera.GetProjectionMatrix(projectionMatrix, 0, 0.1f, 100f); // (float[], int, near, far)
}
```
- `Camera.Pose`, `Camera.TrackingState`, `Camera.DisplayOrientedPose` are
  **properties** (Java getters `getPose()`/`getTrackingState()` become C#
  properties), but `GetViewMatrix`/`GetProjectionMatrix` stay **methods**
  because the Java originals take output-array parameters — the binding only
  turns a Java getter into a C# property when it has zero parameters.
- `TrackingState` (`Google.AR.Core.TrackingState`, **not** nested under
  `Camera`) has no `==`/`!=` operator overload in this binding — comparing
  with `==` compiles (reference equality inherited from `Java.Lang.Object`)
  but is not reliable; use `.Equals(...)` as shown.
- `Pose.ToMatrix(float[] dest, int offset)` is the 4x4-matrix accessor (row/
  column-major as defined by ARCore's `Pose.toMatrix`, column-major
  OpenGL-style); there is no separate "ToMatrix with no offset" overload.

### 5. Depth image acquisition + `NotYetAvailableException`

```csharp
try
{
    using Android.Media.Image depth = frame.AcquireDepthImage16Bits()!;
    // depth.Width, depth.Height, depth.GetPlanes()[0].Buffer, ...RowStride
}
catch (Google.AR.Core.Exceptions.NotYetAvailableException)
{
    // Depth image for this frame isn't ready yet; retry next Session.Update().
}

using Android.Media.Image rawDepth    = frame.AcquireRawDepthImage16Bits()!;
using Android.Media.Image confidence  = frame.AcquireRawDepthConfidenceImage()!;
```
- The binding exposes **five** depth/semantic acquisition methods on `Frame`:
  `AcquireDepthImage` (deprecated-shape 8-bit variant), `AcquireDepthImage16Bits`,
  `AcquireRawDepthImage`, `AcquireRawDepthImage16Bits`,
  `AcquireRawDepthConfidenceImage`, plus `AcquireSemanticImage` /
  `AcquireSemanticConfidenceImage`. The scanner should use the `16Bits`
  variants.
- All acquisition methods return `Android.Media.Image` (a Mono.Android/base
  Android type, not an ARCore-specific type) and throw
  `Google.AR.Core.Exceptions.NotYetAvailableException` when the frame's
  depth data isn't ready yet — this is a normal, expected, per-frame
  condition (not an error) and must be caught every frame.
- Reading the buffer: `image.GetPlanes()` returns `Android.Media.Image.Plane[]`;
  `plane.Buffer` is a `Java.Nio.ByteBuffer`, `plane.RowStride` / `PixelStride`
  are `int`. `image.Width` / `image.Height` are plain `int` properties.

**Depth image format** (per ARCore's public documentation for
`ImageFormat.DEPTH16`):
- `AcquireDepthImage16Bits()` / `AcquireRawDepthImage16Bits()` return a
  single-plane 16-bit-per-pixel image. Each 16-bit sample packs the
  **distance to the camera in millimeters** in the lower 13 bits, with the
  upper 3 bits reserved for a per-pixel confidence indicator baked into the
  same sample (0 = highest confidence bucket). Row stride can exceed
  `width * 2` bytes, so it must always be read via `RowStride`, never assumed.
- `AcquireRawDepthConfidenceImage()` returns a **separate** single-plane
  8-bit image, one byte per pixel, `0`–`255` (low to high confidence),
  aligned 1:1 with the raw depth image's pixel grid.

### 6. Camera intrinsics

```csharp
CameraIntrinsics textureIntrinsics = camera.TextureIntrinsics!;
CameraIntrinsics imageIntrinsics   = camera.ImageIntrinsics!;

float[] focalLength     = imageIntrinsics.GetFocalLength()!;      // [fx, fy]
float[] principalPoint  = imageIntrinsics.GetPrincipalPoint()!;    // [cx, cy]
int[]   imageDimensions = imageIntrinsics.GetImageDimensions()!;   // [width, height]
```
- `Camera.TextureIntrinsics` / `Camera.ImageIntrinsics` are **properties**
  returning `CameraIntrinsics`; `TextureIntrinsics` describes the GPU
  background texture, `ImageIntrinsics` describes `AcquireCameraImage()`'s
  CPU image — they can differ in resolution, so the scanner must intrinsics-
  match against whichever image (texture vs. CPU frame vs. depth image) it's
  actually projecting.
- `GetFocalLength`/`GetPrincipalPoint`/`GetImageDimensions` are **methods**
  (not properties) each with a no-offset array-returning overload and an
  `(array, offset)` in-place overload.

### 7. Camera-background texture coordinate transform

```csharp
var transformed = new float[normalizedQuadCoords.Length];
frame.TransformCoordinates2d(
    Coordinates2d.ViewNormalized!,
    normalizedQuadCoords,
    Coordinates2d.TextureNormalized!,
    transformed);
```
- `Coordinates2d` (`Google.AR.Core.Coordinates2d`, top-level, not nested) is
  the same "Java enum as bound class" pattern as `DepthMode`/`FocusMode`, with
  values `ImageNormalized`, `ImagePixels`, `OpenglNormalizedDeviceCoordinates`,
  `TextureNormalized`, `TextureTexels`, `View`, `ViewNormalized`.
- `Frame.TransformCoordinates2d` has two overloads: one taking `float[]`
  in/out arrays, one taking `Java.Nio.FloatBuffer` in/out — both are
  4-argument `(inputCoordinateSystem, input, outputCoordinateSystem, output)`.
  There's also a 3D variant (`TransformCoordinates3d`) and
  `TransformDisplayUvCoords(FloatBuffer, FloatBuffer)` for just the display
  UVs.

## Surprises

1. **`Frame` and `Image` name collisions.** Both
   `Microsoft.Maui.Controls.Frame`/`.Image` and
   `Google.AR.Core.Frame`/`Android.Media.Image` exist in scope inside a MAUI
   project (the MAUI SDK brings in `Microsoft.Maui.Controls` as an implicit
   global `using`). Using the bare names `Frame`/`Image` anywhere in
   `Platforms/Android` code produces `CS0104` ambiguous-reference errors.
   The probe resolves this with explicit aliases:
   `using ArFrame = Google.AR.Core.Frame;` and
   `using AndroidImage = Android.Media.Image;`. Any real capture code in
   `Scanner.App` will need the same treatment.
2. **Java "enums" are bound as classes with static properties, not C#
   `enum`.** `Config.DepthMode`, `Config.FocusMode`, `TrackingState`,
   `Coordinates2d`, `ArCoreApk.Availability`, `ArCoreApk.InstallStatus`, etc.
   all compile to sealed classes with static instance-returning properties
   (`.Automatic`, `.Tracking`, ...) plus Java-mirroring `ValueOf`/`Values`
   statics — not a `[Flags]`/plain `enum`. Switch-expression-on-enum idioms
   don't apply; compare with `.Equals(...)`.
2b. **No `==` operator on these bound "enum" classes.** `TrackingState ==
   TrackingState.Tracking` *compiles* (falls back to reference equality) but
   is not guaranteed to behave like value equality across all JNI object
   instances; `.Equals(...)` is the safe comparison.
3. **Getters become properties only when parameterless.** Java
   `getTrackingState()`/`getPose()`/`getImageIntrinsics()` all became plain
   C# properties, but `getViewMatrix(float[], int)` /
   `getProjectionMatrix(float[], int, float, float)` stayed methods because
   they take parameters — there's no C# property equivalent of "getter with
   an output-array parameter."
4. **`RequestInstall`'s second parameter has no usable name.** Calling
   `ArCoreApk.Instance.RequestInstall(activity, userRequestedInstall: true)`
   fails to compile (`CS1739`, no parameter named `userRequestedInstall` on
   the best-matching overload) even though that's the name Google's own
   Java/Kotlin docs use for it. Pass it positionally.
5. **The package ships no literal `net10.0-android` asset.** Vapolia.Google.
   ARCore 1.47.1's `lib/` folder only contains `net8.0-android34.0` and
   `net9.0-android35.0`. NuGet's Android TFM compatibility fallback accepts
   the `net9.0-android35.0` asset for a `net10.0-android` project without any
   extra configuration, and the build is warning-free — but this is worth
   watching on future SDK bumps in case that fallback ever stops applying.
6. **Depth image acquisition throws, it doesn't return null/failure codes.**
   `NotYetAvailableException` (`Google.AR.Core.Exceptions`) is the idiomatic
   .NET mapping of ARCore's Java `NotYetAvailableException` and must be
   caught around every `AcquireDepthImage16Bits`/`AcquireRawDepthImage16Bits`/
   `AcquireRawDepthConfidenceImage` call — there's no `TryAcquire...` variant.
7. **Nullable annotations on the binding are inconsistent with "everything
   nullable."** Rather than reason about which specific ARCore members carry
   `[MaybeNull]`/nullable-reference annotations in the shipped binding, the
   probe defensively null-forgives (`!`) every Java-object-returning call.
   This keeps the project's repo-wide `TreatWarningsAsErrors` clean without
   guessing binding-internal nullability metadata; real capture code should
   still null-check where a `null` is actually possible at runtime (e.g. a
   `Session` not yet resumed).

## Environment used

- .NET SDK 10.0.401, workloads `android`, `ios`, `maccatalyst`, `maui-windows`
  already installed.
- Android SDK at `C:\Program Files (x86)\Android\android-sdk`.
- No emulator/device attached; no `dotnet build ... -t:Run` or deploy was
  attempted, per the task's constraints.

## Adaptations in Task 6

Members the Task 6 brief used that the spike had not pinned, as verified by
compiling `src/Scanner.App` (and by reading the binding's metadata) against
`Vapolia.Google.ARCore` 1.47.1:

| Brief spelling | Binding spelling | Notes |
|---|---|---|
| `plane.GetType_()` | `plane.GetType()` | Java `Plane.getType()` is bound as a `GetType()` that hides `object.GetType()` and returns `Plane.Type`. Assign it to a typed local (`ArPlane.Type t = plane.GetType()!;`): `plane.GetType().Equals(...)` would also compile if it ever resolved to `System.Type`, and would then always be false. |
| `Plane` (bare) | `using ArPlane = Google.AR.Core.Plane;` | Ambiguous (`CS0104`) with `System.Numerics.Plane` in any file that also imports `System.Numerics`. |
| `status == ArCoreApk.InstallStatus.InstallRequested` | `status.Equals(ArCoreApk.InstallStatus.InstallRequested!)` | Same bound-Java-enum rule as `TrackingState`: no `==` overload. |

Verified as the brief spelled them (no change needed): `Frame.HitTest(float, float)`
→ `IList<HitResult>`; `HitResult.HitPose` (property); `Pose.Tx()/Ty()/Tz()`
(methods); `Session.GetAllTrackables(Java.Lang.Class)` → **non-generic**
`System.Collections.ICollection` (pattern-match the elements);
`Plane.TrackingState`, `Plane.CenterPose`, `Plane.SubsumedBy` (properties);
`Plane.Type.HorizontalUpwardFacing`; `Config.SetPlaneFindingMode(Config.PlaneFindingMode.Horizontal!)`;
`Frame.HasDisplayGeometryChanged` (property); `Frame.Timestamp` and
`Android.Media.Image.Timestamp` (`long` properties, nanoseconds);
`Coordinates2d.OpenglNormalizedDeviceCoordinates`; `Camera.TextureIntrinsics`;
`ArCoreApk.Availability.IsTransient` / `IsUnsupported` (properties).

Behavioural notes found while wiring the scan screen:

- **`Android.Media.Image` must be closed with `Close()`.** `using`/`Dispose()`
  on a .NET for Android peer only releases the JNI reference; it does not
  call Java `close()`. Unclosed ARCore images exhaust ARCore's image pool
  (`ResourceExhaustedException`) after a few frames. `DepthFrameReader`
  calls `Close()` then `Dispose()` in a `finally`, on every path.
- **Depth intrinsics come from `Camera.TextureIntrinsics`.** ARCore's depth
  images cover the GPU texture's field of view (depth pixels are addressed in
  `TEXTURE_NORMALIZED` coordinates; Google's raw-depth sample scales
  `getTextureIntrinsics()` to the depth size). `ImageIntrinsics` describes
  the CPU image, which can have a different field of view (e.g. 4:3 CPU image
  vs 16:9 texture), so scaling it to the depth size would distort the points.
