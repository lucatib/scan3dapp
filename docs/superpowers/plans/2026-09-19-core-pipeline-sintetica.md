# Synthetic core pipeline → STEP — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** From synthetic depth scans of a cube and a tube (drilled cylinder), produce STEP AP214 files with analytic surfaces, valid and openable in Autodesk Fusion.

**Architecture:** Pure .NET libraries (`Scanner.Capture`, `Scanner.Core`, `Scanner.Brep`) with no platform dependencies. Pipeline: depth frames → sparse TSDF fusion → Naive Surface Nets (mesh + normals from the TSDF gradient) → plane/cylinder RANSAC with least-squares refinement → B-Rep construction (convex polyhedron from planes, or coaxial tube) → topological validation → STEP writer in C#. A small CLI generates the sample files for manual verification in Fusion.

**Tech Stack:** .NET 10 (`net10.0`), C# latest, `System.Numerics`, xUnit.

**Spec:** `docs/superpowers/specs/2026-09-19-scan3d-design.md` (sections 2, 4, 5, 9; milestone 2)

## Global Constraints

- All projects: `net10.0`, `Nullable` enable, `ImplicitUsings` enable, `TreatWarningsAsErrors` true.
- `Scanner.Core` and `Scanner.Brep` have **no** dependency on any platform or on MAUI.
- Internal units: **meters**. STEP is written in **millimeters** (`SI_UNIT(.MILLI.,.METRE.)`).
- Camera convention: OpenCV (x right, y down, z forward); System.Numerics `Matrix4x4` with row vectors (`Vector3.Transform(p, cameraToWorld)`).
- TSDF normalized to [-1, 1], **positive outside** the object; normals point out of the solid.
- STEP schema: `AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }` (AP214).
- STEP numbers formatted with `CultureInfo.InvariantCulture`, always with a decimal point and no exponent.
- Work directly on `main`; every commit ends with the line `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- No explicit `Xunit` using in tests: the xUnit template adds a global `<Using Include="Xunit" />`.

**Conscious deviations from the spec (to report to the user):**
- Meshing with **Naive Surface Nets** instead of Marching Cubes: no 256-case table, more regular mesh; same interface, replaceable later.
- In this plan, the "Mechanical" B-Rep builder covers only **convex polyhedra** (planes only) and **coaxial tubes/cylinders** with two orthogonal caps. The general builder for primitive intersection, the "faceted" fallback STEP, and golden files arrive in later plans.

## File Structure

```
Scan3D.slnx
Directory.Build.props
.gitignore
src/Scanner.Capture/
  CameraIntrinsics.cs          pinhole intrinsics
  DepthFrame.cs                depth frame + pose
src/Scanner.Core/
  ProcessingProfile.cs         Draft/Fine profiles
  LinearAlgebra/Basis.cs       orthonormal basis of a plane
  LinearAlgebra/Linear3.cs     3x3 linear system
  LinearAlgebra/SymmetricEigen3.cs  symmetric 3x3 eigenvalues (Jacobi)
  Synthetic/ISdf.cs            SphereSdf, BoxSdf, TubeSdf
  Synthetic/CameraPoses.cs     LookAt, poses on a Fibonacci sphere
  Synthetic/SyntheticDepthRenderer.cs  sphere tracing → DepthFrame
  Synthetic/SyntheticScan.cs   simulated multi-view scan
  Fusion/TsdfVolume.cs         sparse block-based TSDF
  Meshing/TriangleMesh.cs      mesh with per-vertex normals
  Meshing/SurfaceNets.cs       isosurface extraction
  Shapes/Primitives.cs         PlanePrimitive, CylinderPrimitive
  Segmentation/PointCloud.cs   PointCloud, RansacOptions, DetectedShape
  Segmentation/PrimitiveFitter.cs  least-squares fitting
  Segmentation/RansacDetector.cs   sequential plane/cylinder RANSAC
src/Scanner.Brep/
  Model/BrepModel.cs           vertices, edges, loops, faces, solid
  Model/BrepValidator.cs       topological validation
  Builders/ConvexPolyhedronBuilder.cs
  Builders/TubeBuilder.cs
  Step/StepWriter.cs           ISO 10303-21 AP214
  Reconstruction/MechanicalReconstructor.cs  shapes → B-Rep
  Reconstruction/ScanToStep.cs frame → STEP
tests/Scanner.Core.Tests/…     one test file per component
tests/Scanner.Brep.Tests/…     one test file per component + StepSyntaxChecker
tools/Scanner.Cli/Program.cs   generates cube/tube .stp
docs/fusion-checklist.md       manual verification in Fusion
```

---

### Task 1: Solution scaffold + linear algebra

**Files:**
- Create: `Directory.Build.props`, `Scan3D.slnx` (via CLI), `.gitignore` (via CLI)
- Create: `src/Scanner.Capture/Scanner.Capture.csproj`, `src/Scanner.Core/Scanner.Core.csproj`, `src/Scanner.Brep/Scanner.Brep.csproj`
- Create: `tests/Scanner.Core.Tests/Scanner.Core.Tests.csproj`, `tests/Scanner.Brep.Tests/Scanner.Brep.Tests.csproj`
- Create: `src/Scanner.Core/LinearAlgebra/Basis.cs`, `Linear3.cs`, `SymmetricEigen3.cs`
- Test: `tests/Scanner.Core.Tests/LinearAlgebra/LinearAlgebraTests.cs`

**Interfaces:**
- Produces:
  - `static (Vector3 U, Vector3 V) Basis.Orthonormal(Vector3 n)` — `n` unit vector, `U × V = n`.
  - `static bool Linear3.TrySolve(double[,] a, double[] b, out double[] x)`; `static double Linear3.Det(double[,] m)`.
  - `static (double[] Values, Vector3[] Vectors) SymmetricEigen3.Solve(double[,] matrix)` — increasing eigenvalues, unit eigenvectors.

- [ ] **Step 1: Create the solution and projects**

From `C:\Workspace\scan3dapp`:

```bash
dotnet new gitignore
dotnet new sln -n Scan3D
dotnet new classlib -n Scanner.Capture -o src/Scanner.Capture -f net10.0
dotnet new classlib -n Scanner.Core -o src/Scanner.Core -f net10.0
dotnet new classlib -n Scanner.Brep -o src/Scanner.Brep -f net10.0
dotnet new xunit -n Scanner.Core.Tests -o tests/Scanner.Core.Tests -f net10.0
dotnet new xunit -n Scanner.Brep.Tests -o tests/Scanner.Brep.Tests -f net10.0
rm src/Scanner.Capture/Class1.cs src/Scanner.Core/Class1.cs src/Scanner.Brep/Class1.cs
rm tests/Scanner.Core.Tests/UnitTest1.cs tests/Scanner.Brep.Tests/UnitTest1.cs
dotnet sln add src/Scanner.Capture src/Scanner.Core src/Scanner.Brep tests/Scanner.Core.Tests tests/Scanner.Brep.Tests
dotnet add src/Scanner.Core reference src/Scanner.Capture
dotnet add src/Scanner.Brep reference src/Scanner.Core
dotnet add tests/Scanner.Core.Tests reference src/Scanner.Core
dotnet add tests/Scanner.Brep.Tests reference src/Scanner.Brep
```

Verify that both test `.csproj` files contain `<Using Include="Xunit" />`; if missing, add it in an `<ItemGroup>`.

- [ ] **Step 2: Create `Directory.Build.props`**

```xml
<Project>
  <PropertyGroup>
    <Nullable>enable</Nullable>
    <ImplicitUsings>enable</ImplicitUsings>
    <LangVersion>latest</LangVersion>
    <TreatWarningsAsErrors>true</TreatWarningsAsErrors>
  </PropertyGroup>
</Project>
```

(The `TargetFramework` stays in the individual `.csproj` files: putting it here would break multi-targeting for the future MAUI app.)

- [ ] **Step 3: Write the failing tests**

`tests/Scanner.Core.Tests/LinearAlgebra/LinearAlgebraTests.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Tests.LinearAlgebra;

public class LinearAlgebraTests
{
    [Fact]
    public void Eigen_diagonal_matrix_returns_sorted_values_and_axes()
    {
        var (values, vectors) = SymmetricEigen3.Solve(new double[,] { { 5, 0, 0 }, { 0, 1, 0 }, { 0, 0, 3 } });

        Assert.Equal(1.0, values[0], 9);
        Assert.Equal(3.0, values[1], 9);
        Assert.Equal(5.0, values[2], 9);
        Assert.Equal(1f, MathF.Abs(vectors[0].Y), 5);
        Assert.Equal(1f, MathF.Abs(vectors[1].Z), 5);
        Assert.Equal(1f, MathF.Abs(vectors[2].X), 5);
    }

    [Fact]
    public void Eigen_rotated_matrix_recovers_basis()
    {
        var e1 = Vector3.Normalize(new Vector3(1, 1, 0));
        var e2 = Vector3.Normalize(new Vector3(-1, 1, 0));
        var e3 = Vector3.UnitZ;
        var m = new double[3, 3];
        AddScaledOuter(m, e1, 2);
        AddScaledOuter(m, e2, 7);
        AddScaledOuter(m, e3, 4);

        var (values, vectors) = SymmetricEigen3.Solve(m);

        Assert.Equal(2.0, values[0], 6);
        Assert.Equal(4.0, values[1], 6);
        Assert.Equal(7.0, values[2], 6);
        Assert.Equal(1f, MathF.Abs(Vector3.Dot(vectors[0], e1)), 5);
        Assert.Equal(1f, MathF.Abs(Vector3.Dot(vectors[1], e3)), 5);
        Assert.Equal(1f, MathF.Abs(Vector3.Dot(vectors[2], e2)), 5);
    }

    [Fact]
    public void Linear3_solves_regular_system()
    {
        var a = new double[,] { { 2, 1, 0 }, { 1, 3, 1 }, { 0, 1, 4 } };

        Assert.True(Linear3.TrySolve(a, new double[] { 4, 10, 14 }, out var x));
        Assert.Equal(1.0, x[0], 9);
        Assert.Equal(2.0, x[1], 9);
        Assert.Equal(3.0, x[2], 9);
    }

    [Fact]
    public void Linear3_rejects_singular_system()
    {
        var a = new double[,] { { 1, 2, 3 }, { 2, 4, 6 }, { 0, 1, 1 } };

        Assert.False(Linear3.TrySolve(a, new double[] { 1, 2, 3 }, out _));
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(0, 0, 1)]
    [InlineData(0.3, -0.5, 0.8)]
    public void Basis_is_orthonormal_and_right_handed(float x, float y, float z)
    {
        var n = Vector3.Normalize(new Vector3(x, y, z));

        var (u, v) = Basis.Orthonormal(n);

        Assert.Equal(0f, Vector3.Dot(u, n), 5);
        Assert.Equal(0f, Vector3.Dot(v, n), 5);
        Assert.Equal(1f, u.Length(), 5);
        Assert.Equal(1f, v.Length(), 5);
        Assert.True(Vector3.Distance(Vector3.Cross(u, v), n) < 1e-5f);
    }

    private static void AddScaledOuter(double[,] m, Vector3 e, double s)
    {
        double[] a = { e.X, e.Y, e.Z };
        for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m[r, c] += s * a[r] * a[c];
    }
}
```

- [ ] **Step 4: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: compilation FAILURE — `The type or namespace name 'LinearAlgebra' does not exist`.

- [ ] **Step 5: Implement**

`src/Scanner.Core/LinearAlgebra/Basis.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.LinearAlgebra;

public static class Basis
{
    /// <summary>Orthonormal basis (U, V) of the plane orthogonal to <paramref name="n"/> (unit vector), with U × V = n.</summary>
    public static (Vector3 U, Vector3 V) Orthonormal(Vector3 n)
    {
        var helper = MathF.Abs(n.X) < 0.9f ? Vector3.UnitX : Vector3.UnitY;
        var u = Vector3.Normalize(Vector3.Cross(helper, n));
        var v = Vector3.Cross(n, u);
        return (u, v);
    }
}
```

`src/Scanner.Core/LinearAlgebra/Linear3.cs`:

```csharp
namespace Scanner.Core.LinearAlgebra;

public static class Linear3
{
    /// <summary>Solves A·x = b using Cramer's rule. Returns false if A is nearly singular.</summary>
    public static bool TrySolve(double[,] a, double[] b, out double[] x)
    {
        x = new double[3];
        double det = Det(a);
        double scale = 0;
        foreach (double value in a) scale = Math.Max(scale, Math.Abs(value));
        if (scale == 0 || Math.Abs(det) < 1e-12 * scale * scale * scale) return false;

        for (int col = 0; col < 3; col++)
        {
            var m = (double[,])a.Clone();
            for (int row = 0; row < 3; row++) m[row, col] = b[row];
            x[col] = Det(m) / det;
        }
        return true;
    }

    public static double Det(double[,] m) =>
        m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
        - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
        + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
}
```

`src/Scanner.Core/LinearAlgebra/SymmetricEigen3.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.LinearAlgebra;

/// <summary>Eigenvalues and eigenvectors of a symmetric 3x3 matrix (Jacobi method).</summary>
public static class SymmetricEigen3
{
    /// <summary>Eigenvalues in increasing order with their corresponding unit eigenvectors.</summary>
    public static (double[] Values, Vector3[] Vectors) Solve(double[,] matrix)
    {
        var a = (double[,])matrix.Clone();
        var v = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (int sweep = 0; sweep < 50; sweep++)
        {
            double off = a[0, 1] * a[0, 1] + a[0, 2] * a[0, 2] + a[1, 2] * a[1, 2];
            if (off < 1e-30) break;

            for (int p = 0; p < 2; p++)
            for (int q = p + 1; q < 3; q++)
            {
                if (Math.Abs(a[p, q]) < 1e-300) continue;
                double theta = (a[q, q] - a[p, p]) / (2 * a[p, q]);
                double t = theta == 0 ? 1 : Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                double c = 1 / Math.Sqrt(t * t + 1);
                Rotate(a, v, p, q, c, t * c);
            }
        }

        var order = new[] { 0, 1, 2 }.OrderBy(i => a[i, i]).ToArray();
        var values = order.Select(i => a[i, i]).ToArray();
        var vectors = order
            .Select(i => Vector3.Normalize(new Vector3((float)v[0, i], (float)v[1, i], (float)v[2, i])))
            .ToArray();
        return (values, vectors);
    }

    // A ← Pᵀ·A·P, V ← V·P with P a Jacobi rotation in the (p, q) plane.
    private static void Rotate(double[,] a, double[,] v, int p, int q, double c, double s)
    {
        for (int k = 0; k < 3; k++)
        {
            double akp = a[k, p], akq = a[k, q];
            a[k, p] = c * akp - s * akq;
            a[k, q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; k++)
        {
            double apk = a[p, k], aqk = a[q, k];
            a[p, k] = c * apk - s * aqk;
            a[q, k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; k++)
        {
            double vkp = v[k, p], vkq = v[k, q];
            v[k, p] = c * vkp - s * vkq;
            v[k, q] = s * vkp + c * vkq;
        }
    }
}
```

- [ ] **Step 6: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: PASS (8 tests). Then `dotnet build Scan3D.slnx` with no warnings.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat(core): scaffold solution and 3x3 linear algebra" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Capture types + synthetic scan generator

**Files:**
- Create: `src/Scanner.Capture/CameraIntrinsics.cs`, `src/Scanner.Capture/DepthFrame.cs`
- Create: `src/Scanner.Core/Synthetic/ISdf.cs`, `CameraPoses.cs`, `SyntheticDepthRenderer.cs`, `SyntheticScan.cs`
- Test: `tests/Scanner.Core.Tests/Synthetic/SyntheticTests.cs`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy)`
  - `sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)` with `float DepthAt(int u, int v)`; depth z in meters, 0 = invalid.
  - `interface ISdf { float Distance(Vector3 p); }`; `SphereSdf(Vector3 Center, float Radius)`, `BoxSdf(Vector3 Center, Vector3 HalfSize)`, `TubeSdf(Vector3 Center, float OuterRadius, float InnerRadius, float Height)` (Z axis).
  - `static Matrix4x4 CameraPoses.LookAt(Vector3 eye, Vector3 target)`; `static IReadOnlyList<Matrix4x4> CameraPoses.FibonacciSphere(int count, float radius, Vector3 target)`.
  - `static DepthFrame SyntheticDepthRenderer.Render(ISdf sdf, CameraIntrinsics k, Matrix4x4 cameraToWorld, float noiseSigma = 0f, int seed = 0, double timestampSeconds = 0)`.
  - `static CameraIntrinsics SyntheticScan.DefaultIntrinsics` (320×240, f=300); `static IReadOnlyList<DepthFrame> SyntheticScan.Capture(ISdf shape, int views, float cameraDistance, CameraIntrinsics intrinsics, float noiseSigma, int seed)`.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Core.Tests/Synthetic/SyntheticTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Synthetic;

public class SyntheticTests
{
    [Fact]
    public void Box_sdf_is_negative_inside_and_metric_outside()
    {
        var box = new BoxSdf(Vector3.Zero, new Vector3(0.02f));

        Assert.Equal(-0.02f, box.Distance(Vector3.Zero), 6);
        Assert.Equal(0.01f, box.Distance(new Vector3(0.03f, 0, 0)), 6);
    }

    [Fact]
    public void Tube_sdf_hole_is_outside_and_wall_is_inside()
    {
        var tube = new TubeSdf(Vector3.Zero, 0.02f, 0.01f, 0.03f);

        Assert.Equal(0.01f, tube.Distance(Vector3.Zero), 6);
        Assert.Equal(-0.005f, tube.Distance(new Vector3(0.015f, 0, 0)), 6);
    }

    [Fact]
    public void LookAt_points_camera_z_at_target_with_right_handed_axes()
    {
        var eye = new Vector3(0.1f, 0.05f, -0.02f);
        var m = CameraPoses.LookAt(eye, Vector3.Zero);

        var forward = Vector3.TransformNormal(Vector3.UnitZ, m);
        var right = Vector3.TransformNormal(Vector3.UnitX, m);
        var down = Vector3.TransformNormal(Vector3.UnitY, m);

        Assert.True(Vector3.Distance(forward, Vector3.Normalize(-eye)) < 1e-5f);
        Assert.True(Vector3.Distance(Vector3.Cross(right, down), forward) < 1e-5f);
        Assert.True(Vector3.Distance(m.Translation, eye) < 1e-6f);
    }

    [Fact]
    public void Renderer_measures_box_face_depth_on_optical_axis()
    {
        var k = new CameraIntrinsics(64, 48, 60f, 60f, 32f, 24f);
        var pose = CameraPoses.LookAt(new Vector3(0, 0, -0.1f), Vector3.Zero);

        var frame = SyntheticDepthRenderer.Render(new BoxSdf(Vector3.Zero, new Vector3(0.02f)), k, pose);

        Assert.Equal(0.08f, frame.DepthAt(32, 24), 4);
        Assert.Equal(0f, frame.DepthAt(0, 0));
    }

    [Fact]
    public void Scan_produces_one_frame_per_view()
    {
        var frames = SyntheticScan.Capture(new SphereSdf(Vector3.Zero, 0.02f), 5, 0.12f,
            new CameraIntrinsics(32, 24, 30f, 30f, 16f, 12f), 0f, 1);

        Assert.Equal(5, frames.Count);
        Assert.All(frames, f => Assert.Contains(f.Depth, d => d > 0));
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SyntheticTests"`
Expected: compilation FAILURE — `Scanner.Capture` / `Scanner.Core.Synthetic` do not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Capture/CameraIntrinsics.cs`:

```csharp
namespace Scanner.Capture;

/// <summary>Pinhole intrinsics in pixels, OpenCV convention (x right, y down, z forward).</summary>
public readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy);
```

`src/Scanner.Capture/DepthFrame.cs`:

```csharp
using System.Numerics;

namespace Scanner.Capture;

/// <summary>Depth map (z in meters, 0 = invalid) with the camera→world pose at capture time.</summary>
public sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)
{
    public float DepthAt(int u, int v) => Depth[v * Intrinsics.Width + u];
}
```

`src/Scanner.Core/Synthetic/ISdf.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Synthetic;

/// <summary>Signed distance function: negative inside the object.</summary>
public interface ISdf
{
    float Distance(Vector3 p);
}

public sealed record SphereSdf(Vector3 Center, float Radius) : ISdf
{
    public float Distance(Vector3 p) => Vector3.Distance(p, Center) - Radius;
}

public sealed record BoxSdf(Vector3 Center, Vector3 HalfSize) : ISdf
{
    public float Distance(Vector3 p)
    {
        var q = Vector3.Abs(p - Center) - HalfSize;
        float outside = Vector3.Max(q, Vector3.Zero).Length();
        float inside = MathF.Min(MathF.Max(q.X, MathF.Max(q.Y, q.Z)), 0f);
        return outside + inside;
    }
}

/// <summary>Drilled cylinder with Z axis centered at <see cref="Center"/>.</summary>
public sealed record TubeSdf(Vector3 Center, float OuterRadius, float InnerRadius, float Height) : ISdf
{
    public float Distance(Vector3 p)
    {
        var d = p - Center;
        float r = MathF.Sqrt(d.X * d.X + d.Y * d.Y);
        float dr = MathF.Abs(r - (OuterRadius + InnerRadius) / 2) - (OuterRadius - InnerRadius) / 2;
        float dz = MathF.Abs(d.Z) - Height / 2;
        float outside = new Vector2(MathF.Max(dr, 0f), MathF.Max(dz, 0f)).Length();
        float inside = MathF.Min(MathF.Max(dr, dz), 0f);
        return outside + inside;
    }
}
```

`src/Scanner.Core/Synthetic/CameraPoses.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Synthetic;

public static class CameraPoses
{
    /// <summary>Camera→world pose (OpenCV convention) looking at <paramref name="target"/> from <paramref name="eye"/>.</summary>
    public static Matrix4x4 LookAt(Vector3 eye, Vector3 target)
    {
        var forward = Vector3.Normalize(target - eye);
        var up = MathF.Abs(forward.Z) > 0.99f ? Vector3.UnitY : Vector3.UnitZ;
        var right = Vector3.Normalize(Vector3.Cross(forward, up));
        var down = Vector3.Cross(forward, right);
        return new Matrix4x4(
            right.X, right.Y, right.Z, 0,
            down.X, down.Y, down.Z, 0,
            forward.X, forward.Y, forward.Z, 0,
            eye.X, eye.Y, eye.Z, 1);
    }

    /// <summary><paramref name="count"/> poses uniformly distributed on a sphere, all facing the center.</summary>
    public static IReadOnlyList<Matrix4x4> FibonacciSphere(int count, float radius, Vector3 target)
    {
        float golden = MathF.PI * (3f - MathF.Sqrt(5f));
        var poses = new List<Matrix4x4>(count);
        for (int i = 0; i < count; i++)
        {
            float y = 1f - 2f * (i + 0.5f) / count;
            float r = MathF.Sqrt(1f - y * y);
            float phi = golden * i;
            var direction = new Vector3(MathF.Cos(phi) * r, y, MathF.Sin(phi) * r);
            poses.Add(LookAt(target + direction * radius, target));
        }
        return poses;
    }
}
```

`src/Scanner.Core/Synthetic/SyntheticDepthRenderer.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Synthetic;

/// <summary>Generates depth maps via sphere tracing over an SDF.</summary>
public static class SyntheticDepthRenderer
{
    private const int MaxSteps = 256;
    private const float MaxDistance = 2f;
    private const float HitEpsilon = 1e-5f;

    public static DepthFrame Render(ISdf sdf, CameraIntrinsics k, Matrix4x4 cameraToWorld,
        float noiseSigma = 0f, int seed = 0, double timestampSeconds = 0)
    {
        var depth = new float[k.Width * k.Height];
        var eye = cameraToWorld.Translation;

        Parallel.For(0, k.Height, v =>
        {
            for (int u = 0; u < k.Width; u++)
            {
                var rayCamera = new Vector3((u - k.Cx) / k.Fx, (v - k.Cy) / k.Fy, 1f);
                var direction = Vector3.Normalize(Vector3.TransformNormal(rayCamera, cameraToWorld));
                float t = 0f;
                for (int i = 0; i < MaxSteps && t < MaxDistance; i++)
                {
                    float d = sdf.Distance(eye + direction * t);
                    if (d < HitEpsilon)
                    {
                        depth[v * k.Width + u] = t / rayCamera.Length();
                        break;
                    }
                    t += d;
                }
            }
        });

        if (noiseSigma > 0)
        {
            var rng = new Random(seed);
            for (int i = 0; i < depth.Length; i++)
                if (depth[i] > 0) depth[i] += noiseSigma * Gaussian(rng);
        }

        return new DepthFrame(k, depth, cameraToWorld, timestampSeconds);
    }

    private static float Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
```

`src/Scanner.Core/Synthetic/SyntheticScan.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Synthetic;

/// <summary>Simulated scan: views distributed on a sphere around the origin.</summary>
public static class SyntheticScan
{
    public static CameraIntrinsics DefaultIntrinsics => new(320, 240, 300f, 300f, 160f, 120f);

    public static IReadOnlyList<DepthFrame> Capture(ISdf shape, int views, float cameraDistance,
        CameraIntrinsics intrinsics, float noiseSigma, int seed)
    {
        var poses = CameraPoses.FibonacciSphere(views, cameraDistance, Vector3.Zero);
        return poses
            .Select((pose, i) => SyntheticDepthRenderer.Render(shape, intrinsics, pose, noiseSigma, seed + i, i))
            .ToList();
    }
}
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SyntheticTests"`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): DepthFrame types and synthetic scan generator" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Sparse TSDF volume

**Files:**
- Create: `src/Scanner.Core/Fusion/TsdfVolume.cs`
- Test: `tests/Scanner.Core.Tests/Fusion/TsdfVolumeTests.cs`

**Interfaces:**
- Consumes: `DepthFrame`, `CameraIntrinsics` (Task 2); `CameraPoses.LookAt`, `SyntheticDepthRenderer.Render`, `SphereSdf` (test only).
- Produces: `sealed class TsdfVolume(float voxelSize, float truncationDistance)` with
  `float VoxelSize`, `float TruncationDistance`, `int BlockCount`, `IEnumerable<(int X, int Y, int Z)> AllocatedBlocks` (block coordinates),
  `const int BlockSize = 8`, `Vector3 VoxelToWorld(int x, int y, int z)`,
  `bool TryGet(int x, int y, int z, out float tsdf, out float weight)` (true only if weight > 0),
  `void Set(int x, int y, int z, float tsdf, float weight)`, `void Integrate(DepthFrame frame)`.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Core.Tests/Fusion/TsdfVolumeTests.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;
using Scanner.Core.Fusion;
using Scanner.Core.Synthetic;

namespace Scanner.Core.Tests.Fusion;

public class TsdfVolumeTests
{
    [Fact]
    public void Set_and_get_work_with_negative_coordinates()
    {
        var volume = new TsdfVolume(0.002f, 0.006f);

        volume.Set(-1, -9, 17, 0.25f, 3f);

        Assert.True(volume.TryGet(-1, -9, 17, out float tsdf, out float weight));
        Assert.Equal(0.25f, tsdf);
        Assert.Equal(3f, weight);
        Assert.False(volume.TryGet(-2, -9, 17, out _, out _));
        Assert.False(volume.TryGet(100, 100, 100, out _, out _));
        Assert.True(Vector3.Distance(new Vector3(-0.002f, -0.018f, 0.034f), volume.VoxelToWorld(-1, -9, 17)) < 1e-7f);
    }

    [Fact]
    public void Integrate_single_view_sets_signed_values_along_optical_axis()
    {
        var k = new CameraIntrinsics(64, 48, 60f, 60f, 32f, 24f);
        var pose = CameraPoses.LookAt(new Vector3(0, 0, -0.1f), Vector3.Zero);
        var frame = SyntheticDepthRenderer.Render(new SphereSdf(Vector3.Zero, 0.02f), k, pose);
        var volume = new TsdfVolume(0.002f, 0.006f);

        volume.Integrate(frame);

        Assert.True(volume.BlockCount > 0);
        Assert.True(volume.TryGet(0, 0, -12, out float outside, out _));  // 4 mm outside the sphere
        Assert.Equal(0.667f, outside, 2);
        Assert.True(volume.TryGet(0, 0, -8, out float inside, out _));    // 4 mm inside the sphere
        Assert.Equal(-0.667f, inside, 2);
        Assert.False(volume.TryGet(0, 0, 0, out _, out _));               // center: beyond truncation
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~TsdfVolumeTests"`
Expected: compilation FAILURE — `TsdfVolume` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Core/Fusion/TsdfVolume.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Fusion;

/// <summary>
/// Sparse TSDF volume: 8³-voxel blocks allocated only near the observed surface.
/// Voxel (x, y, z) is centered at (x, y, z)·VoxelSize in the world frame.
/// Values normalized to [-1, 1], positive outside the object.
/// </summary>
public sealed class TsdfVolume
{
    public const int BlockSize = 8;
    private const int BlockVoxelCount = BlockSize * BlockSize * BlockSize;
    private const int KeyOffset = 1 << 20;
    private const float MaxWeight = 64f;

    private readonly Dictionary<long, Block> _blocks = new();

    public TsdfVolume(float voxelSize, float truncationDistance)
    {
        if (voxelSize <= 0) throw new ArgumentOutOfRangeException(nameof(voxelSize));
        if (truncationDistance < voxelSize) throw new ArgumentOutOfRangeException(nameof(truncationDistance));
        VoxelSize = voxelSize;
        TruncationDistance = truncationDistance;
    }

    public float VoxelSize { get; }
    public float TruncationDistance { get; }
    public int BlockCount => _blocks.Count;
    public IEnumerable<(int X, int Y, int Z)> AllocatedBlocks => _blocks.Keys.Select(Unpack);

    public Vector3 VoxelToWorld(int x, int y, int z) => new Vector3(x, y, z) * VoxelSize;

    public bool TryGet(int x, int y, int z, out float tsdf, out float weight)
    {
        if (_blocks.TryGetValue(Pack(x >> 3, y >> 3, z >> 3), out var block))
        {
            int i = LocalIndex(x, y, z);
            tsdf = block.Tsdf[i];
            weight = block.Weight[i];
            return weight > 0;
        }
        tsdf = 0;
        weight = 0;
        return false;
    }

    public void Set(int x, int y, int z, float tsdf, float weight)
    {
        var block = GetOrAdd(Pack(x >> 3, y >> 3, z >> 3));
        int i = LocalIndex(x, y, z);
        block.Tsdf[i] = tsdf;
        block.Weight[i] = weight;
    }

    public void Integrate(DepthFrame frame)
    {
        if (!Matrix4x4.Invert(frame.CameraToWorld, out var worldToCamera))
            throw new ArgumentException("Camera pose is not invertible.", nameof(frame));

        var touched = AllocateBlocksNearSurface(frame);
        Parallel.ForEach(touched, key => UpdateBlock(key, _blocks[key], frame, worldToCamera));
    }

    private HashSet<long> AllocateBlocksNearSurface(DepthFrame frame)
    {
        var k = frame.Intrinsics;
        var touched = new HashSet<long>();
        for (int v = 0; v < k.Height; v++)
        for (int u = 0; u < k.Width; u++)
        {
            float d = frame.DepthAt(u, v);
            if (d <= 0) continue;
            var ray = new Vector3((u - k.Cx) / k.Fx, (v - k.Cy) / k.Fy, 1f);
            for (float z = d - TruncationDistance; z <= d + TruncationDistance; z += VoxelSize)
            {
                if (z <= 0) continue;
                var p = Vector3.Transform(ray * z, frame.CameraToWorld) / VoxelSize;
                touched.Add(Pack((int)MathF.Round(p.X) >> 3, (int)MathF.Round(p.Y) >> 3, (int)MathF.Round(p.Z) >> 3));
            }
        }
        foreach (long key in touched) GetOrAdd(key);
        return touched;
    }

    private void UpdateBlock(long key, Block block, DepthFrame frame, Matrix4x4 worldToCamera)
    {
        var k = frame.Intrinsics;
        var (bx, by, bz) = Unpack(key);
        for (int lz = 0; lz < BlockSize; lz++)
        for (int ly = 0; ly < BlockSize; ly++)
        for (int lx = 0; lx < BlockSize; lx++)
        {
            var world = VoxelToWorld(bx * BlockSize + lx, by * BlockSize + ly, bz * BlockSize + lz);
            var pc = Vector3.Transform(world, worldToCamera);
            if (pc.Z <= 0) continue;
            int u = (int)MathF.Round(k.Fx * pc.X / pc.Z + k.Cx);
            int v = (int)MathF.Round(k.Fy * pc.Y / pc.Z + k.Cy);
            if (u < 0 || v < 0 || u >= k.Width || v >= k.Height) continue;
            float d = frame.DepthAt(u, v);
            if (d <= 0) continue;
            float sdf = d - pc.Z;
            if (sdf < -TruncationDistance) continue;

            float tsdf = MathF.Min(1f, sdf / TruncationDistance);
            int i = lx + BlockSize * (ly + BlockSize * lz);
            float w = block.Weight[i];
            block.Tsdf[i] = (block.Tsdf[i] * w + tsdf) / (w + 1);
            block.Weight[i] = MathF.Min(w + 1, MaxWeight);
        }
    }

    private Block GetOrAdd(long key)
    {
        if (!_blocks.TryGetValue(key, out var block))
        {
            block = new Block();
            _blocks[key] = block;
        }
        return block;
    }

    private static long Pack(int bx, int by, int bz) =>
        ((long)(bx + KeyOffset) << 42) | ((long)(by + KeyOffset) << 21) | (long)(bz + KeyOffset);

    private static (int X, int Y, int Z) Unpack(long key) =>
        ((int)(key >> 42) - KeyOffset, (int)((key >> 21) & 0x1FFFFF) - KeyOffset, (int)(key & 0x1FFFFF) - KeyOffset);

    private static int LocalIndex(int x, int y, int z) => (x & 7) + BlockSize * ((y & 7) + BlockSize * (z & 7));

    private sealed class Block
    {
        public readonly float[] Tsdf = new float[BlockVoxelCount];
        public readonly float[] Weight = new float[BlockVoxelCount];
    }
}
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~TsdfVolumeTests"`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): sparse block-based TSDF volume" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Mesh with Naive Surface Nets

**Files:**
- Create: `src/Scanner.Core/Meshing/TriangleMesh.cs`, `src/Scanner.Core/Meshing/SurfaceNets.cs`
- Test: `tests/Scanner.Core.Tests/Meshing/SurfaceNetsTests.cs`

**Interfaces:**
- Consumes: `TsdfVolume` (Task 3).
- Produces:
  - `sealed class TriangleMesh` with `List<Vector3> Positions`, `List<Vector3> Normals` (per vertex, outward, from the TSDF gradient), `List<int> Indices`, `int TriangleCount`, `Vector3 FaceNormal(int triangle)`, `void AddQuad(int a, int b, int c, int d)`.
  - `static TriangleMesh SurfaceNets.Extract(TsdfVolume volume)` — triangles wound counterclockwise as seen from outside.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Core.Tests/Meshing/SurfaceNetsTests.cs`:

```csharp
using System.Numerics;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;

namespace Scanner.Core.Tests.Meshing;

public class SurfaceNetsTests
{
    private const float Radius = 0.02f;

    [Fact]
    public void Sphere_vertices_lie_on_surface_with_outward_normals()
    {
        var mesh = SurfaceNets.Extract(SphereVolume());

        Assert.True(mesh.Positions.Count > 500);
        Assert.Equal(mesh.Positions.Count, mesh.Normals.Count);
        for (int i = 0; i < mesh.Positions.Count; i++)
        {
            var p = mesh.Positions[i];
            Assert.InRange(p.Length(), Radius - 0.0005f, Radius + 0.0005f);
            Assert.True(Vector3.Dot(mesh.Normals[i], Vector3.Normalize(p)) > 0.95f);
        }
    }

    [Fact]
    public void Sphere_triangles_are_wound_outward()
    {
        var mesh = SurfaceNets.Extract(SphereVolume());

        int outward = 0;
        for (int t = 0; t < mesh.TriangleCount; t++)
        {
            var centroid = (mesh.Positions[mesh.Indices[3 * t]] + mesh.Positions[mesh.Indices[3 * t + 1]]
                            + mesh.Positions[mesh.Indices[3 * t + 2]]) / 3f;
            if (Vector3.Dot(mesh.FaceNormal(t), centroid) > 0) outward++;
        }

        Assert.True(mesh.TriangleCount > 1000);
        Assert.True(outward >= mesh.TriangleCount * 0.99);
    }

    private static TsdfVolume SphereVolume()
    {
        var volume = new TsdfVolume(0.002f, 0.006f);
        for (int z = -15; z <= 15; z++)
        for (int y = -15; y <= 15; y++)
        for (int x = -15; x <= 15; x++)
        {
            float sdf = volume.VoxelToWorld(x, y, z).Length() - Radius;
            volume.Set(x, y, z, Math.Clamp(sdf / volume.TruncationDistance, -1f, 1f), 1f);
        }
        return volume;
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SurfaceNetsTests"`
Expected: compilation FAILURE — `Scanner.Core.Meshing` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Core/Meshing/TriangleMesh.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Meshing;

public sealed class TriangleMesh
{
    public List<Vector3> Positions { get; } = new();
    /// <summary>Per-vertex normals, pointing out of the solid.</summary>
    public List<Vector3> Normals { get; } = new();
    public List<int> Indices { get; } = new();

    public int TriangleCount => Indices.Count / 3;

    public Vector3 FaceNormal(int triangle)
    {
        var a = Positions[Indices[3 * triangle]];
        var b = Positions[Indices[3 * triangle + 1]];
        var c = Positions[Indices[3 * triangle + 2]];
        return Vector3.Normalize(Vector3.Cross(b - a, c - a));
    }

    /// <summary>Adds the quad a-b-c-d as two triangles with the same winding.</summary>
    public void AddQuad(int a, int b, int c, int d)
    {
        Indices.AddRange([a, b, c, a, c, d]);
    }
}
```

`src/Scanner.Core/Meshing/SurfaceNets.cs`:

```csharp
using System.Numerics;
using Scanner.Core.Fusion;

namespace Scanner.Core.Meshing;

/// <summary>
/// Naive Surface Nets: one vertex per cell crossed by the surface (average of the intersections
/// on the edges), one quad for each voxel edge with a sign change.
/// Cell (x, y, z) has as its corners the voxels from (x, y, z) to (x+1, y+1, z+1);
/// corner i has offset (i &amp; 1, (i &gt;&gt; 1) &amp; 1, (i &gt;&gt; 2) &amp; 1).
/// </summary>
public static class SurfaceNets
{
    private static readonly int[] EdgeBits = [1, 2, 4];

    public static TriangleMesh Extract(TsdfVolume volume)
    {
        var mesh = new TriangleMesh();
        var cells = new Dictionary<(int, int, int), int>();
        var corners = new float[8];

        foreach (var (bx, by, bz) in volume.AllocatedBlocks)
        for (int lz = 0; lz < TsdfVolume.BlockSize; lz++)
        for (int ly = 0; ly < TsdfVolume.BlockSize; ly++)
        for (int lx = 0; lx < TsdfVolume.BlockSize; lx++)
        {
            int x = bx * TsdfVolume.BlockSize + lx;
            int y = by * TsdfVolume.BlockSize + ly;
            int z = bz * TsdfVolume.BlockSize + lz;
            if (!TryReadCorners(volume, x, y, z, corners) || !HasSignChange(corners)) continue;

            cells[(x, y, z)] = mesh.Positions.Count;
            mesh.Positions.Add(CellVertex(volume.VoxelSize, x, y, z, corners));
            mesh.Normals.Add(CellNormal(corners));
        }

        foreach (var ((x, y, z), _) in cells)
        {
            volume.TryGet(x, y, z, out float v0, out _);
            // Order of cells around the edge: counterclockwise as seen from the positive axis direction.
            if (volume.TryGet(x + 1, y, z, out float vx, out _))
                EmitQuad(mesh, cells, v0, vx, (x, y - 1, z - 1), (x, y, z - 1), (x, y, z), (x, y - 1, z));
            if (volume.TryGet(x, y + 1, z, out float vy, out _))
                EmitQuad(mesh, cells, v0, vy, (x - 1, y, z - 1), (x - 1, y, z), (x, y, z), (x, y, z - 1));
            if (volume.TryGet(x, y, z + 1, out float vz, out _))
                EmitQuad(mesh, cells, v0, vz, (x - 1, y - 1, z), (x, y - 1, z), (x, y, z), (x - 1, y, z));
        }

        return mesh;
    }

    private static void EmitQuad(TriangleMesh mesh, Dictionary<(int, int, int), int> cells, float v0, float v1,
        (int, int, int) c0, (int, int, int) c1, (int, int, int) c2, (int, int, int) c3)
    {
        if ((v0 < 0) == (v1 < 0)) return;
        if (!cells.TryGetValue(c0, out int i0) || !cells.TryGetValue(c1, out int i1)
            || !cells.TryGetValue(c2, out int i2) || !cells.TryGetValue(c3, out int i3)) return;

        // v0 inside and v1 outside: the outward normal has the positive axis direction.
        if (v0 < 0) mesh.AddQuad(i0, i1, i2, i3);
        else mesh.AddQuad(i0, i3, i2, i1);
    }

    private static bool TryReadCorners(TsdfVolume volume, int x, int y, int z, float[] corners)
    {
        for (int i = 0; i < 8; i++)
            if (!volume.TryGet(x + (i & 1), y + ((i >> 1) & 1), z + ((i >> 2) & 1), out corners[i], out _))
                return false;
        return true;
    }

    private static bool HasSignChange(float[] corners)
    {
        bool anyInside = false, anyOutside = false;
        foreach (float c in corners)
        {
            if (c < 0) anyInside = true;
            else anyOutside = true;
        }
        return anyInside && anyOutside;
    }

    private static Vector3 CellVertex(float voxelSize, int x, int y, int z, float[] corners)
    {
        var sum = Vector3.Zero;
        int count = 0;
        for (int i = 0; i < 8; i++)
        foreach (int bit in EdgeBits)
        {
            if ((i & bit) != 0) continue;
            int j = i | bit;
            float a = corners[i], b = corners[j];
            if ((a < 0) == (b < 0)) continue;
            sum += Vector3.Lerp(Offset(i), Offset(j), a / (a - b));
            count++;
        }
        return (new Vector3(x, y, z) + sum / count) * voxelSize;
    }

    // Gradient of the trilinear interpolation at the cell center: points outward.
    private static Vector3 CellNormal(float[] c)
    {
        var g = new Vector3(
            c[1] + c[3] + c[5] + c[7] - (c[0] + c[2] + c[4] + c[6]),
            c[2] + c[3] + c[6] + c[7] - (c[0] + c[1] + c[4] + c[5]),
            c[4] + c[5] + c[6] + c[7] - (c[0] + c[1] + c[2] + c[3]));
        return g.LengthSquared() > 0 ? Vector3.Normalize(g) : Vector3.UnitZ;
    }

    private static Vector3 Offset(int i) => new(i & 1, (i >> 1) & 1, (i >> 2) & 1);
}
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SurfaceNetsTests"`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): mesh extraction with Naive Surface Nets" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Primitives and plane/cylinder RANSAC

**Files:**
- Create: `src/Scanner.Core/Shapes/Primitives.cs`
- Create: `src/Scanner.Core/Segmentation/PointCloud.cs`, `PrimitiveFitter.cs`, `RansacDetector.cs`
- Test: `tests/Scanner.Core.Tests/Segmentation/SampleClouds.cs`, `tests/Scanner.Core.Tests/Segmentation/RansacDetectorTests.cs`

**Interfaces:**
- Consumes: `Basis`, `Linear3`, `SymmetricEigen3` (Task 1); `TriangleMesh` (Task 4).
- Produces:
  - `abstract record Primitive`;
    `sealed record PlanePrimitive(Vector3 Normal, float D) : Primitive` (plane `Normal·x = D`, outward normal) with `float SignedDistance(Vector3 p)`;
    `sealed record CylinderPrimitive(Vector3 AxisPoint, Vector3 Axis, float Radius, bool IsHole) : Primitive` with `Vector3 RadialVector(Vector3 p)`, `float Distance(Vector3 p)`. `IsHole` = normals point toward the axis.
  - `sealed record PointCloud(Vector3[] Points, Vector3[] Normals)` with `int Count`, `static PointCloud FromMesh(TriangleMesh mesh)`.
  - `sealed record RansacOptions(float DistanceThreshold, float NormalThresholdDegrees, int MinInliers, int IterationsPerShape = 1500, int MaxShapes = 20, float MaxCylinderRadius = 0.5f, int Seed = 1)`.
  - `sealed record DetectedShape(Primitive Primitive, int[] InlierIndices)`.
  - `static IReadOnlyList<DetectedShape> RansacDetector.Detect(PointCloud cloud, RansacOptions options)`.
  - `static Primitive? PrimitiveFitter.Refine(Primitive shape, PointCloud cloud, IReadOnlyList<int> indices)`, `FitPlane(...)`, `FitCylinder(...)`.

- [ ] **Step 1: Write the test data and the failing tests**

`tests/Scanner.Core.Tests/Segmentation/SampleClouds.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Segmentation;

namespace Scanner.Core.Tests.Segmentation;

/// <summary>Point clouds sampled directly on known shapes, with exact outward normals.</summary>
internal static class SampleClouds
{
    public static PointCloud Cube(float half, float spacing, float noise, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
        foreach (float sign in new[] { -1f, 1f })
        {
            var n = axis * sign;
            var (u, v) = Basis.Orthonormal(n);
            for (float a = -half; a <= half; a += spacing)
            for (float b = -half; b <= half; b += spacing)
            {
                points.Add(n * half + u * a + v * b + n * (noise * Gaussian(rng)));
                normals.Add(n);
            }
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    public static PointCloud Tube(float outer, float inner, float height, float spacing, float noise, int seed)
    {
        var rng = new Random(seed);
        var points = new List<Vector3>();
        var normals = new List<Vector3>();
        foreach (var (radius, sign) in new[] { (outer, 1f), (inner, -1f) })
        {
            int steps = (int)(2 * MathF.PI * radius / spacing);
            for (int i = 0; i < steps; i++)
            {
                float theta = 2 * MathF.PI * i / steps;
                var radial = new Vector3(MathF.Cos(theta), MathF.Sin(theta), 0);
                for (float z = -height / 2; z <= height / 2; z += spacing)
                {
                    points.Add(radial * (radius + noise * Gaussian(rng)) + new Vector3(0, 0, z));
                    normals.Add(radial * sign);
                }
            }
        }
        foreach (float side in new[] { -1f, 1f })
        for (float x = -outer; x <= outer; x += spacing)
        for (float y = -outer; y <= outer; y += spacing)
        {
            float r = MathF.Sqrt(x * x + y * y);
            if (r < inner || r > outer) continue;
            points.Add(new Vector3(x, y, side * height / 2 + noise * Gaussian(rng)));
            normals.Add(new Vector3(0, 0, side));
        }
        return new PointCloud(points.ToArray(), normals.ToArray());
    }

    private static float Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
```

`tests/Scanner.Core.Tests/Segmentation/RansacDetectorTests.cs`:

```csharp
using System.Numerics;
using Scanner.Core.Segmentation;
using Scanner.Core.Shapes;

namespace Scanner.Core.Tests.Segmentation;

public class RansacDetectorTests
{
    private static readonly RansacOptions Options = new(DistanceThreshold: 0.001f, NormalThresholdDegrees: 20f, MinInliers: 200);

    [Fact]
    public void Cube_yields_six_axis_aligned_planes()
    {
        var cloud = SampleClouds.Cube(0.02f, 0.001f, 0.0002f, seed: 3);

        var shapes = RansacDetector.Detect(cloud, Options);

        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        Assert.Equal(6, shapes.Count);
        Assert.Equal(6, planes.Count);
        foreach (var plane in planes)
        {
            float maxComponent = MathF.Max(MathF.Abs(plane.Normal.X), MathF.Max(MathF.Abs(plane.Normal.Y), MathF.Abs(plane.Normal.Z)));
            Assert.True(maxComponent > MathF.Cos(2f * MathF.PI / 180f));
            Assert.InRange(plane.D, 0.0197f, 0.0203f);
        }
    }

    [Fact]
    public void Tube_yields_outer_cylinder_hole_and_two_caps()
    {
        var cloud = SampleClouds.Tube(0.02f, 0.01f, 0.03f, 0.001f, 0.0002f, seed: 5);

        var shapes = RansacDetector.Detect(cloud, Options);

        var cylinders = shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>().ToList();
        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        Assert.Equal(2, cylinders.Count);
        Assert.Equal(2, planes.Count);

        var outer = Assert.Single(cylinders, c => !c.IsHole);
        var hole = Assert.Single(cylinders, c => c.IsHole);
        Assert.InRange(outer.Radius, 0.0197f, 0.0203f);
        Assert.InRange(hole.Radius, 0.0097f, 0.0103f);
        Assert.True(MathF.Abs(outer.Axis.Z) > 0.999f);
        Assert.True(outer.RadialVector(Vector3.Zero).Length() < 0.0003f);
        Assert.All(planes, p => Assert.InRange(p.D, 0.0147f, 0.0153f));
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~RansacDetectorTests"`
Expected: compilation FAILURE — `Scanner.Core.Segmentation` / `Scanner.Core.Shapes` do not exist.

- [ ] **Step 3: Implement the primitives and the point cloud**

`src/Scanner.Core/Shapes/Primitives.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Shapes;

public abstract record Primitive;

/// <summary>Plane Normal·x = D, with Normal a unit vector pointing out of the solid.</summary>
public sealed record PlanePrimitive(Vector3 Normal, float D) : Primitive
{
    public float SignedDistance(Vector3 p) => Vector3.Dot(Normal, p) - D;
}

/// <summary>Infinite cylinder with axis through AxisPoint in direction Axis (unit vector).
/// IsHole is true when the surface normals point toward the axis (a hole).</summary>
public sealed record CylinderPrimitive(Vector3 AxisPoint, Vector3 Axis, float Radius, bool IsHole) : Primitive
{
    public Vector3 RadialVector(Vector3 p)
    {
        var d = p - AxisPoint;
        return d - Vector3.Dot(d, Axis) * Axis;
    }

    public float Distance(Vector3 p) => MathF.Abs(RadialVector(p).Length() - Radius);
}
```

`src/Scanner.Core/Segmentation/PointCloud.cs`:

```csharp
using System.Numerics;
using Scanner.Core.Meshing;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

public sealed record PointCloud(Vector3[] Points, Vector3[] Normals)
{
    public int Count => Points.Length;

    public static PointCloud FromMesh(TriangleMesh mesh) => new(mesh.Positions.ToArray(), mesh.Normals.ToArray());
}

public sealed record RansacOptions(
    float DistanceThreshold,
    float NormalThresholdDegrees,
    int MinInliers,
    int IterationsPerShape = 1500,
    int MaxShapes = 20,
    float MaxCylinderRadius = 0.5f,
    int Seed = 1);

public sealed record DetectedShape(Primitive Primitive, int[] InlierIndices);
```

- [ ] **Step 4: Implement the least-squares fitting**

`src/Scanner.Core/Segmentation/PrimitiveFitter.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

public static class PrimitiveFitter
{
    public static Primitive? Refine(Primitive shape, PointCloud cloud, IReadOnlyList<int> indices) => shape switch
    {
        PlanePrimitive plane => FitPlane(cloud, indices, plane.Normal),
        CylinderPrimitive cylinder => FitCylinder(cloud, indices, cylinder.Axis),
        _ => null,
    };

    /// <summary>Least-squares plane (minimum eigenvector of the covariance), oriented like <paramref name="orientation"/>.</summary>
    public static PlanePrimitive? FitPlane(PointCloud cloud, IReadOnlyList<int> indices, Vector3 orientation)
    {
        if (indices.Count < 3) return null;
        var centroid = Centroid(cloud.Points, indices);
        var covariance = new double[3, 3];
        foreach (int i in indices) AddOuter(covariance, cloud.Points[i] - centroid);

        var normal = SymmetricEigen3.Solve(covariance).Vectors[0];
        if (Vector3.Dot(normal, orientation) < 0) normal = -normal;
        return new PlanePrimitive(normal, Vector3.Dot(normal, centroid));
    }

    /// <summary>Axis = direction orthogonal to all normals; cross-section = Kåsa circle fit on the projected points.</summary>
    public static CylinderPrimitive? FitCylinder(PointCloud cloud, IReadOnlyList<int> indices, Vector3 axisHint)
    {
        if (indices.Count < 6) return null;
        var normalMoments = new double[3, 3];
        foreach (int i in indices) AddOuter(normalMoments, cloud.Normals[i]);
        var axis = SymmetricEigen3.Solve(normalMoments).Vectors[0];
        if (Vector3.Dot(axis, axisHint) < 0) axis = -axis;

        var (u, v) = Basis.Orthonormal(axis);
        var centroid = Centroid(cloud.Points, indices);
        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
        foreach (int i in indices)
        {
            var d = cloud.Points[i] - centroid;
            double x = Vector3.Dot(d, u), y = Vector3.Dot(d, v), z = x * x + y * y;
            sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
            sxz += x * z; syz += y * z; sz += z;
        }
        var a = new double[,] { { sxx, sxy, sx }, { sxy, syy, sy }, { sx, sy, indices.Count } };
        if (!Linear3.TrySolve(a, new[] { -sxz, -syz, -sz }, out var s)) return null;

        double cx = -s[0] / 2, cy = -s[1] / 2, r2 = cx * cx + cy * cy - s[2];
        if (r2 <= 0) return null;

        var cylinder = new CylinderPrimitive(centroid + u * (float)cx + v * (float)cy, axis, (float)Math.Sqrt(r2), IsHole: false);
        int inward = indices.Count(i => Vector3.Dot(cloud.Normals[i], cylinder.RadialVector(cloud.Points[i])) < 0);
        return cylinder with { IsHole = inward * 2 > indices.Count };
    }

    private static Vector3 Centroid(Vector3[] points, IReadOnlyList<int> indices)
    {
        var sum = Vector3.Zero;
        foreach (int i in indices) sum += points[i];
        return sum / indices.Count;
    }

    private static void AddOuter(double[,] m, Vector3 d)
    {
        double[] a = { d.X, d.Y, d.Z };
        for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m[r, c] += a[r] * a[c];
    }
}
```

- [ ] **Step 5: Implement RANSAC**

`src/Scanner.Core/Segmentation/RansacDetector.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

/// <summary>
/// Sequential RANSAC: each round picks the primitive (plane or cylinder) with the most inliers among the
/// remaining points, refines it via least squares, and removes its inliers.
/// </summary>
public static class RansacDetector
{
    public static IReadOnlyList<DetectedShape> Detect(PointCloud cloud, RansacOptions options)
    {
        var rng = new Random(options.Seed);
        float cosThreshold = MathF.Cos(options.NormalThresholdDegrees * MathF.PI / 180f);
        var remaining = Enumerable.Range(0, cloud.Count).ToList();
        var shapes = new List<DetectedShape>();

        while (remaining.Count >= options.MinInliers && shapes.Count < options.MaxShapes)
        {
            Primitive? best = null;
            int bestScore = 0;
            for (int iteration = 0; iteration < options.IterationsPerShape; iteration++)
            {
                Primitive? candidate = iteration % 2 == 0
                    ? PlaneFromSample(cloud, remaining, rng)
                    : CylinderFromSample(cloud, remaining, rng, options);
                if (candidate is null) continue;
                int score = CountInliers(cloud, remaining, candidate, options.DistanceThreshold, cosThreshold);
                if (score > bestScore)
                {
                    bestScore = score;
                    best = candidate;
                }
            }
            if (best is null || bestScore < options.MinInliers) break;

            var inliers = CollectInliers(cloud, remaining, best, options.DistanceThreshold, cosThreshold);
            for (int pass = 0; pass < 2; pass++)
            {
                var refined = PrimitiveFitter.Refine(best, cloud, inliers);
                if (refined is null) break;
                best = refined;
                inliers = CollectInliers(cloud, remaining, best, options.DistanceThreshold, cosThreshold);
            }
            if (inliers.Count < options.MinInliers) break;

            shapes.Add(new DetectedShape(best, inliers.ToArray()));
            var taken = inliers.ToHashSet();
            remaining.RemoveAll(taken.Contains);
        }
        return shapes;
    }

    public static bool IsInlier(Primitive shape, Vector3 p, Vector3 n, float distance, float cosThreshold) => shape switch
    {
        PlanePrimitive plane => MathF.Abs(plane.SignedDistance(p)) <= distance && Vector3.Dot(plane.Normal, n) >= cosThreshold,
        CylinderPrimitive cylinder => IsCylinderInlier(cylinder, p, n, distance, cosThreshold),
        _ => false,
    };

    private static bool IsCylinderInlier(CylinderPrimitive cylinder, Vector3 p, Vector3 n, float distance, float cosThreshold)
    {
        var radial = cylinder.RadialVector(p);
        float length = radial.Length();
        if (length < 1e-9f || MathF.Abs(length - cylinder.Radius) > distance) return false;
        float alignment = Vector3.Dot(n, radial / length);
        return cylinder.IsHole ? alignment <= -cosThreshold : alignment >= cosThreshold;
    }

    private static int CountInliers(PointCloud cloud, List<int> remaining, Primitive shape, float distance, float cosThreshold)
    {
        int count = 0;
        foreach (int i in remaining)
            if (IsInlier(shape, cloud.Points[i], cloud.Normals[i], distance, cosThreshold)) count++;
        return count;
    }

    private static List<int> CollectInliers(PointCloud cloud, List<int> remaining, Primitive shape, float distance, float cosThreshold) =>
        remaining.Where(i => IsInlier(shape, cloud.Points[i], cloud.Normals[i], distance, cosThreshold)).ToList();

    private static PlanePrimitive PlaneFromSample(PointCloud cloud, List<int> remaining, Random rng)
    {
        int i = remaining[rng.Next(remaining.Count)];
        var n = cloud.Normals[i];
        return new PlanePrimitive(n, Vector3.Dot(n, cloud.Points[i]));
    }

    // Two points with normals: axis = n1 × n2; the cross-section center is the intersection of the projected normals.
    private static CylinderPrimitive? CylinderFromSample(PointCloud cloud, List<int> remaining, Random rng, RansacOptions options)
    {
        int i = remaining[rng.Next(remaining.Count)];
        int j = remaining[rng.Next(remaining.Count)];
        if (i == j) return null;
        Vector3 p1 = cloud.Points[i], p2 = cloud.Points[j], n1 = cloud.Normals[i], n2 = cloud.Normals[j];

        var cross = Vector3.Cross(n1, n2);
        if (cross.Length() < 0.2f) return null;
        var axis = Vector3.Normalize(cross);
        var (u, v) = Basis.Orthonormal(axis);

        var q1 = new Vector2(Vector3.Dot(p1, u), Vector3.Dot(p1, v));
        var q2 = new Vector2(Vector3.Dot(p2, u), Vector3.Dot(p2, v));
        var m1 = Vector2.Normalize(new Vector2(Vector3.Dot(n1, u), Vector3.Dot(n1, v)));
        var m2 = Vector2.Normalize(new Vector2(Vector3.Dot(n2, u), Vector3.Dot(n2, v)));

        // q1 + t1·m1 = q2 + t2·m2
        float det = -m1.X * m2.Y + m2.X * m1.Y;
        if (MathF.Abs(det) < 1e-6f) return null;
        var r = q2 - q1;
        float t1 = (-r.X * m2.Y + m2.X * r.Y) / det;
        var center = q1 + t1 * m1;
        float radius = (Vector2.Distance(q1, center) + Vector2.Distance(q2, center)) / 2;
        if (radius < options.DistanceThreshold || radius > options.MaxCylinderRadius) return null;

        var cylinder = new CylinderPrimitive(u * center.X + v * center.Y, axis, radius, IsHole: false);
        bool isHole = Vector3.Dot(n1, cylinder.RadialVector(p1)) < 0;
        return cylinder with { IsHole = isHole };
    }
}
```

- [ ] **Step 6: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~RansacDetectorTests"`
Expected: PASS (2 tests).

- [ ] **Step 7: Run the whole Core suite and commit**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: PASS (all).

```bash
git add -A
git commit -m "feat(core): primitives and plane/cylinder RANSAC with refinement" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: B-Rep model, validator and convex polyhedron

**Files:**
- Create: `src/Scanner.Brep/Model/BrepModel.cs`, `src/Scanner.Brep/Model/BrepValidator.cs`
- Create: `src/Scanner.Brep/Builders/ConvexPolyhedronBuilder.cs`
- Test: `tests/Scanner.Brep.Tests/Builders/ConvexPolyhedronBuilderTests.cs`

**Interfaces:**
- Consumes: `PlanePrimitive` (Task 5), `Basis`, `Linear3` (Task 1).
- Produces:
  - `sealed class BrepVertex(Vector3 position)` → `Position`.
  - `abstract record BrepCurve`; `sealed record LineCurve(Vector3 Origin, Vector3 Direction)`; `sealed record CircleCurve(Vector3 Center, Vector3 Axis, Vector3 RefDirection, float Radius)` (counterclockwise around `Axis`).
  - `sealed class BrepEdge(BrepVertex start, BrepVertex end, BrepCurve curve)` → `Start`, `End`, `Curve`.
  - `readonly record struct OrientedEdge(BrepEdge Edge, bool SameSense)` → `StartVertex`, `EndVertex`.
  - `sealed class BrepLoop(IReadOnlyList<OrientedEdge> edges)` → `Edges`.
  - `abstract record BrepSurface`; `sealed record PlaneSurface(Vector3 Origin, Vector3 Normal, Vector3 RefDirection)`; `sealed record CylinderSurface(Vector3 Origin, Vector3 Axis, Vector3 RefDirection, float Radius)` (normal pointing away from the axis).
  - `sealed class BrepFace(BrepSurface surface, IReadOnlyList<BrepLoop> loops, bool sameSense)` → `Surface`, `Loops` (for planar faces `Loops[0]` is the outer boundary), `SameSense`.
  - `sealed class BrepSolid(IReadOnlyList<BrepFace> faces)` → `Faces`, `IReadOnlyList<BrepEdge> DistinctEdges()`, `IReadOnlyList<BrepVertex> DistinctVertices()`.
  - `static IReadOnlyList<string> BrepValidator.Validate(BrepSolid solid)` — empty list if valid.
  - `static BrepSolid ConvexPolyhedronBuilder.Build(IReadOnlyList<PlanePrimitive> planes, float tolerance)` — throws `InvalidOperationException` if degenerate.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Brep.Tests/Builders/ConvexPolyhedronBuilderTests.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Tests.Builders;

public class ConvexPolyhedronBuilderTests
{
    internal static List<PlanePrimitive> CubePlanes(float half) =>
    [
        new(Vector3.UnitX, half), new(-Vector3.UnitX, half),
        new(Vector3.UnitY, half), new(-Vector3.UnitY, half),
        new(Vector3.UnitZ, half), new(-Vector3.UnitZ, half),
    ];

    [Fact]
    public void Cube_has_expected_topology_and_is_valid()
    {
        var solid = ConvexPolyhedronBuilder.Build(CubePlanes(0.02f), 1e-4f);

        Assert.Equal(6, solid.Faces.Count);
        Assert.Equal(12, solid.DistinctEdges().Count);
        Assert.Equal(8, solid.DistinctVertices().Count);
        Assert.All(solid.Faces, f => Assert.Equal(4, Assert.Single(f.Loops).Edges.Count));
        Assert.All(solid.DistinctVertices(), v =>
            Assert.True(Vector3.Distance(Vector3.Abs(v.Position), new Vector3(0.02f)) < 1e-6f));
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Face_loops_run_counterclockwise_around_outward_normal()
    {
        var solid = ConvexPolyhedronBuilder.Build(CubePlanes(0.02f), 1e-4f);

        foreach (var face in solid.Faces)
        {
            var normal = ((PlaneSurface)face.Surface).Normal;
            var loop = face.Loops[0].Edges;
            var a = loop[0].StartVertex.Position;
            var b = loop[1].StartVertex.Position;
            var c = loop[2].StartVertex.Position;
            Assert.True(Vector3.Dot(Vector3.Cross(b - a, c - b), normal) > 0);
        }
    }

    [Fact]
    public void Near_duplicate_planes_are_merged()
    {
        var planes = CubePlanes(0.02f);
        planes.Add(new PlanePrimitive(Vector3.Normalize(new Vector3(1f, 0.01f, 0)), 0.02001f));

        var solid = ConvexPolyhedronBuilder.Build(planes, 1e-4f);

        Assert.Equal(6, solid.Faces.Count);
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Too_few_planes_throw()
    {
        Assert.Throws<InvalidOperationException>(() =>
            ConvexPolyhedronBuilder.Build(CubePlanes(0.02f).Take(3).ToList(), 1e-4f));
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: compilation FAILURE — `Scanner.Brep.Builders` / `Scanner.Brep.Model` do not exist.

- [ ] **Step 3: Implement the model**

`src/Scanner.Brep/Model/BrepModel.cs`:

```csharp
using System.Numerics;

namespace Scanner.Brep.Model;

public sealed class BrepVertex(Vector3 position)
{
    public Vector3 Position { get; } = position;
}

public abstract record BrepCurve;

/// <summary>Line through Origin with unit direction Direction.</summary>
public sealed record LineCurve(Vector3 Origin, Vector3 Direction) : BrepCurve;

/// <summary>Circle in the plane orthogonal to Axis, traversed counterclockwise around Axis starting from RefDirection.</summary>
public sealed record CircleCurve(Vector3 Center, Vector3 Axis, Vector3 RefDirection, float Radius) : BrepCurve;

public sealed class BrepEdge(BrepVertex start, BrepVertex end, BrepCurve curve)
{
    public BrepVertex Start { get; } = start;
    public BrepVertex End { get; } = end;
    public BrepCurve Curve { get; } = curve;
}

public readonly record struct OrientedEdge(BrepEdge Edge, bool SameSense)
{
    public BrepVertex StartVertex => SameSense ? Edge.Start : Edge.End;
    public BrepVertex EndVertex => SameSense ? Edge.End : Edge.Start;
}

/// <summary>Closed cycle of oriented edges; the face interior is on the left when looking along the face normal.</summary>
public sealed class BrepLoop(IReadOnlyList<OrientedEdge> edges)
{
    public IReadOnlyList<OrientedEdge> Edges { get; } = edges;
}

public abstract record BrepSurface;

public sealed record PlaneSurface(Vector3 Origin, Vector3 Normal, Vector3 RefDirection) : BrepSurface;

/// <summary>Cylindrical surface; its normal points away from the axis.</summary>
public sealed record CylinderSurface(Vector3 Origin, Vector3 Axis, Vector3 RefDirection, float Radius) : BrepSurface;

/// <summary>Face bounded by one or more loops; for planar faces Loops[0] is the outer boundary.
/// SameSense = false flips the surface normal.</summary>
public sealed class BrepFace(BrepSurface surface, IReadOnlyList<BrepLoop> loops, bool sameSense)
{
    public BrepSurface Surface { get; } = surface;
    public IReadOnlyList<BrepLoop> Loops { get; } = loops;
    public bool SameSense { get; } = sameSense;
}

public sealed class BrepSolid(IReadOnlyList<BrepFace> faces)
{
    public IReadOnlyList<BrepFace> Faces { get; } = faces;

    public IReadOnlyList<BrepEdge> DistinctEdges() =>
        Faces.SelectMany(f => f.Loops).SelectMany(l => l.Edges).Select(e => e.Edge).Distinct().ToList();

    public IReadOnlyList<BrepVertex> DistinctVertices() =>
        DistinctEdges().SelectMany(e => new[] { e.Start, e.End }).Distinct().ToList();
}
```

`src/Scanner.Brep/Model/BrepValidator.cs`:

```csharp
namespace Scanner.Brep.Model;

/// <summary>Minimal topological checks for a closed manifold solid.</summary>
public static class BrepValidator
{
    public static IReadOnlyList<string> Validate(BrepSolid solid)
    {
        var errors = new List<string>();
        var uses = new Dictionary<BrepEdge, (int Forward, int Backward)>();
        int loopCount = 0;

        for (int f = 0; f < solid.Faces.Count; f++)
        foreach (var loop in solid.Faces[f].Loops)
        {
            loopCount++;
            if (loop.Edges.Count == 0)
            {
                errors.Add($"Face {f}: empty loop.");
                continue;
            }
            for (int i = 0; i < loop.Edges.Count; i++)
            {
                var current = loop.Edges[i];
                var next = loop.Edges[(i + 1) % loop.Edges.Count];
                if (current.EndVertex != next.StartVertex)
                    errors.Add($"Face {f}: loop not closed after edge {i}.");

                uses.TryGetValue(current.Edge, out var count);
                uses[current.Edge] = current.SameSense ? (count.Forward + 1, count.Backward) : (count.Forward, count.Backward + 1);
            }
        }

        foreach (var (_, count) in uses)
            if (count.Forward != 1 || count.Backward != 1)
                errors.Add($"Edge used {count.Forward} times forward and {count.Backward} backward (expected 1 and 1).");

        int vertices = solid.DistinctVertices().Count;
        int faces = solid.Faces.Count;
        // Euler-Poincaré: V − E + F − (L − F) = 2(S − G), with S = 1 shell.
        int chi = vertices - uses.Count + faces - (loopCount - faces);
        if (chi > 2 || chi % 2 != 0)
            errors.Add($"Invalid Euler-Poincaré characteristic: {chi}.");

        return errors;
    }
}
```

- [ ] **Step 4: Implement the convex polyhedron builder**

`src/Scanner.Brep/Builders/ConvexPolyhedronBuilder.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Builders;

/// <summary>Convex solid as the intersection of the half-spaces Normal·x ≤ D.</summary>
public static class ConvexPolyhedronBuilder
{
    private const float DuplicateAngleDegrees = 5f;

    public static BrepSolid Build(IReadOnlyList<PlanePrimitive> planes, float tolerance)
    {
        var unique = MergeDuplicates(planes, tolerance);
        if (unique.Count < 4) throw new InvalidOperationException($"At least 4 distinct planes are required, found {unique.Count}.");

        var points = IntersectionVertices(unique, tolerance);
        if (points.Count < 4) throw new InvalidOperationException("Degenerate polyhedron: fewer than 4 vertices.");

        var vertices = points.Select(p => new BrepVertex(p)).ToList();
        var edges = new Dictionary<(int, int), BrepEdge>();
        var faces = new List<BrepFace>();

        foreach (var plane in unique)
        {
            var onPlane = Enumerable.Range(0, points.Count)
                .Where(i => MathF.Abs(plane.SignedDistance(points[i])) <= tolerance)
                .ToList();
            if (onPlane.Count < 3) continue;

            var centroid = onPlane.Aggregate(Vector3.Zero, (sum, i) => sum + points[i]) / onPlane.Count;
            var (u, v) = Basis.Orthonormal(plane.Normal);
            // Increasing angle in the (u, v) basis with u × v = normal: counterclockwise as seen from outside.
            var ordered = onPlane
                .OrderBy(i => MathF.Atan2(Vector3.Dot(points[i] - centroid, v), Vector3.Dot(points[i] - centroid, u)))
                .ToList();

            var loop = new List<OrientedEdge>();
            for (int k = 0; k < ordered.Count; k++)
            {
                int a = ordered[k], b = ordered[(k + 1) % ordered.Count];
                var key = (Math.Min(a, b), Math.Max(a, b));
                if (!edges.TryGetValue(key, out var edge))
                {
                    var start = points[key.Item1];
                    var end = points[key.Item2];
                    edge = new BrepEdge(vertices[key.Item1], vertices[key.Item2], new LineCurve(start, Vector3.Normalize(end - start)));
                    edges[key] = edge;
                }
                loop.Add(new OrientedEdge(edge, SameSense: a == key.Item1));
            }
            faces.Add(new BrepFace(new PlaneSurface(centroid, plane.Normal, u), [new BrepLoop(loop)], sameSense: true));
        }

        return new BrepSolid(faces);
    }

    private static List<PlanePrimitive> MergeDuplicates(IReadOnlyList<PlanePrimitive> planes, float tolerance)
    {
        float cosLimit = MathF.Cos(DuplicateAngleDegrees * MathF.PI / 180f);
        var unique = new List<PlanePrimitive>();
        foreach (var plane in planes)
        {
            bool duplicate = unique.Any(u =>
                Vector3.Dot(u.Normal, plane.Normal) > cosLimit && MathF.Abs(u.D - plane.D) < 3 * tolerance + 1e-3f);
            if (!duplicate) unique.Add(plane);
        }
        return unique;
    }

    private static List<Vector3> IntersectionVertices(List<PlanePrimitive> planes, float tolerance)
    {
        var points = new List<Vector3>();
        for (int i = 0; i < planes.Count; i++)
        for (int j = i + 1; j < planes.Count; j++)
        for (int k = j + 1; k < planes.Count; k++)
        {
            PlanePrimitive a = planes[i], b = planes[j], c = planes[k];
            var m = new double[,]
            {
                { a.Normal.X, a.Normal.Y, a.Normal.Z },
                { b.Normal.X, b.Normal.Y, b.Normal.Z },
                { c.Normal.X, c.Normal.Y, c.Normal.Z },
            };
            if (!Linear3.TrySolve(m, new double[] { a.D, b.D, c.D }, out var x)) continue;

            var p = new Vector3((float)x[0], (float)x[1], (float)x[2]);
            if (planes.Any(plane => plane.SignedDistance(p) > tolerance)) continue;
            if (points.Any(q => Vector3.Distance(p, q) <= tolerance)) continue;
            points.Add(p);
        }
        return points;
    }
}
```

Note: the merge threshold on `D` is `3·tolerance + 1 mm` because two RANSAC planes from the same face can differ by roughly one distance threshold; `tolerance` (0.1 mm) is instead used for vertices.

- [ ] **Step 5: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: PASS (4 tests).

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat(brep): B-Rep model, topology validator and convex polyhedron" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Tube builder (cylinder with coaxial hole)

**Files:**
- Create: `src/Scanner.Brep/Builders/TubeBuilder.cs`
- Test: `tests/Scanner.Brep.Tests/Builders/TubeBuilderTests.cs`

**Interfaces:**
- Consumes: the B-Rep model and `BrepValidator` (Task 6), `Basis` (Task 1).
- Produces: `static BrepSolid TubeBuilder.Build(Vector3 axisPoint, Vector3 axis, float outerRadius, float? innerRadius, float zBottom, float zTop)` — `z` measured along `axis` starting from `axisPoint`; faces in order: outer, [hole], bottom cap, top cap.

- [ ] **Step 1: Write the failing tests**

`tests/Scanner.Brep.Tests/Builders/TubeBuilderTests.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;

namespace Scanner.Brep.Tests.Builders;

public class TubeBuilderTests
{
    [Fact]
    public void Tube_has_four_faces_and_valid_genus_one_topology()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        Assert.Equal(4, solid.Faces.Count);
        Assert.Equal(4, solid.DistinctEdges().Count);
        Assert.Equal(4, solid.DistinctVertices().Count);
        Assert.Equal(2, solid.Faces.Count(f => f.Surface is CylinderSurface));
        Assert.Single(solid.Faces, f => f.Surface is CylinderSurface && !f.SameSense);
        Assert.All(solid.Faces.Where(f => f.Surface is PlaneSurface), f => Assert.Equal(2, f.Loops.Count));
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Solid_cylinder_without_hole_is_valid()
    {
        var solid = TubeBuilder.Build(new Vector3(0.01f, 0, 0), Vector3.UnitY, 0.02f, null, 0f, 0.05f);

        Assert.Equal(3, solid.Faces.Count);
        Assert.Empty(BrepValidator.Validate(solid));
    }

    [Fact]
    public void Cap_planes_face_away_from_each_other_along_axis()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        var caps = solid.Faces.Select(f => f.Surface).OfType<PlaneSurface>().ToList();
        Assert.Contains(caps, p => p.Normal == -Vector3.UnitZ && MathF.Abs(p.Origin.Z + 0.015f) < 1e-6f);
        Assert.Contains(caps, p => p.Normal == Vector3.UnitZ && MathF.Abs(p.Origin.Z - 0.015f) < 1e-6f);
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~TubeBuilderTests"`
Expected: compilation FAILURE — `TubeBuilder` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Brep/Builders/TubeBuilder.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Brep.Builders;

/// <summary>
/// Outer cylinder with an optional coaxial hole and two planar caps orthogonal to the axis.
/// Each circle is a single closed edge (start vertex = end vertex), traversed counterclockwise around the axis.
/// Orientations (face interior on the left when looking along the face normal):
/// outer: bottom +, top −; hole (SameSense=false): bottom −, top +;
/// bottom cap (normal −axis): outer −, hole +; top cap (normal +axis): outer +, hole −.
/// </summary>
public static class TubeBuilder
{
    public static BrepSolid Build(Vector3 axisPoint, Vector3 axis, float outerRadius, float? innerRadius, float zBottom, float zTop)
    {
        if (zTop <= zBottom) throw new ArgumentException("zTop must be greater than zBottom.");
        if (innerRadius is { } r && (r <= 0 || r >= outerRadius)) throw new ArgumentException("Invalid hole radius.");

        var a = Vector3.Normalize(axis);
        var (refDirection, _) = Basis.Orthonormal(a);
        var bottom = axisPoint + a * zBottom;
        var top = axisPoint + a * zTop;

        var outerBottom = Circle(bottom, a, refDirection, outerRadius);
        var outerTop = Circle(top, a, refDirection, outerRadius);

        var faces = new List<BrepFace>
        {
            new(new CylinderSurface(bottom, a, refDirection, outerRadius),
                [Loop(outerBottom, true), Loop(outerTop, false)], sameSense: true),
        };

        var bottomLoops = new List<BrepLoop> { Loop(outerBottom, false) };
        var topLoops = new List<BrepLoop> { Loop(outerTop, true) };

        if (innerRadius is { } inner)
        {
            var innerBottom = Circle(bottom, a, refDirection, inner);
            var innerTop = Circle(top, a, refDirection, inner);
            faces.Add(new BrepFace(new CylinderSurface(bottom, a, refDirection, inner),
                [Loop(innerBottom, false), Loop(innerTop, true)], sameSense: false));
            bottomLoops.Add(Loop(innerBottom, true));
            topLoops.Add(Loop(innerTop, false));
        }

        faces.Add(new BrepFace(new PlaneSurface(bottom, -a, refDirection), bottomLoops, sameSense: true));
        faces.Add(new BrepFace(new PlaneSurface(top, a, refDirection), topLoops, sameSense: true));
        return new BrepSolid(faces);
    }

    private static BrepEdge Circle(Vector3 center, Vector3 axis, Vector3 refDirection, float radius)
    {
        var vertex = new BrepVertex(center + refDirection * radius);
        return new BrepEdge(vertex, vertex, new CircleCurve(center, axis, refDirection, radius));
    }

    private static BrepLoop Loop(BrepEdge edge, bool sameSense) => new([new OrientedEdge(edge, sameSense)]);
}
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~TubeBuilderTests"`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(brep): coaxial tube/cylinder builder" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: STEP AP214 writer

**Files:**
- Create: `src/Scanner.Brep/Step/StepWriter.cs`
- Test: `tests/Scanner.Brep.Tests/Step/StepSyntaxChecker.cs`, `tests/Scanner.Brep.Tests/Step/StepWriterTests.cs`

**Interfaces:**
- Consumes: the B-Rep model (Task 6), `ConvexPolyhedronBuilder` (Task 6), `TubeBuilder` (Task 7).
- Produces: `static string StepWriter.Write(BrepSolid solid, string productName, DateTime timestampUtc)` — complete ISO 10303-21 file, coordinates in mm.

- [ ] **Step 1: Write the syntax checker and the failing tests**

`tests/Scanner.Brep.Tests/Step/StepSyntaxChecker.cs`:

```csharp
using System.Text.RegularExpressions;

namespace Scanner.Brep.Tests.Step;

/// <summary>Structural checks on a STEP file: header, unique ids, resolved references, terminated lines.</summary>
internal static class StepSyntaxChecker
{
    public static IReadOnlyList<string> Check(string step)
    {
        var errors = new List<string>();
        var text = step.Trim();
        if (!text.StartsWith("ISO-10303-21;")) errors.Add("Missing ISO-10303-21 header.");
        if (!text.EndsWith("END-ISO-10303-21;")) errors.Add("Missing END-ISO-10303-21 closing.");
        if (!text.Contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));")) errors.Add("Missing AP214 schema.");

        var defined = new HashSet<int>();
        foreach (Match m in Regex.Matches(step, @"^#(\d+)=", RegexOptions.Multiline))
            if (!defined.Add(int.Parse(m.Groups[1].Value))) errors.Add($"Duplicate id #{m.Groups[1].Value}.");

        foreach (Match m in Regex.Matches(step, @"#(\d+)"))
            if (!defined.Contains(int.Parse(m.Groups[1].Value))) errors.Add($"Undefined reference #{m.Groups[1].Value}.");

        foreach (var line in step.Split('\n').Where(l => l.StartsWith('#')))
        {
            if (!line.TrimEnd().EndsWith(';')) errors.Add($"Unterminated line: {line}");
            if (line.Count(c => c == '(') != line.Count(c => c == ')')) errors.Add($"Unbalanced parentheses: {line}");
        }
        return errors;
    }

    public static int Count(string step, string entity) => Regex.Matches(step, $@"={entity}\(").Count;
}
```

`tests/Scanner.Brep.Tests/Step/StepWriterTests.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Step;
using Scanner.Brep.Tests.Builders;

namespace Scanner.Brep.Tests.Step;

public class StepWriterTests
{
    private static readonly DateTime Timestamp = new(2026, 9, 19, 12, 0, 0, DateTimeKind.Utc);

    [Fact]
    public void Cube_step_is_well_formed_with_expected_entities()
    {
        var solid = ConvexPolyhedronBuilder.Build(ConvexPolyhedronBuilderTests.CubePlanes(0.02f), 1e-4f);

        var step = StepWriter.Write(solid, "cube", Timestamp);

        Assert.Empty(StepSyntaxChecker.Check(step));
        Assert.Equal(1, StepSyntaxChecker.Count(step, "MANIFOLD_SOLID_BREP"));
        Assert.Equal(1, StepSyntaxChecker.Count(step, "CLOSED_SHELL"));
        Assert.Equal(6, StepSyntaxChecker.Count(step, "ADVANCED_FACE"));
        Assert.Equal(6, StepSyntaxChecker.Count(step, "PLANE"));
        Assert.Equal(12, StepSyntaxChecker.Count(step, "EDGE_CURVE"));
        Assert.Equal(12, StepSyntaxChecker.Count(step, "LINE"));
        Assert.Equal(8, StepSyntaxChecker.Count(step, "VERTEX_POINT"));
        Assert.Contains("CARTESIAN_POINT('',(20.0,20.0,20.0))", step);
        Assert.Contains("SI_UNIT(.MILLI.,.METRE.)", step);
    }

    [Fact]
    public void Tube_step_uses_cylinders_and_circles()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        var step = StepWriter.Write(solid, "tube", Timestamp);

        Assert.Empty(StepSyntaxChecker.Check(step));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "ADVANCED_FACE"));
        Assert.Equal(2, StepSyntaxChecker.Count(step, "CYLINDRICAL_SURFACE"));
        Assert.Equal(2, StepSyntaxChecker.Count(step, "PLANE"));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "CIRCLE"));
        Assert.Equal(4, StepSyntaxChecker.Count(step, "EDGE_CURVE"));
        Assert.Equal(0, StepSyntaxChecker.Count(step, "LINE"));
        Assert.Contains("CYLINDRICAL_SURFACE('',#", step);
        Assert.Contains(",20.0)", step);
        Assert.Contains(",10.0)", step);
    }

    [Fact]
    public void Output_is_deterministic()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, 0.01f, -0.015f, 0.015f);

        Assert.Equal(StepWriter.Write(solid, "tube", Timestamp), StepWriter.Write(solid, "tube", Timestamp));
    }

    [Fact]
    public void Product_name_quotes_are_escaped()
    {
        var solid = TubeBuilder.Build(Vector3.Zero, Vector3.UnitZ, 0.02f, null, 0f, 0.01f);

        var step = StepWriter.Write(solid, "user's ring", Timestamp);

        Assert.Contains("PRODUCT('user''s ring','user''s ring'", step);
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~StepWriterTests"`
Expected: compilation FAILURE — `Scanner.Brep.Step` does not exist.

- [ ] **Step 3: Implement**

`src/Scanner.Brep/Step/StepWriter.cs`:

```csharp
using System.Globalization;
using System.Numerics;
using System.Text;
using Scanner.Brep.Model;

namespace Scanner.Brep.Step;

/// <summary>Writes a <see cref="BrepSolid"/> as a STEP AP214 file (ISO 10303-21) in millimeters.</summary>
public sealed class StepWriter
{
    private const double MetersToMillimeters = 1000.0;

    private readonly StringBuilder _data = new();
    private readonly Dictionary<object, int> _ids = new(ReferenceEqualityComparer.Instance);
    private int _nextId = 1;

    private StepWriter()
    {
    }

    public static string Write(BrepSolid solid, string productName, DateTime timestampUtc) =>
        new StepWriter().WriteFile(solid, productName, timestampUtc);

    private string WriteFile(BrepSolid solid, string productName, DateTime timestampUtc)
    {
        string name = Escape(productName);

        int app = Add("APPLICATION_CONTEXT('automotive design')");
        Add($"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})");
        int productContext = Add($"PRODUCT_CONTEXT('',#{app},'mechanical')");
        int product = Add($"PRODUCT('{name}','{name}','',(#{productContext}))");
        Add($"PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))");
        int formation = Add($"PRODUCT_DEFINITION_FORMATION('','',#{product})");
        int definitionContext = Add($"PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')");
        int definition = Add($"PRODUCT_DEFINITION('design','',#{formation},#{definitionContext})");
        int shape = Add($"PRODUCT_DEFINITION_SHAPE('','',#{definition})");

        int length = Add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        int angle = Add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        int solidAngle = Add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
        int uncertainty = Add($"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-03),#{length},'distance_accuracy_value','confusion accuracy')");
        int context = Add("(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
                          + $"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty}))"
                          + $"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solidAngle}))"
                          + "REPRESENTATION_CONTEXT('Context3D','3D Context'))");

        int brep = WriteSolid(solid);
        int origin = WriteAxis(Vector3.Zero, Vector3.UnitZ, Vector3.UnitX);
        int representation = Add($"ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',(#{origin},#{brep}),#{context})");
        Add($"SHAPE_DEFINITION_REPRESENTATION(#{shape},#{representation})");

        var file = new StringBuilder();
        file.Append("ISO-10303-21;\n");
        file.Append("HEADER;\n");
        file.Append("FILE_DESCRIPTION(('Scan3D model'),'2;1');\n");
        file.Append($"FILE_NAME('{name}.stp','{timestampUtc.ToString("yyyy-MM-ddTHH:mm:ss", CultureInfo.InvariantCulture)}',('Scan3D'),(''),'Scan3D','Scan3D','');\n");
        file.Append("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
        file.Append("ENDSEC;\n");
        file.Append("DATA;\n");
        file.Append(_data);
        file.Append("ENDSEC;\n");
        file.Append("END-ISO-10303-21;\n");
        return file.ToString();
    }

    private int WriteSolid(BrepSolid solid)
    {
        var faces = solid.Faces.Select(WriteFace).ToList();
        int shell = Add($"CLOSED_SHELL('',({Refs(faces)}))");
        return Add($"MANIFOLD_SOLID_BREP('',#{shell})");
    }

    private int WriteFace(BrepFace face)
    {
        int surface = WriteSurface(face.Surface);
        var bounds = new List<int>();
        for (int i = 0; i < face.Loops.Count; i++)
        {
            int loop = WriteLoop(face.Loops[i]);
            string kind = i == 0 && face.Surface is PlaneSurface ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            bounds.Add(Add($"{kind}('',#{loop},.T.)"));
        }
        return Add($"ADVANCED_FACE('',({Refs(bounds)}),#{surface},{Bool(face.SameSense)})");
    }

    private int WriteLoop(BrepLoop loop)
    {
        var oriented = loop.Edges
            .Select(e => Add($"ORIENTED_EDGE('',*,*,#{WriteEdge(e.Edge)},{Bool(e.SameSense)})"))
            .ToList();
        return Add($"EDGE_LOOP('',({Refs(oriented)}))");
    }

    private int WriteEdge(BrepEdge edge)
    {
        if (_ids.TryGetValue(edge, out int id)) return id;
        int start = WriteVertex(edge.Start);
        int end = WriteVertex(edge.End);
        int curve = WriteCurve(edge.Curve);
        id = Add($"EDGE_CURVE('',#{start},#{end},#{curve},.T.)");
        _ids[edge] = id;
        return id;
    }

    private int WriteVertex(BrepVertex vertex)
    {
        if (_ids.TryGetValue(vertex, out int id)) return id;
        id = Add($"VERTEX_POINT('',#{WritePoint(vertex.Position)})");
        _ids[vertex] = id;
        return id;
    }

    private int WriteCurve(BrepCurve curve) => curve switch
    {
        LineCurve line => Add($"LINE('',#{WritePoint(line.Origin)},#{Add($"VECTOR('',#{WriteDirection(line.Direction)},1.0)")})"),
        CircleCurve circle => Add($"CIRCLE('',#{WriteAxis(circle.Center, circle.Axis, circle.RefDirection)},{Length(circle.Radius)})"),
        _ => throw new NotSupportedException($"Unsupported curve: {curve.GetType().Name}"),
    };

    private int WriteSurface(BrepSurface surface) => surface switch
    {
        PlaneSurface plane => Add($"PLANE('',#{WriteAxis(plane.Origin, plane.Normal, plane.RefDirection)})"),
        CylinderSurface cylinder => Add($"CYLINDRICAL_SURFACE('',#{WriteAxis(cylinder.Origin, cylinder.Axis, cylinder.RefDirection)},{Length(cylinder.Radius)})"),
        _ => throw new NotSupportedException($"Unsupported surface: {surface.GetType().Name}"),
    };

    private int WriteAxis(Vector3 location, Vector3 axis, Vector3 refDirection) =>
        Add($"AXIS2_PLACEMENT_3D('',#{WritePoint(location)},#{WriteDirection(axis)},#{WriteDirection(refDirection)})");

    private int WritePoint(Vector3 p) =>
        Add($"CARTESIAN_POINT('',({Length(p.X)},{Length(p.Y)},{Length(p.Z)}))");

    private int WriteDirection(Vector3 d)
    {
        var n = Vector3.Normalize(d);
        return Add($"DIRECTION('',({Real(n.X)},{Real(n.Y)},{Real(n.Z)}))");
    }

    private int Add(string entity)
    {
        int id = _nextId++;
        _data.Append('#').Append(id).Append('=').Append(entity).Append(";\n");
        return id;
    }

    private static string Refs(IEnumerable<int> ids) => string.Join(",", ids.Select(i => $"#{i}"));

    private static string Bool(bool value) => value ? ".T." : ".F.";

    private static string Length(float meters) => Real(meters * MetersToMillimeters);

    // Rounding to 1e-6 (nm for lengths) to avoid float noise like 19.9999995.
    private static string Real(double value) =>
        Math.Round(value, 6).ToString("0.0#####", CultureInfo.InvariantCulture);

    private static string Escape(string text) => text.Replace("'", "''");
}
```

Note: `Length(float)` multiplies in `double` (`meters * 1000.0`) — `0.02f` becomes `19.99999955…`, rounded to `20.0`.

- [ ] **Step 4: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: PASS (all, 11 tests).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(brep): STEP AP214 writer in millimeters" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 9: Mechanical reconstruction and full scan → STEP pipeline

**Files:**
- Create: `src/Scanner.Core/ProcessingProfile.cs`
- Create: `src/Scanner.Brep/Reconstruction/MechanicalReconstructor.cs`, `src/Scanner.Brep/Reconstruction/ScanToStep.cs`
- Test: `tests/Scanner.Brep.Tests/Reconstruction/MechanicalReconstructorTests.cs`, `tests/Scanner.Brep.Tests/Reconstruction/ScanToStepTests.cs`

**Interfaces:**
- Consumes: `TsdfVolume` (T3), `SurfaceNets` (T4), `PointCloud`/`RansacOptions`/`RansacDetector`/`DetectedShape`/primitives (T5), builders and validator (T6, T7), `StepWriter` (T8), `SyntheticScan`/`BoxSdf`/`TubeSdf` (T2, in tests).
- Produces:
  - `sealed record ProcessingProfile(float VoxelSize, float TruncationVoxels, float RansacDistance, float NormalThresholdDegrees, float MinInlierFraction, int IterationsPerShape, int Seed)` with `static ProcessingProfile Draft` and `static ProcessingProfile Fine`.
  - `sealed record ReconstructionOptions(float VertexTolerance = 1e-4f, float CoaxialTolerance = 0.002f, float AngleToleranceDegrees = 3f)`.
  - `sealed record ReconstructionResult(BrepSolid? Solid, string? FailureReason)`.
  - `static ReconstructionResult MechanicalReconstructor.Reconstruct(IReadOnlyList<DetectedShape> shapes, ReconstructionOptions options)`.
  - `sealed record ScanResult(TriangleMesh Mesh, IReadOnlyList<DetectedShape> Shapes, BrepSolid? Solid, string? Step, string? FailureReason)` with `bool Succeeded`.
  - `static ScanResult ScanToStep.Run(IEnumerable<DepthFrame> frames, ProcessingProfile profile, string productName, CancellationToken cancellationToken = default)`.

- [ ] **Step 1: Write the reconstructor tests (failing)**

`tests/Scanner.Brep.Tests/Reconstruction/MechanicalReconstructorTests.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Brep.Reconstruction;
using Scanner.Brep.Tests.Builders;
using Scanner.Core.Segmentation;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Tests.Reconstruction;

public class MechanicalReconstructorTests
{
    private static DetectedShape Shape(Primitive p) => new(p, []);

    [Fact]
    public void Planes_only_build_a_polyhedron()
    {
        var shapes = ConvexPolyhedronBuilderTests.CubePlanes(0.02f).Select(Shape).ToList();

        var result = MechanicalReconstructor.Reconstruct(shapes, new ReconstructionOptions());

        Assert.Null(result.FailureReason);
        Assert.Equal(6, result.Solid!.Faces.Count);
    }

    [Fact]
    public void Coaxial_cylinders_and_caps_build_a_tube()
    {
        var shapes = new List<DetectedShape>
        {
            Shape(new CylinderPrimitive(new Vector3(0.0005f, 0, 0.01f), Vector3.UnitZ, 0.02f, IsHole: false)),
            Shape(new CylinderPrimitive(Vector3.Zero, -Vector3.UnitZ, 0.01f, IsHole: true)),
            Shape(new PlanePrimitive(Vector3.UnitZ, 0.015f)),
            Shape(new PlanePrimitive(-Vector3.UnitZ, 0.015f)),
        };

        var result = MechanicalReconstructor.Reconstruct(shapes, new ReconstructionOptions());

        Assert.Null(result.FailureReason);
        Assert.Equal(4, result.Solid!.Faces.Count);
        var caps = result.Solid.Faces.Select(f => f.Surface).OfType<PlaneSurface>().ToList();
        Assert.Contains(caps, c => MathF.Abs(c.Origin.Z + 0.015f) < 1e-5f);
        Assert.Contains(caps, c => MathF.Abs(c.Origin.Z - 0.015f) < 1e-5f);
    }

    [Fact]
    public void Non_coaxial_hole_is_reported_not_thrown()
    {
        var shapes = new List<DetectedShape>
        {
            Shape(new CylinderPrimitive(Vector3.Zero, Vector3.UnitZ, 0.02f, IsHole: false)),
            Shape(new CylinderPrimitive(new Vector3(0.008f, 0, 0), Vector3.UnitZ, 0.005f, IsHole: true)),
            Shape(new PlanePrimitive(Vector3.UnitZ, 0.015f)),
            Shape(new PlanePrimitive(-Vector3.UnitZ, 0.015f)),
        };

        var result = MechanicalReconstructor.Reconstruct(shapes, new ReconstructionOptions());

        Assert.Null(result.Solid);
        Assert.Contains("coaxial", result.FailureReason!);
    }

    [Fact]
    public void Too_few_planes_are_reported()
    {
        var shapes = ConvexPolyhedronBuilderTests.CubePlanes(0.02f).Take(3).Select(Shape).ToList();

        var result = MechanicalReconstructor.Reconstruct(shapes, new ReconstructionOptions());

        Assert.Null(result.Solid);
        Assert.NotNull(result.FailureReason);
    }
}
```

- [ ] **Step 2: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~MechanicalReconstructorTests"`
Expected: compilation FAILURE — `Scanner.Brep.Reconstruction` does not exist.

- [ ] **Step 3: Implement the profile and the reconstructor**

`src/Scanner.Core/ProcessingProfile.cs`:

```csharp
namespace Scanner.Core;

/// <summary>Pipeline parameters. Draft for the phone, Fine for the desktop.</summary>
public sealed record ProcessingProfile(
    float VoxelSize,
    float TruncationVoxels,
    float RansacDistance,
    float NormalThresholdDegrees,
    float MinInlierFraction,
    int IterationsPerShape,
    int Seed)
{
    public static ProcessingProfile Draft => new(0.004f, 3f, 0.004f, 25f, 0.03f, 800, 1);
    public static ProcessingProfile Fine => new(0.0015f, 3f, 0.0015f, 20f, 0.03f, 1500, 1);
}
```

`src/Scanner.Brep/Reconstruction/MechanicalReconstructor.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Builders;
using Scanner.Brep.Model;
using Scanner.Core.Segmentation;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Reconstruction;

public sealed record ReconstructionOptions(float VertexTolerance = 1e-4f, float CoaxialTolerance = 0.002f, float AngleToleranceDegrees = 3f);

public sealed record ReconstructionResult(BrepSolid? Solid, string? FailureReason);

/// <summary>
/// Mechanical mode (v0): planes only → convex polyhedron; one outer cylinder + optional coaxial hole
/// + two orthogonal caps → tube. Every other case is reported as unsupported, never as an exception.
/// </summary>
public static class MechanicalReconstructor
{
    public static ReconstructionResult Reconstruct(IReadOnlyList<DetectedShape> shapes, ReconstructionOptions options)
    {
        var planes = shapes.Select(s => s.Primitive).OfType<PlanePrimitive>().ToList();
        var cylinders = shapes.Select(s => s.Primitive).OfType<CylinderPrimitive>().ToList();
        try
        {
            var solid = cylinders.Count == 0
                ? ConvexPolyhedronBuilder.Build(planes, options.VertexTolerance)
                : BuildTube(planes, cylinders, options);
            var errors = BrepValidator.Validate(solid);
            return errors.Count == 0
                ? new ReconstructionResult(solid, null)
                : new ReconstructionResult(null, "Invalid B-Rep: " + string.Join("; ", errors));
        }
        catch (Exception ex) when (ex is NotSupportedException or InvalidOperationException or ArgumentException)
        {
            return new ReconstructionResult(null, ex.Message);
        }
    }

    private static BrepSolid BuildTube(List<PlanePrimitive> planes, List<CylinderPrimitive> cylinders, ReconstructionOptions options)
    {
        float cosTolerance = MathF.Cos(options.AngleToleranceDegrees * MathF.PI / 180f);
        var outer = cylinders.Where(c => !c.IsHole).ToList();
        var holes = cylinders.Where(c => c.IsHole).ToList();
        if (outer.Count != 1 || holes.Count > 1)
            throw new NotSupportedException(
                $"Only one outer cylinder and at most one hole are supported (found {outer.Count} outer and {holes.Count} holes).");

        var main = outer[0];
        var axis = main.Axis;
        var hole = holes.SingleOrDefault();
        if (hole is not null)
        {
            bool parallel = MathF.Abs(Vector3.Dot(hole.Axis, axis)) >= cosTolerance;
            float offset = main.RadialVector(hole.AxisPoint).Length();
            if (!parallel || offset > options.CoaxialTolerance)
                throw new NotSupportedException("The hole is not coaxial with the outer cylinder.");
        }

        var caps = planes.Where(p => MathF.Abs(Vector3.Dot(p.Normal, axis)) >= cosTolerance).ToList();
        if (caps.Count != 2 || planes.Count != 2)
            throw new NotSupportedException(
                $"Exactly two planes orthogonal to the axis are required (found {caps.Count} out of {planes.Count}).");

        // Axis point AxisPoint + h·axis on the plane n·x = d  ⇒  h = (d − n·AxisPoint) / (n·axis)
        var heights = caps
            .Select(p => (p.D - Vector3.Dot(p.Normal, main.AxisPoint)) / Vector3.Dot(p.Normal, axis))
            .OrderBy(h => h)
            .ToArray();
        return TubeBuilder.Build(main.AxisPoint, axis, main.Radius, hole?.Radius, heights[0], heights[1]);
    }
}
```

- [ ] **Step 4: Run the reconstructor tests and verify they pass**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~MechanicalReconstructorTests"`
Expected: PASS (4 tests).

- [ ] **Step 5: Write the end-to-end tests (failing)**

`tests/Scanner.Brep.Tests/Reconstruction/ScanToStepTests.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Brep.Reconstruction;
using Scanner.Brep.Tests.Step;
using Scanner.Core;
using Scanner.Core.Synthetic;

namespace Scanner.Brep.Tests.Reconstruction;

public class ScanToStepTests
{
    private const int Views = 40;
    private const float CameraDistance = 0.12f;
    private const float Noise = 0.0003f;

    [Fact]
    public void Synthetic_cube_scan_becomes_a_six_face_step()
    {
        var frames = SyntheticScan.Capture(new BoxSdf(Vector3.Zero, new Vector3(0.02f)), Views, CameraDistance,
            SyntheticScan.DefaultIntrinsics, Noise, seed: 11);

        var result = ScanToStep.Run(frames, ProcessingProfile.Fine, "cube");

        Assert.True(result.Succeeded, result.FailureReason ?? string.Empty);
        Assert.Equal(6, result.Solid!.Faces.Count);
        Assert.All(result.Solid.DistinctVertices(), v =>
        {
            Assert.InRange(MathF.Abs(v.Position.X), 0.019f, 0.021f);
            Assert.InRange(MathF.Abs(v.Position.Y), 0.019f, 0.021f);
            Assert.InRange(MathF.Abs(v.Position.Z), 0.019f, 0.021f);
        });
        Assert.Empty(StepSyntaxChecker.Check(result.Step!));
    }

    [Fact]
    public void Synthetic_tube_scan_becomes_a_cylindrical_step()
    {
        var frames = SyntheticScan.Capture(new TubeSdf(Vector3.Zero, 0.02f, 0.01f, 0.03f), Views, CameraDistance,
            SyntheticScan.DefaultIntrinsics, Noise, seed: 13);

        var result = ScanToStep.Run(frames, ProcessingProfile.Fine, "tube");

        Assert.True(result.Succeeded, result.FailureReason ?? string.Empty);
        var cylinders = result.Solid!.Faces.Select(f => f.Surface).OfType<CylinderSurface>().OrderBy(c => c.Radius).ToList();
        Assert.Equal(2, cylinders.Count);
        Assert.InRange(cylinders[0].Radius, 0.009f, 0.011f);
        Assert.InRange(cylinders[1].Radius, 0.019f, 0.021f);
        var capHeights = result.Solid.Faces.Select(f => f.Surface).OfType<PlaneSurface>()
            .Select(p => MathF.Abs(Vector3.Dot(p.Origin, cylinders[1].Axis))).ToList();
        Assert.All(capHeights, h => Assert.InRange(h, 0.014f, 0.016f));
        Assert.Equal(2, StepSyntaxChecker.Count(result.Step!, "CYLINDRICAL_SURFACE"));
    }

    [Fact]
    public void Cancellation_is_honoured()
    {
        var frames = SyntheticScan.Capture(new BoxSdf(Vector3.Zero, new Vector3(0.02f)), 2, CameraDistance,
            SyntheticScan.DefaultIntrinsics, 0f, seed: 1);
        using var cts = new CancellationTokenSource();
        cts.Cancel();

        Assert.Throws<OperationCanceledException>(() => ScanToStep.Run(frames, ProcessingProfile.Fine, "x", cts.Token));
    }
}
```

- [ ] **Step 6: Run the tests and verify they fail**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~ScanToStepTests"`
Expected: compilation FAILURE — `ScanToStep` does not exist.

- [ ] **Step 7: Implement the pipeline**

`src/Scanner.Brep/Reconstruction/ScanToStep.cs`:

```csharp
using Scanner.Brep.Model;
using Scanner.Brep.Step;
using Scanner.Capture;
using Scanner.Core;
using Scanner.Core.Fusion;
using Scanner.Core.Meshing;
using Scanner.Core.Segmentation;

namespace Scanner.Brep.Reconstruction;

public sealed record ScanResult(TriangleMesh Mesh, IReadOnlyList<DetectedShape> Shapes, BrepSolid? Solid, string? Step, string? FailureReason)
{
    public bool Succeeded => Step is not null;
}

/// <summary>Full Mechanical-mode pipeline: fusion → mesh → RANSAC → B-Rep → STEP.</summary>
public static class ScanToStep
{
    private const int AbsoluteMinInliers = 30;

    public static ScanResult Run(IEnumerable<DepthFrame> frames, ProcessingProfile profile, string productName,
        CancellationToken cancellationToken = default)
    {
        var volume = new TsdfVolume(profile.VoxelSize, profile.VoxelSize * profile.TruncationVoxels);
        foreach (var frame in frames)
        {
            cancellationToken.ThrowIfCancellationRequested();
            volume.Integrate(frame);
        }

        cancellationToken.ThrowIfCancellationRequested();
        var mesh = SurfaceNets.Extract(volume);

        cancellationToken.ThrowIfCancellationRequested();
        var cloud = PointCloud.FromMesh(mesh);
        var options = new RansacOptions(
            profile.RansacDistance,
            profile.NormalThresholdDegrees,
            Math.Max(AbsoluteMinInliers, (int)(cloud.Count * profile.MinInlierFraction)),
            profile.IterationsPerShape,
            Seed: profile.Seed);
        var shapes = RansacDetector.Detect(cloud, options);

        cancellationToken.ThrowIfCancellationRequested();
        var reconstruction = MechanicalReconstructor.Reconstruct(shapes, new ReconstructionOptions());
        if (reconstruction.Solid is null)
            return new ScanResult(mesh, shapes, null, null, reconstruction.FailureReason);

        var step = StepWriter.Write(reconstruction.Solid, productName, DateTime.UtcNow);
        return new ScanResult(mesh, shapes, reconstruction.Solid, step, null);
    }
}
```

- [ ] **Step 8: Run the tests and verify they pass**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~ScanToStepTests"`
Expected: PASS (3 tests).

If an end-to-end test fails, the message contains `FailureReason`. Typical diagnoses:
- "Only one outer cylinder and at most one hole are supported…" or "Exactly two planes orthogonal to the axis are required…" on the cube/tube → RANSAC found spurious shapes on the rounded edges: raise `MinInlierFraction` in `ProcessingProfile.Fine` to `0.05f`.
- Fewer than 6 planes on the cube → lower `MinInlierFraction` to `0.02f` or increase `IterationsPerShape` to `3000`.
First check by printing `result.Shapes` (type, parameters, inlier count) in a temporary test; do not change the thresholds without having seen which shape is wrong.

- [ ] **Step 9: Run all suites and commit**

Run: `dotnet test Scan3D.slnx`
Expected: PASS (all Core and Brep tests).

```bash
git add -A
git commit -m "feat(brep): mechanical reconstruction and scan → STEP pipeline" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 10: CLI for sample files + Fusion checklist

**Files:**
- Create: `tools/Scanner.Cli/Scanner.Cli.csproj` (via CLI), `tools/Scanner.Cli/Program.cs`
- Create: `docs/fusion-checklist.md`
- Modify: `.gitignore` (add `out/`)

**Interfaces:**
- Consumes: `SyntheticScan`, `BoxSdf`, `TubeSdf` (T2), `ProcessingProfile` (T9), `ScanToStep.Run` (T9).
- Produces: command `dotnet run --project tools/Scanner.Cli -- <cube|tube> <file.stp>`; exit code 0 if the STEP is written, 1 for incorrect usage, 2 if reconstruction fails.

- [ ] **Step 1: Create the project**

```bash
dotnet new console -n Scanner.Cli -o tools/Scanner.Cli -f net10.0
dotnet sln add tools/Scanner.Cli
dotnet add tools/Scanner.Cli reference src/Scanner.Brep
echo "out/" >> .gitignore
```

- [ ] **Step 2: Write `Program.cs`**

`tools/Scanner.Cli/Program.cs` (replaces the generated content):

```csharp
using System.Diagnostics;
using System.Numerics;
using Scanner.Brep.Reconstruction;
using Scanner.Core;
using Scanner.Core.Shapes;
using Scanner.Core.Synthetic;

if (args.Length != 2 || args[0] is not ("cube" or "tube"))
{
    Console.Error.WriteLine("Usage: Scanner.Cli <cube|tube> <file.stp>");
    return 1;
}

ISdf shape = args[0] == "cube"
    ? new BoxSdf(Vector3.Zero, new Vector3(0.02f))
    : new TubeSdf(Vector3.Zero, 0.02f, 0.01f, 0.03f);

var stopwatch = Stopwatch.StartNew();
var frames = SyntheticScan.Capture(shape, 40, 0.12f, SyntheticScan.DefaultIntrinsics, 0.0003f, seed: 7);
var result = ScanToStep.Run(frames, ProcessingProfile.Fine, Path.GetFileNameWithoutExtension(args[1]));

Console.WriteLine($"Mesh: {result.Mesh.Positions.Count} vertices, {result.Mesh.TriangleCount} triangles");
foreach (var detected in result.Shapes)
{
    string description = detected.Primitive switch
    {
        PlanePrimitive p => $"plane    n=({p.Normal.X:F3},{p.Normal.Y:F3},{p.Normal.Z:F3}) d={p.D * 1000:F2} mm",
        CylinderPrimitive c => $"cylinder r={c.Radius * 1000:F2} mm axis=({c.Axis.X:F3},{c.Axis.Y:F3},{c.Axis.Z:F3}) hole={c.IsHole}",
        _ => detected.Primitive.GetType().Name,
    };
    Console.WriteLine($"  {description}  inlier={detected.InlierIndices.Length}");
}

if (!result.Succeeded)
{
    Console.Error.WriteLine($"Reconstruction failed: {result.FailureReason}");
    return 2;
}

Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(args[1]))!);
File.WriteAllText(args[1], result.Step);
Console.WriteLine($"Wrote {args[1]} ({result.Solid!.Faces.Count} faces) in {stopwatch.Elapsed.TotalSeconds:F1} s");
return 0;
```

- [ ] **Step 3: Run the CLI and verify the output**

Run: `dotnet run --project tools/Scanner.Cli -- cube out/cube.stp`
Expected: 6 `plane` lines with `d≈20.00 mm`, then `Wrote out/cube.stp (6 faces)`; exit code 0.

Run: `dotnet run --project tools/Scanner.Cli -- tube out/tube.stp`
Expected: two `cylinder` lines (r≈20 mm hole=False, r≈10 mm hole=True), two `plane` lines with `d≈15.00 mm`, then `Wrote out/tube.stp (4 faces)`; exit code 0.

Run: `dotnet run --project tools/Scanner.Cli -- sphere out/x.stp`
Expected: usage message, exit code 1.

- [ ] **Step 4: Write the Fusion checklist**

`docs/fusion-checklist.md`:

```markdown
# Manual verification in Autodesk Fusion

Generate the files:

    dotnet run --project tools/Scanner.Cli -- cube out/cube.stp
    dotnet run --project tools/Scanner.Cli -- tube out/tube.stp

For each file: Fusion → File → Open → "Open from my computer…" → select the `.stp`.

## cube.stp
- [ ] Imports without repair warnings.
- [ ] Appears in the Browser under **Bodies** (not "Mesh Bodies").
- [ ] 6 planar faces; Inspect → Measure between opposite faces ≈ 40 mm (±1 mm).
- [ ] Modify → Press Pull on a face works.
- [ ] Modify → Fillet on an edge (2 mm) works.

## tube.stp
- [ ] Imports without repair warnings and is a **solid body**.
- [ ] Outer diameter ≈ 40 mm, hole ≈ 20 mm, height ≈ 30 mm (Inspect → Measure).
- [ ] Selecting the hole, Fusion recognizes it as a cylindrical face (shows the diameter).
- [ ] Modify → Chamfer on the top circular edge works.

Note the Fusion version, date, and any issues at the bottom of this file.
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(cli): sample STEP generator and Fusion verification checklist" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 6: Manual verification (user)**

Ask the user to open `out/cube.stp` and `out/tube.stp` in Fusion following `docs/fusion-checklist.md` and report the outcome. The milestone is closed only once both open as editable solid bodies.
