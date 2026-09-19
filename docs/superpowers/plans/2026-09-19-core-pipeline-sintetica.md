# Pipeline core sintetica → STEP — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Da scansioni di profondità sintetiche di un cubo e di un tubo (cilindro forato) produrre file STEP AP214 con superfici analitiche, validi e apribili in Autodesk Fusion.

**Architecture:** Librerie .NET pure (`Scanner.Capture`, `Scanner.Core`, `Scanner.Brep`) senza dipendenze da piattaforma. Pipeline: frame di profondità → fusione TSDF sparsa → Naive Surface Nets (mesh + normali da gradiente TSDF) → RANSAC piani/cilindri con raffinamento ai minimi quadrati → costruzione B-Rep (poliedro convesso da piani, oppure tubo coassiale) → validazione topologica → writer STEP in C#. Un piccolo CLI genera i file di esempio per la verifica manuale in Fusion.

**Tech Stack:** .NET 10 (`net10.0`), C# latest, `System.Numerics`, xUnit.

**Spec:** `docs/superpowers/specs/2026-09-19-scan3d-design.md` (sezioni 2, 4, 5, 9; milestone 2)

## Global Constraints

- Tutti i progetti: `net10.0`, `Nullable` enable, `ImplicitUsings` enable, `TreatWarningsAsErrors` true.
- `Scanner.Core` e `Scanner.Brep` non hanno **alcuna** dipendenza da piattaforma o da MAUI.
- Unità interne: **metri**. Lo STEP è scritto in **millimetri** (`SI_UNIT(.MILLI.,.METRE.)`).
- Convenzione camera: OpenCV (x destra, y giù, z avanti); `Matrix4x4` di System.Numerics a vettori riga (`Vector3.Transform(p, cameraToWorld)`).
- TSDF normalizzata in [-1, 1], **positiva fuori** dall'oggetto; le normali puntano fuori dal solido.
- Schema STEP: `AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }` (AP214).
- Numeri STEP formattati con `CultureInfo.InvariantCulture`, sempre con punto decimale e senza esponente.
- Lavoro diretto su `main`; ogni commit termina con la riga `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- Nessun `Xunit` using esplicito nei test: il template xUnit aggiunge `<Using Include="Xunit" />` globale.

**Deviazioni consapevoli dalla spec (da riportare all'utente):**
- Meshing con **Naive Surface Nets** invece di Marching Cubes: nessuna tabella da 256 casi, mesh più regolare; stessa interfaccia, sostituibile in seguito.
- In questo piano il costruttore B-Rep "Meccanico" copre solo **poliedri convessi** (solo piani) e **tubi/cilindri coassiali** con due tappi ortogonali. Il costruttore generale per intersezione di primitive, lo STEP "a faccette" di ripiego e i golden file arrivano in piani successivi.

## File Structure

```
Scan3D.slnx
Directory.Build.props
.gitignore
src/Scanner.Capture/
  CameraIntrinsics.cs          intrinseci pinhole
  DepthFrame.cs                frame di profondità + posa
src/Scanner.Core/
  ProcessingProfile.cs         profili Bozza/Fine
  LinearAlgebra/Basis.cs       base ortonormale di un piano
  LinearAlgebra/Linear3.cs     sistema lineare 3x3
  LinearAlgebra/SymmetricEigen3.cs  autovalori 3x3 simmetrica (Jacobi)
  Synthetic/ISdf.cs            SphereSdf, BoxSdf, TubeSdf
  Synthetic/CameraPoses.cs     LookAt, pose su sfera di Fibonacci
  Synthetic/SyntheticDepthRenderer.cs  sphere tracing → DepthFrame
  Synthetic/SyntheticScan.cs   scansione simulata multi-vista
  Fusion/TsdfVolume.cs         TSDF sparsa a blocchi 8³
  Meshing/TriangleMesh.cs      mesh con normali per vertice
  Meshing/SurfaceNets.cs       estrazione isosuperficie
  Shapes/Primitives.cs         PlanePrimitive, CylinderPrimitive
  Segmentation/PointCloud.cs   PointCloud, RansacOptions, DetectedShape
  Segmentation/PrimitiveFitter.cs  fitting ai minimi quadrati
  Segmentation/RansacDetector.cs   RANSAC sequenziale piani/cilindri
src/Scanner.Brep/
  Model/BrepModel.cs           vertici, spigoli, loop, facce, solido
  Model/BrepValidator.cs       validazione topologica
  Builders/ConvexPolyhedronBuilder.cs
  Builders/TubeBuilder.cs
  Step/StepWriter.cs           ISO 10303-21 AP214
  Reconstruction/MechanicalReconstructor.cs  forme → B-Rep
  Reconstruction/ScanToStep.cs frame → STEP
tests/Scanner.Core.Tests/…     un file di test per componente
tests/Scanner.Brep.Tests/…     un file di test per componente + StepSyntaxChecker
tools/Scanner.Cli/Program.cs   genera cube/tube .stp
docs/fusion-checklist.md       verifica manuale in Fusion
```

---

### Task 1: Scaffold della solution + algebra lineare

**Files:**
- Create: `Directory.Build.props`, `Scan3D.slnx` (via CLI), `.gitignore` (via CLI)
- Create: `src/Scanner.Capture/Scanner.Capture.csproj`, `src/Scanner.Core/Scanner.Core.csproj`, `src/Scanner.Brep/Scanner.Brep.csproj`
- Create: `tests/Scanner.Core.Tests/Scanner.Core.Tests.csproj`, `tests/Scanner.Brep.Tests/Scanner.Brep.Tests.csproj`
- Create: `src/Scanner.Core/LinearAlgebra/Basis.cs`, `Linear3.cs`, `SymmetricEigen3.cs`
- Test: `tests/Scanner.Core.Tests/LinearAlgebra/LinearAlgebraTests.cs`

**Interfaces:**
- Produces:
  - `static (Vector3 U, Vector3 V) Basis.Orthonormal(Vector3 n)` — `n` unitario, `U × V = n`.
  - `static bool Linear3.TrySolve(double[,] a, double[] b, out double[] x)`; `static double Linear3.Det(double[,] m)`.
  - `static (double[] Values, Vector3[] Vectors) SymmetricEigen3.Solve(double[,] matrix)` — autovalori crescenti, autovettori unitari.

- [ ] **Step 1: Creare la solution e i progetti**

Da `C:\Workspace\scan3dapp`:

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

Verificare che entrambi i `.csproj` di test contengano `<Using Include="Xunit" />`; se manca, aggiungerlo in un `<ItemGroup>`.

- [ ] **Step 2: Creare `Directory.Build.props`**

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

(Il `TargetFramework` resta nei singoli `.csproj`: metterlo qui romperebbe il multi-targeting della futura app MAUI.)

- [ ] **Step 3: Scrivere i test che falliscono**

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

- [ ] **Step 4: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: FAIL di compilazione — `The type or namespace name 'LinearAlgebra' does not exist`.

- [ ] **Step 5: Implementare**

`src/Scanner.Core/LinearAlgebra/Basis.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.LinearAlgebra;

public static class Basis
{
    /// <summary>Base ortonormale (U, V) del piano ortogonale a <paramref name="n"/> (unitario), con U × V = n.</summary>
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
    /// <summary>Risolve A·x = b con la regola di Cramer. Restituisce false se A è quasi singolare.</summary>
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

/// <summary>Autovalori e autovettori di una matrice 3x3 simmetrica (metodo di Jacobi).</summary>
public static class SymmetricEigen3
{
    /// <summary>Autovalori in ordine crescente con i relativi autovettori unitari.</summary>
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

    // A ← Pᵀ·A·P, V ← V·P con P rotazione di Jacobi nel piano (p, q).
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

- [ ] **Step 6: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: PASS (8 test). Poi `dotnet build Scan3D.slnx` senza warning.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat(core): scaffold solution e algebra lineare 3x3" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Tipi di acquisizione + generatore di scansioni sintetiche

**Files:**
- Create: `src/Scanner.Capture/CameraIntrinsics.cs`, `src/Scanner.Capture/DepthFrame.cs`
- Create: `src/Scanner.Core/Synthetic/ISdf.cs`, `CameraPoses.cs`, `SyntheticDepthRenderer.cs`, `SyntheticScan.cs`
- Test: `tests/Scanner.Core.Tests/Synthetic/SyntheticTests.cs`

**Interfaces:**
- Consumes: niente.
- Produces:
  - `readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy)`
  - `sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)` con `float DepthAt(int u, int v)`; profondità z in metri, 0 = non valida.
  - `interface ISdf { float Distance(Vector3 p); }`; `SphereSdf(Vector3 Center, float Radius)`, `BoxSdf(Vector3 Center, Vector3 HalfSize)`, `TubeSdf(Vector3 Center, float OuterRadius, float InnerRadius, float Height)` (asse Z).
  - `static Matrix4x4 CameraPoses.LookAt(Vector3 eye, Vector3 target)`; `static IReadOnlyList<Matrix4x4> CameraPoses.FibonacciSphere(int count, float radius, Vector3 target)`.
  - `static DepthFrame SyntheticDepthRenderer.Render(ISdf sdf, CameraIntrinsics k, Matrix4x4 cameraToWorld, float noiseSigma = 0f, int seed = 0, double timestampSeconds = 0)`.
  - `static CameraIntrinsics SyntheticScan.DefaultIntrinsics` (320×240, f=300); `static IReadOnlyList<DepthFrame> SyntheticScan.Capture(ISdf shape, int views, float cameraDistance, CameraIntrinsics intrinsics, float noiseSigma, int seed)`.

- [ ] **Step 1: Scrivere i test che falliscono**

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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SyntheticTests"`
Expected: FAIL di compilazione — `Scanner.Capture` / `Scanner.Core.Synthetic` inesistenti.

- [ ] **Step 3: Implementare**

`src/Scanner.Capture/CameraIntrinsics.cs`:

```csharp
namespace Scanner.Capture;

/// <summary>Intrinseci pinhole in pixel, convenzione OpenCV (x destra, y giù, z avanti).</summary>
public readonly record struct CameraIntrinsics(int Width, int Height, float Fx, float Fy, float Cx, float Cy);
```

`src/Scanner.Capture/DepthFrame.cs`:

```csharp
using System.Numerics;

namespace Scanner.Capture;

/// <summary>Mappa di profondità (z in metri, 0 = non valida) con la posa camera→mondo al momento dello scatto.</summary>
public sealed record DepthFrame(CameraIntrinsics Intrinsics, float[] Depth, Matrix4x4 CameraToWorld, double TimestampSeconds)
{
    public float DepthAt(int u, int v) => Depth[v * Intrinsics.Width + u];
}
```

`src/Scanner.Core/Synthetic/ISdf.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Synthetic;

/// <summary>Funzione di distanza con segno: negativa dentro l'oggetto.</summary>
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

/// <summary>Cilindro forato con asse Z centrato in <see cref="Center"/>.</summary>
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
    /// <summary>Posa camera→mondo (convenzione OpenCV) che guarda <paramref name="target"/> da <paramref name="eye"/>.</summary>
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

    /// <summary><paramref name="count"/> pose distribuite uniformemente su una sfera, tutte rivolte al centro.</summary>
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

/// <summary>Genera mappe di profondità per sphere tracing su una SDF.</summary>
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

/// <summary>Scansione simulata: viste distribuite su una sfera attorno all'origine.</summary>
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

- [ ] **Step 4: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SyntheticTests"`
Expected: PASS (5 test).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): tipi DepthFrame e generatore di scansioni sintetiche" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Volume TSDF sparso

**Files:**
- Create: `src/Scanner.Core/Fusion/TsdfVolume.cs`
- Test: `tests/Scanner.Core.Tests/Fusion/TsdfVolumeTests.cs`

**Interfaces:**
- Consumes: `DepthFrame`, `CameraIntrinsics` (Task 2); `CameraPoses.LookAt`, `SyntheticDepthRenderer.Render`, `SphereSdf` (solo nei test).
- Produces: `sealed class TsdfVolume(float voxelSize, float truncationDistance)` con
  `float VoxelSize`, `float TruncationDistance`, `int BlockCount`, `IEnumerable<(int X, int Y, int Z)> AllocatedBlocks` (coordinate di blocco),
  `const int BlockSize = 8`, `Vector3 VoxelToWorld(int x, int y, int z)`,
  `bool TryGet(int x, int y, int z, out float tsdf, out float weight)` (true solo se weight > 0),
  `void Set(int x, int y, int z, float tsdf, float weight)`, `void Integrate(DepthFrame frame)`.

- [ ] **Step 1: Scrivere i test che falliscono**

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
        Assert.True(volume.TryGet(0, 0, -12, out float outside, out _));  // 4 mm fuori dalla sfera
        Assert.Equal(0.667f, outside, 2);
        Assert.True(volume.TryGet(0, 0, -8, out float inside, out _));    // 4 mm dentro la sfera
        Assert.Equal(-0.667f, inside, 2);
        Assert.False(volume.TryGet(0, 0, 0, out _, out _));               // centro: oltre la troncatura
    }
}
```

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~TsdfVolumeTests"`
Expected: FAIL di compilazione — `TsdfVolume` inesistente.

- [ ] **Step 3: Implementare**

`src/Scanner.Core/Fusion/TsdfVolume.cs`:

```csharp
using System.Numerics;
using Scanner.Capture;

namespace Scanner.Core.Fusion;

/// <summary>
/// Volume TSDF sparso: blocchi di 8³ voxel allocati solo vicino alla superficie osservata.
/// Il voxel (x, y, z) ha centro in (x, y, z)·VoxelSize nel sistema mondo.
/// Valori normalizzati in [-1, 1], positivi fuori dall'oggetto.
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
            throw new ArgumentException("Posa della camera non invertibile.", nameof(frame));

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

- [ ] **Step 4: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~TsdfVolumeTests"`
Expected: PASS (2 test).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): volume TSDF sparso a blocchi" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Mesh con Naive Surface Nets

**Files:**
- Create: `src/Scanner.Core/Meshing/TriangleMesh.cs`, `src/Scanner.Core/Meshing/SurfaceNets.cs`
- Test: `tests/Scanner.Core.Tests/Meshing/SurfaceNetsTests.cs`

**Interfaces:**
- Consumes: `TsdfVolume` (Task 3).
- Produces:
  - `sealed class TriangleMesh` con `List<Vector3> Positions`, `List<Vector3> Normals` (per vertice, dal gradiente TSDF, uscenti), `List<int> Indices`, `int TriangleCount`, `Vector3 FaceNormal(int triangle)`, `void AddQuad(int a, int b, int c, int d)`.
  - `static TriangleMesh SurfaceNets.Extract(TsdfVolume volume)` — triangoli con avvolgimento antiorario visto da fuori.

- [ ] **Step 1: Scrivere i test che falliscono**

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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SurfaceNetsTests"`
Expected: FAIL di compilazione — `Scanner.Core.Meshing` inesistente.

- [ ] **Step 3: Implementare**

`src/Scanner.Core/Meshing/TriangleMesh.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Meshing;

public sealed class TriangleMesh
{
    public List<Vector3> Positions { get; } = new();
    /// <summary>Normali per vertice, uscenti dal solido.</summary>
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

    /// <summary>Aggiunge il quad a-b-c-d come due triangoli con lo stesso avvolgimento.</summary>
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
/// Naive Surface Nets: un vertice per cella attraversata dalla superficie (media delle intersezioni
/// sugli spigoli), un quad per ogni spigolo di voxel con cambio di segno.
/// La cella (x, y, z) ha come angoli i voxel da (x, y, z) a (x+1, y+1, z+1);
/// l'angolo i ha offset (i &amp; 1, (i &gt;&gt; 1) &amp; 1, (i &gt;&gt; 2) &amp; 1).
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
            // Ordine delle celle attorno allo spigolo: antiorario visto dalla direzione positiva dell'asse.
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

        // v0 dentro e v1 fuori: la normale uscente ha il verso positivo dell'asse.
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

    // Gradiente dell'interpolazione trilineare al centro della cella: punta verso l'esterno.
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

- [ ] **Step 4: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~SurfaceNetsTests"`
Expected: PASS (2 test).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(core): estrazione mesh con Naive Surface Nets" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Primitive e RANSAC piani/cilindri

**Files:**
- Create: `src/Scanner.Core/Shapes/Primitives.cs`
- Create: `src/Scanner.Core/Segmentation/PointCloud.cs`, `PrimitiveFitter.cs`, `RansacDetector.cs`
- Test: `tests/Scanner.Core.Tests/Segmentation/SampleClouds.cs`, `tests/Scanner.Core.Tests/Segmentation/RansacDetectorTests.cs`

**Interfaces:**
- Consumes: `Basis`, `Linear3`, `SymmetricEigen3` (Task 1); `TriangleMesh` (Task 4).
- Produces:
  - `abstract record Primitive`;
    `sealed record PlanePrimitive(Vector3 Normal, float D) : Primitive` (piano `Normal·x = D`, normale uscente) con `float SignedDistance(Vector3 p)`;
    `sealed record CylinderPrimitive(Vector3 AxisPoint, Vector3 Axis, float Radius, bool IsHole) : Primitive` con `Vector3 RadialVector(Vector3 p)`, `float Distance(Vector3 p)`. `IsHole` = normali verso l'asse.
  - `sealed record PointCloud(Vector3[] Points, Vector3[] Normals)` con `int Count`, `static PointCloud FromMesh(TriangleMesh mesh)`.
  - `sealed record RansacOptions(float DistanceThreshold, float NormalThresholdDegrees, int MinInliers, int IterationsPerShape = 1500, int MaxShapes = 20, float MaxCylinderRadius = 0.5f, int Seed = 1)`.
  - `sealed record DetectedShape(Primitive Primitive, int[] InlierIndices)`.
  - `static IReadOnlyList<DetectedShape> RansacDetector.Detect(PointCloud cloud, RansacOptions options)`.
  - `static Primitive? PrimitiveFitter.Refine(Primitive shape, PointCloud cloud, IReadOnlyList<int> indices)`, `FitPlane(...)`, `FitCylinder(...)`.

- [ ] **Step 1: Scrivere i dati di test e i test che falliscono**

`tests/Scanner.Core.Tests/Segmentation/SampleClouds.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Segmentation;

namespace Scanner.Core.Tests.Segmentation;

/// <summary>Nuvole di punti campionate direttamente su forme note, con normali uscenti esatte.</summary>
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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~RansacDetectorTests"`
Expected: FAIL di compilazione — `Scanner.Core.Segmentation` / `Scanner.Core.Shapes` inesistenti.

- [ ] **Step 3: Implementare le primitive e la nuvola di punti**

`src/Scanner.Core/Shapes/Primitives.cs`:

```csharp
using System.Numerics;

namespace Scanner.Core.Shapes;

public abstract record Primitive;

/// <summary>Piano Normal·x = D, con Normal unitaria uscente dal solido.</summary>
public sealed record PlanePrimitive(Vector3 Normal, float D) : Primitive
{
    public float SignedDistance(Vector3 p) => Vector3.Dot(Normal, p) - D;
}

/// <summary>Cilindro infinito con asse per AxisPoint e direzione Axis (unitaria).
/// IsHole è true quando le normali di superficie puntano verso l'asse (foro).</summary>
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

- [ ] **Step 4: Implementare il fitting ai minimi quadrati**

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

    /// <summary>Piano ai minimi quadrati (autovettore minimo della covarianza), orientato come <paramref name="orientation"/>.</summary>
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

    /// <summary>Asse = direzione ortogonale a tutte le normali; sezione = cerchio di Kåsa sui punti proiettati.</summary>
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

- [ ] **Step 5: Implementare il RANSAC**

`src/Scanner.Core/Segmentation/RansacDetector.cs`:

```csharp
using System.Numerics;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Core.Segmentation;

/// <summary>
/// RANSAC sequenziale: a ogni giro sceglie la primitiva (piano o cilindro) con più inlier fra i punti
/// rimasti, la raffina ai minimi quadrati e ne rimuove gli inlier.
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

    // Due punti con normale: asse = n1 × n2; il centro della sezione è l'intersezione delle normali proiettate.
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

- [ ] **Step 6: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Core.Tests --filter "FullyQualifiedName~RansacDetectorTests"`
Expected: PASS (2 test).

- [ ] **Step 7: Eseguire tutta la suite Core e fare commit**

Run: `dotnet test tests/Scanner.Core.Tests`
Expected: PASS (tutti).

```bash
git add -A
git commit -m "feat(core): primitive e RANSAC piani/cilindri con raffinamento" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Modello B-Rep, validatore e poliedro convesso

**Files:**
- Create: `src/Scanner.Brep/Model/BrepModel.cs`, `src/Scanner.Brep/Model/BrepValidator.cs`
- Create: `src/Scanner.Brep/Builders/ConvexPolyhedronBuilder.cs`
- Test: `tests/Scanner.Brep.Tests/Builders/ConvexPolyhedronBuilderTests.cs`

**Interfaces:**
- Consumes: `PlanePrimitive` (Task 5), `Basis`, `Linear3` (Task 1).
- Produces:
  - `sealed class BrepVertex(Vector3 position)` → `Position`.
  - `abstract record BrepCurve`; `sealed record LineCurve(Vector3 Origin, Vector3 Direction)`; `sealed record CircleCurve(Vector3 Center, Vector3 Axis, Vector3 RefDirection, float Radius)` (antioraria attorno ad `Axis`).
  - `sealed class BrepEdge(BrepVertex start, BrepVertex end, BrepCurve curve)` → `Start`, `End`, `Curve`.
  - `readonly record struct OrientedEdge(BrepEdge Edge, bool SameSense)` → `StartVertex`, `EndVertex`.
  - `sealed class BrepLoop(IReadOnlyList<OrientedEdge> edges)` → `Edges`.
  - `abstract record BrepSurface`; `sealed record PlaneSurface(Vector3 Origin, Vector3 Normal, Vector3 RefDirection)`; `sealed record CylinderSurface(Vector3 Origin, Vector3 Axis, Vector3 RefDirection, float Radius)` (normale lontana dall'asse).
  - `sealed class BrepFace(BrepSurface surface, IReadOnlyList<BrepLoop> loops, bool sameSense)` → `Surface`, `Loops` (per le facce piane `Loops[0]` è il contorno esterno), `SameSense`.
  - `sealed class BrepSolid(IReadOnlyList<BrepFace> faces)` → `Faces`, `IReadOnlyList<BrepEdge> DistinctEdges()`, `IReadOnlyList<BrepVertex> DistinctVertices()`.
  - `static IReadOnlyList<string> BrepValidator.Validate(BrepSolid solid)` — lista vuota se valido.
  - `static BrepSolid ConvexPolyhedronBuilder.Build(IReadOnlyList<PlanePrimitive> planes, float tolerance)` — lancia `InvalidOperationException` se degenere.

- [ ] **Step 1: Scrivere i test che falliscono**

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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: FAIL di compilazione — `Scanner.Brep.Builders` / `Scanner.Brep.Model` inesistenti.

- [ ] **Step 3: Implementare il modello**

`src/Scanner.Brep/Model/BrepModel.cs`:

```csharp
using System.Numerics;

namespace Scanner.Brep.Model;

public sealed class BrepVertex(Vector3 position)
{
    public Vector3 Position { get; } = position;
}

public abstract record BrepCurve;

/// <summary>Retta per Origin con direzione unitaria Direction.</summary>
public sealed record LineCurve(Vector3 Origin, Vector3 Direction) : BrepCurve;

/// <summary>Circonferenza nel piano ortogonale ad Axis, percorsa in senso antiorario attorno ad Axis a partire da RefDirection.</summary>
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

/// <summary>Ciclo chiuso di spigoli orientati; l'interno della faccia sta a sinistra guardando dalla normale della faccia.</summary>
public sealed class BrepLoop(IReadOnlyList<OrientedEdge> edges)
{
    public IReadOnlyList<OrientedEdge> Edges { get; } = edges;
}

public abstract record BrepSurface;

public sealed record PlaneSurface(Vector3 Origin, Vector3 Normal, Vector3 RefDirection) : BrepSurface;

/// <summary>Superficie cilindrica; la sua normale punta lontano dall'asse.</summary>
public sealed record CylinderSurface(Vector3 Origin, Vector3 Axis, Vector3 RefDirection, float Radius) : BrepSurface;

/// <summary>Faccia delimitata da uno o più loop; per le facce piane Loops[0] è il contorno esterno.
/// SameSense = false inverte la normale della superficie.</summary>
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

/// <summary>Controlli topologici minimi per un solido manifold chiuso.</summary>
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
                errors.Add($"Faccia {f}: loop vuoto.");
                continue;
            }
            for (int i = 0; i < loop.Edges.Count; i++)
            {
                var current = loop.Edges[i];
                var next = loop.Edges[(i + 1) % loop.Edges.Count];
                if (current.EndVertex != next.StartVertex)
                    errors.Add($"Faccia {f}: loop non chiuso dopo lo spigolo {i}.");

                uses.TryGetValue(current.Edge, out var count);
                uses[current.Edge] = current.SameSense ? (count.Forward + 1, count.Backward) : (count.Forward, count.Backward + 1);
            }
        }

        foreach (var (_, count) in uses)
            if (count.Forward != 1 || count.Backward != 1)
                errors.Add($"Spigolo usato {count.Forward} volte in avanti e {count.Backward} all'indietro (atteso 1 e 1).");

        int vertices = solid.DistinctVertices().Count;
        int faces = solid.Faces.Count;
        // Eulero-Poincaré: V − E + F − (L − F) = 2(S − G), con S = 1 guscio.
        int chi = vertices - uses.Count + faces - (loopCount - faces);
        if (chi > 2 || chi % 2 != 0)
            errors.Add($"Caratteristica di Eulero-Poincaré non valida: {chi}.");

        return errors;
    }
}
```

- [ ] **Step 4: Implementare il costruttore del poliedro convesso**

`src/Scanner.Brep/Builders/ConvexPolyhedronBuilder.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;
using Scanner.Core.Shapes;

namespace Scanner.Brep.Builders;

/// <summary>Solido convesso come intersezione dei semispazi Normal·x ≤ D.</summary>
public static class ConvexPolyhedronBuilder
{
    private const float DuplicateAngleDegrees = 5f;

    public static BrepSolid Build(IReadOnlyList<PlanePrimitive> planes, float tolerance)
    {
        var unique = MergeDuplicates(planes, tolerance);
        if (unique.Count < 4) throw new InvalidOperationException($"Servono almeno 4 piani distinti, trovati {unique.Count}.");

        var points = IntersectionVertices(unique, tolerance);
        if (points.Count < 4) throw new InvalidOperationException("Poliedro degenere: meno di 4 vertici.");

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
            // Angolo crescente nella base (u, v) con u × v = normale: senso antiorario visto da fuori.
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

Nota: la soglia di merge in `D` è `3·tolerance + 1 mm` perché due piani RANSAC della stessa faccia possono differire di circa una soglia di distanza; `tolerance` (0,1 mm) serve invece per i vertici.

- [ ] **Step 5: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: PASS (4 test).

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat(brep): modello B-Rep, validatore topologico e poliedro convesso" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Costruttore del tubo (cilindro con foro coassiale)

**Files:**
- Create: `src/Scanner.Brep/Builders/TubeBuilder.cs`
- Test: `tests/Scanner.Brep.Tests/Builders/TubeBuilderTests.cs`

**Interfaces:**
- Consumes: modello B-Rep e `BrepValidator` (Task 6), `Basis` (Task 1).
- Produces: `static BrepSolid TubeBuilder.Build(Vector3 axisPoint, Vector3 axis, float outerRadius, float? innerRadius, float zBottom, float zTop)` — `z` misurate lungo `axis` a partire da `axisPoint`; facce in ordine: esterno, [foro], tappo inferiore, tappo superiore.

- [ ] **Step 1: Scrivere i test che falliscono**

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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~TubeBuilderTests"`
Expected: FAIL di compilazione — `TubeBuilder` inesistente.

- [ ] **Step 3: Implementare**

`src/Scanner.Brep/Builders/TubeBuilder.cs`:

```csharp
using System.Numerics;
using Scanner.Brep.Model;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Brep.Builders;

/// <summary>
/// Cilindro esterno con foro coassiale opzionale e due tappi piani ortogonali all'asse.
/// Ogni circonferenza è un unico spigolo chiuso (vertice iniziale = finale), percorso antiorario attorno all'asse.
/// Orientamenti (interno della faccia a sinistra guardando dalla normale della faccia):
/// esterno: basso +, alto −; foro (SameSense=false): basso −, alto +;
/// tappo inferiore (normale −asse): esterno −, foro +; tappo superiore (normale +asse): esterno +, foro −.
/// </summary>
public static class TubeBuilder
{
    public static BrepSolid Build(Vector3 axisPoint, Vector3 axis, float outerRadius, float? innerRadius, float zBottom, float zTop)
    {
        if (zTop <= zBottom) throw new ArgumentException("zTop deve essere maggiore di zBottom.");
        if (innerRadius is { } r && (r <= 0 || r >= outerRadius)) throw new ArgumentException("Raggio del foro non valido.");

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

- [ ] **Step 4: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~TubeBuilderTests"`
Expected: PASS (3 test).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(brep): costruttore tubo/cilindro coassiale" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Writer STEP AP214

**Files:**
- Create: `src/Scanner.Brep/Step/StepWriter.cs`
- Test: `tests/Scanner.Brep.Tests/Step/StepSyntaxChecker.cs`, `tests/Scanner.Brep.Tests/Step/StepWriterTests.cs`

**Interfaces:**
- Consumes: modello B-Rep (Task 6), `ConvexPolyhedronBuilder` (Task 6), `TubeBuilder` (Task 7).
- Produces: `static string StepWriter.Write(BrepSolid solid, string productName, DateTime timestampUtc)` — file ISO 10303-21 completo, coordinate in mm.

- [ ] **Step 1: Scrivere il controllore di sintassi e i test che falliscono**

`tests/Scanner.Brep.Tests/Step/StepSyntaxChecker.cs`:

```csharp
using System.Text.RegularExpressions;

namespace Scanner.Brep.Tests.Step;

/// <summary>Controlli strutturali su un file STEP: header, id univoci, riferimenti risolti, righe terminate.</summary>
internal static class StepSyntaxChecker
{
    public static IReadOnlyList<string> Check(string step)
    {
        var errors = new List<string>();
        var text = step.Trim();
        if (!text.StartsWith("ISO-10303-21;")) errors.Add("Manca l'intestazione ISO-10303-21.");
        if (!text.EndsWith("END-ISO-10303-21;")) errors.Add("Manca la chiusura END-ISO-10303-21.");
        if (!text.Contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));")) errors.Add("Schema AP214 mancante.");

        var defined = new HashSet<int>();
        foreach (Match m in Regex.Matches(step, @"^#(\d+)=", RegexOptions.Multiline))
            if (!defined.Add(int.Parse(m.Groups[1].Value))) errors.Add($"Id duplicato #{m.Groups[1].Value}.");

        foreach (Match m in Regex.Matches(step, @"#(\d+)"))
            if (!defined.Contains(int.Parse(m.Groups[1].Value))) errors.Add($"Riferimento non definito #{m.Groups[1].Value}.");

        foreach (var line in step.Split('\n').Where(l => l.StartsWith('#')))
        {
            if (!line.TrimEnd().EndsWith(';')) errors.Add($"Riga non terminata: {line}");
            if (line.Count(c => c == '(') != line.Count(c => c == ')')) errors.Add($"Parentesi sbilanciate: {line}");
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

        var step = StepWriter.Write(solid, "l'anello", Timestamp);

        Assert.Contains("PRODUCT('l''anello','l''anello'", step);
    }
}
```

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~StepWriterTests"`
Expected: FAIL di compilazione — `Scanner.Brep.Step` inesistente.

- [ ] **Step 3: Implementare**

`src/Scanner.Brep/Step/StepWriter.cs`:

```csharp
using System.Globalization;
using System.Numerics;
using System.Text;
using Scanner.Brep.Model;

namespace Scanner.Brep.Step;

/// <summary>Scrive un <see cref="BrepSolid"/> come file STEP AP214 (ISO 10303-21) in millimetri.</summary>
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
        _ => throw new NotSupportedException($"Curva non supportata: {curve.GetType().Name}"),
    };

    private int WriteSurface(BrepSurface surface) => surface switch
    {
        PlaneSurface plane => Add($"PLANE('',#{WriteAxis(plane.Origin, plane.Normal, plane.RefDirection)})"),
        CylinderSurface cylinder => Add($"CYLINDRICAL_SURFACE('',#{WriteAxis(cylinder.Origin, cylinder.Axis, cylinder.RefDirection)},{Length(cylinder.Radius)})"),
        _ => throw new NotSupportedException($"Superficie non supportata: {surface.GetType().Name}"),
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

    // Arrotondamento a 1e-6 (nm per le lunghezze) per evitare rumore float come 19.9999995.
    private static string Real(double value) =>
        Math.Round(value, 6).ToString("0.0#####", CultureInfo.InvariantCulture);

    private static string Escape(string text) => text.Replace("'", "''");
}
```

Nota: `Length(float)` moltiplica in `double` (`meters * 1000.0`) — `0.02f` diventa `19.99999955…`, arrotondato a `20.0`.

- [ ] **Step 4: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Brep.Tests`
Expected: PASS (tutti, 11 test).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(brep): writer STEP AP214 in millimetri" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 9: Ricostruzione meccanica e pipeline completa scansione → STEP

**Files:**
- Create: `src/Scanner.Core/ProcessingProfile.cs`
- Create: `src/Scanner.Brep/Reconstruction/MechanicalReconstructor.cs`, `src/Scanner.Brep/Reconstruction/ScanToStep.cs`
- Test: `tests/Scanner.Brep.Tests/Reconstruction/MechanicalReconstructorTests.cs`, `tests/Scanner.Brep.Tests/Reconstruction/ScanToStepTests.cs`

**Interfaces:**
- Consumes: `TsdfVolume` (T3), `SurfaceNets` (T4), `PointCloud`/`RansacOptions`/`RansacDetector`/`DetectedShape`/primitive (T5), builder e validatore (T6, T7), `StepWriter` (T8), `SyntheticScan`/`BoxSdf`/`TubeSdf` (T2, nei test).
- Produces:
  - `sealed record ProcessingProfile(float VoxelSize, float TruncationVoxels, float RansacDistance, float NormalThresholdDegrees, float MinInlierFraction, int IterationsPerShape, int Seed)` con `static ProcessingProfile Draft` e `static ProcessingProfile Fine`.
  - `sealed record ReconstructionOptions(float VertexTolerance = 1e-4f, float CoaxialTolerance = 0.002f, float AngleToleranceDegrees = 3f)`.
  - `sealed record ReconstructionResult(BrepSolid? Solid, string? FailureReason)`.
  - `static ReconstructionResult MechanicalReconstructor.Reconstruct(IReadOnlyList<DetectedShape> shapes, ReconstructionOptions options)`.
  - `sealed record ScanResult(TriangleMesh Mesh, IReadOnlyList<DetectedShape> Shapes, BrepSolid? Solid, string? Step, string? FailureReason)` con `bool Succeeded`.
  - `static ScanResult ScanToStep.Run(IEnumerable<DepthFrame> frames, ProcessingProfile profile, string productName, CancellationToken cancellationToken = default)`.

- [ ] **Step 1: Scrivere i test del ricostruttore (falliscono)**

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
        Assert.Contains("coassiale", result.FailureReason!);
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

- [ ] **Step 2: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~MechanicalReconstructorTests"`
Expected: FAIL di compilazione — `Scanner.Brep.Reconstruction` inesistente.

- [ ] **Step 3: Implementare profilo e ricostruttore**

`src/Scanner.Core/ProcessingProfile.cs`:

```csharp
namespace Scanner.Core;

/// <summary>Parametri della pipeline. Bozza per il telefono, Fine per il desktop.</summary>
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
/// Modalità Meccanico (v0): solo piani → poliedro convesso; un cilindro esterno + foro coassiale opzionale
/// + due tappi ortogonali → tubo. Ogni altro caso è riportato come non supportato, mai come eccezione.
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
                : new ReconstructionResult(null, "B-Rep non valido: " + string.Join("; ", errors));
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
                $"Supportati solo un cilindro esterno e al più un foro (trovati {outer.Count} esterni e {holes.Count} fori).");

        var main = outer[0];
        var axis = main.Axis;
        var hole = holes.SingleOrDefault();
        if (hole is not null)
        {
            bool parallel = MathF.Abs(Vector3.Dot(hole.Axis, axis)) >= cosTolerance;
            float offset = main.RadialVector(hole.AxisPoint).Length();
            if (!parallel || offset > options.CoaxialTolerance)
                throw new NotSupportedException("Il foro non è coassiale al cilindro esterno.");
        }

        var caps = planes.Where(p => MathF.Abs(Vector3.Dot(p.Normal, axis)) >= cosTolerance).ToList();
        if (caps.Count != 2 || planes.Count != 2)
            throw new NotSupportedException(
                $"Servono esattamente due piani ortogonali all'asse (trovati {caps.Count} su {planes.Count}).");

        // Punto dell'asse AxisPoint + h·axis sul piano n·x = d  ⇒  h = (d − n·AxisPoint) / (n·axis)
        var heights = caps
            .Select(p => (p.D - Vector3.Dot(p.Normal, main.AxisPoint)) / Vector3.Dot(p.Normal, axis))
            .OrderBy(h => h)
            .ToArray();
        return TubeBuilder.Build(main.AxisPoint, axis, main.Radius, hole?.Radius, heights[0], heights[1]);
    }
}
```

- [ ] **Step 4: Eseguire i test del ricostruttore e verificare che passino**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~MechanicalReconstructorTests"`
Expected: PASS (4 test).

- [ ] **Step 5: Scrivere i test end-to-end (falliscono)**

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

- [ ] **Step 6: Eseguire i test e verificare che falliscano**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~ScanToStepTests"`
Expected: FAIL di compilazione — `ScanToStep` inesistente.

- [ ] **Step 7: Implementare la pipeline**

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

/// <summary>Pipeline completa modalità Meccanico: fusione → mesh → RANSAC → B-Rep → STEP.</summary>
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

- [ ] **Step 8: Eseguire i test e verificare che passino**

Run: `dotnet test tests/Scanner.Brep.Tests --filter "FullyQualifiedName~ScanToStepTests"`
Expected: PASS (3 test).

Se un test end-to-end fallisce, il messaggio contiene `FailureReason`. Diagnosi tipiche:
- "Supportati solo un cilindro esterno…" o "Servono esattamente due piani…" sul cubo/tubo → RANSAC ha trovato forme spurie sugli spigoli arrotondati: alzare `MinInlierFraction` in `ProcessingProfile.Fine` a `0.05f`.
- Meno di 6 piani sul cubo → abbassare `MinInlierFraction` a `0.02f` o aumentare `IterationsPerShape` a `3000`.
Verificare prima stampando `result.Shapes` (tipo, parametri, numero di inlier) in un test temporaneo; non modificare le soglie senza aver visto quale forma è sbagliata.

- [ ] **Step 9: Eseguire tutte le suite e fare commit**

Run: `dotnet test Scan3D.slnx`
Expected: PASS (tutti i test di Core e Brep).

```bash
git add -A
git commit -m "feat(brep): ricostruzione meccanica e pipeline scansione → STEP" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 10: CLI per i file di esempio + checklist Fusion

**Files:**
- Create: `tools/Scanner.Cli/Scanner.Cli.csproj` (via CLI), `tools/Scanner.Cli/Program.cs`
- Create: `docs/fusion-checklist.md`
- Modify: `.gitignore` (aggiungere `out/`)

**Interfaces:**
- Consumes: `SyntheticScan`, `BoxSdf`, `TubeSdf` (T2), `ProcessingProfile` (T9), `ScanToStep.Run` (T9).
- Produces: comando `dotnet run --project tools/Scanner.Cli -- <cube|tube> <file.stp>`; exit code 0 se lo STEP è scritto, 1 per uso errato, 2 se la ricostruzione fallisce.

- [ ] **Step 1: Creare il progetto**

```bash
dotnet new console -n Scanner.Cli -o tools/Scanner.Cli -f net10.0
dotnet sln add tools/Scanner.Cli
dotnet add tools/Scanner.Cli reference src/Scanner.Brep
echo "out/" >> .gitignore
```

- [ ] **Step 2: Scrivere `Program.cs`**

`tools/Scanner.Cli/Program.cs` (sostituisce il contenuto generato):

```csharp
using System.Diagnostics;
using System.Numerics;
using Scanner.Brep.Reconstruction;
using Scanner.Core;
using Scanner.Core.Shapes;
using Scanner.Core.Synthetic;

if (args.Length != 2 || args[0] is not ("cube" or "tube"))
{
    Console.Error.WriteLine("Uso: Scanner.Cli <cube|tube> <file.stp>");
    return 1;
}

ISdf shape = args[0] == "cube"
    ? new BoxSdf(Vector3.Zero, new Vector3(0.02f))
    : new TubeSdf(Vector3.Zero, 0.02f, 0.01f, 0.03f);

var stopwatch = Stopwatch.StartNew();
var frames = SyntheticScan.Capture(shape, 40, 0.12f, SyntheticScan.DefaultIntrinsics, 0.0003f, seed: 7);
var result = ScanToStep.Run(frames, ProcessingProfile.Fine, Path.GetFileNameWithoutExtension(args[1]));

Console.WriteLine($"Mesh: {result.Mesh.Positions.Count} vertici, {result.Mesh.TriangleCount} triangoli");
foreach (var detected in result.Shapes)
{
    string description = detected.Primitive switch
    {
        PlanePrimitive p => $"piano    n=({p.Normal.X:F3},{p.Normal.Y:F3},{p.Normal.Z:F3}) d={p.D * 1000:F2} mm",
        CylinderPrimitive c => $"cilindro r={c.Radius * 1000:F2} mm asse=({c.Axis.X:F3},{c.Axis.Y:F3},{c.Axis.Z:F3}) foro={c.IsHole}",
        _ => detected.Primitive.GetType().Name,
    };
    Console.WriteLine($"  {description}  inlier={detected.InlierIndices.Length}");
}

if (!result.Succeeded)
{
    Console.Error.WriteLine($"Ricostruzione fallita: {result.FailureReason}");
    return 2;
}

Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(args[1]))!);
File.WriteAllText(args[1], result.Step);
Console.WriteLine($"Scritto {args[1]} ({result.Solid!.Faces.Count} facce) in {stopwatch.Elapsed.TotalSeconds:F1} s");
return 0;
```

- [ ] **Step 3: Eseguire il CLI e verificare l'output**

Run: `dotnet run --project tools/Scanner.Cli -- cube out/cube.stp`
Expected: 6 righe `piano` con `d≈20.00 mm`, poi `Scritto out/cube.stp (6 facce)`; exit code 0.

Run: `dotnet run --project tools/Scanner.Cli -- tube out/tube.stp`
Expected: due righe `cilindro` (r≈20 mm foro=False, r≈10 mm foro=True), due `piano` con `d≈15.00 mm`, poi `Scritto out/tube.stp (4 facce)`; exit code 0.

Run: `dotnet run --project tools/Scanner.Cli -- sphere out/x.stp`
Expected: messaggio d'uso, exit code 1.

- [ ] **Step 4: Scrivere la checklist Fusion**

`docs/fusion-checklist.md`:

```markdown
# Verifica manuale in Autodesk Fusion

Generare i file:

    dotnet run --project tools/Scanner.Cli -- cube out/cube.stp
    dotnet run --project tools/Scanner.Cli -- tube out/tube.stp

Per ciascun file: Fusion → File → Apri → "Apri dal mio computer…" → selezionare il `.stp`.

## cube.stp
- [ ] Si importa senza avvisi di riparazione.
- [ ] Nel Browser compare sotto **Corpi** (non "Corpi mesh").
- [ ] 6 facce piane; Inspect → Measure tra facce opposte ≈ 40 mm (±1 mm).
- [ ] Modify → Press Pull su una faccia funziona.
- [ ] Modify → Fillet su uno spigolo (2 mm) funziona.

## tube.stp
- [ ] Si importa senza avvisi di riparazione ed è un **corpo solido**.
- [ ] Diametro esterno ≈ 40 mm, foro ≈ 20 mm, altezza ≈ 30 mm (Inspect → Measure).
- [ ] Selezionando il foro Fusion lo riconosce come faccia cilindrica (mostra il diametro).
- [ ] Modify → Chamfer sullo spigolo circolare superiore funziona.

Annotare versione di Fusion, data ed eventuali problemi in fondo a questo file.
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(cli): generatore STEP di esempio e checklist di verifica in Fusion" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 6: Verifica manuale (utente)**

Chiedere all'utente di aprire `out/cube.stp` e `out/tube.stp` in Fusion seguendo `docs/fusion-checklist.md` e riportare l'esito. Il traguardo è chiuso solo quando entrambi si aprono come corpi solidi modificabili.
