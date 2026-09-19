# Scan3D — Specifica di design

Data: 2026-09-19
Stato: bozza da revisionare

## 1. Obiettivo

App .NET MAUI (net10) che trasforma uno smartphone in uno scanner 3D e produce
**file STEP (B-Rep) importabili e modificabili in Autodesk Fusion**, non solo
nuvole di punti o mesh. La stessa app gira su desktop (Windows, macOS via Mac
Catalyst) per rielaborare le scansioni ad alta risoluzione.

### In perimetro (v1)
- Acquisizione su iOS (ARKit, LiDAR / scene depth) e Android (ARCore Depth API).
- Import su desktop di sessioni `.scan` e di mesh generiche (PLY/OBJ/STL).
- Elaborazione **interamente in locale**, stesso codice su tutte le piattaforme,
  con profili di risoluzione diversi (mobile = bozza, desktop = fine).
- Tre modalità selezionabili dall'utente: **Meccanico**, **Organico**, **Ambiente**.
- Export: STEP AP214, STL, OBJ, PLY, `.scan`.
- Visualizzazione 3D con Silk.NET.

### Fuori perimetro (v1)
- Gadget esterni (ToF VL53Lx via BLE, piatto rotante con stepper). Restano
  possibili tramite le interfacce `IDepthSource` / `IPoseSource`.
- Elaborazione su server/cloud.
- Backend OpenCASCADE (previsto in futuro, solo desktop, dietro `IBrepBuilder`).
- Texture/colore sul modello STEP.
- Fotogrammetria pura (senza profondità).

## 2. Architettura

```
Scanner.App (MAUI, net10.0-android/ios/maccatalyst/windows)
 ├─ UI MVVM (CommunityToolkit.Mvvm)
 ├─ Platforms/iOS      → ArKitDepthSource, ArKitPoseSource
 ├─ Platforms/Android  → ArCoreDepthSource, ArCorePoseSource
 └─ Viewer             → handler MAUI che ospita il renderer Silk.NET

Scanner.Capture   (net10.0)  interfacce IDepthSource, IPoseSource, ICameraSource,
                             modello DepthFrame, formato sessione .scan
Scanner.Core      (net10.0)  geometria, TSDF, marching cubes, pulizia mesh,
                             segmentazione, RANSAC, fitting primitive e B-spline
Scanner.Brep      (net10.0)  topologia B-Rep, IBrepBuilder, CSharpBrepBuilder,
                             StepWriter AP214, export STL/OBJ/PLY
Scanner.Render    (net10.0)  IRenderer + implementazione Silk.NET (GLES 3.0)
Scanner.*.Tests   (xUnit)    test su Core, Brep, Capture (serializzazione)
```

Regole:
- `Scanner.Core` e `Scanner.Brep` non hanno **alcuna** dipendenza da piattaforma
  o da MAUI e sono testabili su PC.
- Le implementazioni native vivono solo in `Scanner.App/Platforms/*`.
- Il calcolo usa `System.Numerics` (SIMD) e `Parallel`; niente GPU compute in v1.

## 3. Acquisizione

- `IDepthSource` produce `DepthFrame`: mappa di profondità (float32, metri),
  confidenza opzionale, intrinseci (fx, fy, cx, cy), timestamp.
- `IPoseSource` produce la posa camera→mondo (matrice 4x4) per timestamp.
- iOS: `ARFrame.sceneDepth` (LiDAR, 256×192) o `smoothedSceneDepth`; posa da
  `ARCamera.transform`.
- Android: ARCore `acquireDepthImage16Bits` + confidence; posa da `Camera.getPose`.
- Frequenza di integrazione 5–10 Hz, frame scartati se il tracking non è `Normal`.
- Il dispositivo senza supporto profondità mostra un messaggio e permette solo
  la modalità import.

### Formato sessione `.scan`
Archivio zip:
- `manifest.json` — versione formato, dispositivo, modalità, intrinseci, unità.
- `frames/NNNNNN.depth` — profondità float16 compressa + confidenza.
- `frames/NNNNNN.pose` — matrice 4x4 float32.
- `keyframes/NNNNNN.jpg` — foto opzionali (per usi futuri).

Serve a rielaborare su desktop ad alta risoluzione e come dato di test.

## 4. Pipeline di elaborazione

Fasi comuni:
1. **Fusione TSDF** su griglia voxel sparsa (blocchi 8³ in hash map).
   Voxel: mobile 4–8 mm, desktop 1–2 mm (parametri per profilo).
2. **Marching Cubes** → mesh triangolare.
3. **Pulizia**: rimozione componenti piccole, smoothing leggero (Taubin),
   ricalcolo normali, decimazione opzionale.
4. **Ritaglio**: box 3D dell'utente; rimozione automatica del piano d'appoggio
   (RANSAC sul piano dominante sotto l'oggetto).

Fasi per modalità:

| Modalità | Algoritmo | Output STEP |
|---|---|---|
| Meccanico | Region growing su normali + RANSAC efficiente (piano, cilindro, cono, sfera), raffinamento ai minimi quadrati; spigoli e vertici da intersezione di primitive adiacenti; snap opzionale (parallelismo, perpendicolarità, coassialità, raggi standard) | Solido chiuso con superfici analitiche |
| Organico | Partizione in patch quadrangolari, fitting B-spline per patch, continuità C0 (G1 approssimata) ai bordi | Shell di `B_SPLINE_SURFACE_WITH_KNOTS` |
| Ambiente | RANSAC solo piani; pavimento, soffitto, pareti verticali; pianta 2D estrusa; aperture rettangolari | Solido pareti/pavimento o set di superfici |

**Ripiego**: se la topologia non si chiude, si esportano comunque STL/OBJ e uno
STEP "a faccette" (facce triangolari planari). L'app mostra una metrica di
qualità: % area coperta da primitive e RMS dell'errore in mm.

**Limiti noti**: raccordi e smussi sotto ~3× la dimensione voxel diventano
spigoli vivi; continuità tra patch organiche solo approssimata.

## 5. B-Rep ed export STEP

- `IBrepBuilder` riceve primitive e adiacenze, restituisce un modello B-Rep
  (vertici, spigoli, loop, facce, shell, solido).
- `CSharpBrepBuilder` (v1): calcola gli spigoli per intersezione analitica
  (piano-piano, piano-cilindro, piano-cono, piano-sfera, cilindro-cilindro
  coassiale); le intersezioni non gestite degradano a spigoli polilinea/B-spline
  approssimati.
- `StepWriter`: scrive ISO 10303-21, schema AP214 (`AUTOMOTIVE_DESIGN`), unità mm.
  Entità: `MANIFOLD_SOLID_BREP`, `CLOSED_SHELL`, `ADVANCED_FACE`, `PLANE`,
  `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`,
  `B_SPLINE_SURFACE_WITH_KNOTS`, `EDGE_CURVE`, `LINE`, `CIRCLE`,
  `B_SPLINE_CURVE_WITH_KNOTS`, e le entità di contesto prodotto richieste.
- Validazione interna prima dell'export: ogni spigolo condiviso da esattamente
  due facce, caratteristica di Eulero coerente, loop chiusi e orientati.
- Futuro: `OcctBrepBuilder` (OpenCASCADE) solo su desktop, stessa interfaccia.

## 6. Rendering (Silk.NET)

- `IRenderer`: carica mesh (posizioni, normali, colori per vertice, indici),
  camera orbitale, selezione (ray picking), linee di spigoli, gizmo del box di
  ritaglio, overlay trasparente.
- Implementazione: **OpenGL ES 3.0 via Silk.NET**.
  - Windows: ANGLE (D3D11) o WGL.
  - Android: EGL nativo.
  - iOS / Mac Catalyst: **ANGLE con backend Metal**.
- Ospitato in un handler MAUI per piattaforma che fornisce la superficie nativa.
- **Rischio principale**: disponibilità e packaging di ANGLE su iOS/Catalyst.
  Mitigazione: primo compito del piano è uno spike (triangolo → mesh sulle 4
  piattaforme). Se fallisce su Apple, si passa a **Silk.NET.WebGPU**
  (wgpu-native) dietro la stessa `IRenderer`.
- Durante l'acquisizione: preview camera nativa (ARKit/ARCore) con overlay della
  mesh TSDF disegnata dal renderer Silk.NET su layer trasparente.

## 7. UI e flusso

1. **Progetti**: elenco scansioni (miniatura, modalità, data); su desktop "Importa".
2. **Nuova scansione**: modalità + qualità (Bozza / Fine).
3. **Acquisizione** (mobile): preview + mesh live colorata per copertura;
   Start / Pausa / Fine; avvisi (troppo veloce, troppo lontano, tracking perso).
4. **Ritaglio**: box 3D, "rimuovi piano d'appoggio".
5. **Elaborazione**: progresso per fase, annullabile, in background.
6. **Risultato**: vista 3D con primitive colorate per tipo, heatmap errore,
   statistiche; parametri (tolleranza RANSAC, snap, raggio minimo) con
   "rilancia fitting".
7. **Export**: STEP / STL / OBJ / PLY / `.scan` via share sheet o "Salva con nome".

## 8. Gestione errori

- Tracking perso o profondità non valida: frame scartato, avviso in UI.
- Memoria: limite di voxel per profilo; oltre soglia si aumenta la dimensione
  voxel e si avvisa l'utente.
- Fallimento fitting/B-Rep: si passa al ripiego (mesh + STEP a faccette) con
  motivazione mostrata all'utente, mai un crash.
- Elaborazione annullabile tramite `CancellationToken` in tutte le fasi.

## 9. Test

- **Sintetici** (`Scanner.Core.Tests`): generatore di oggetti noti (cubo,
  cilindro forato, flangia, sfera, stanza) e simulatore di frame di profondità
  da pose note con rumore e buchi. Verifica che fusione e fitting ritrovino
  le primitive entro tolleranza.
- **STEP** (`Scanner.Brep.Tests`): file sintatticamente valido (parser interno),
  topologia chiusa, golden file per casi semplici.
- **Regressione reale**: scansioni `.scan` reali in `testdata/` (Git LFS).
- **Manuale**: checklist di import in Fusion su un set fisso di esempi;
  prove su dispositivo per acquisizione e UI.

## 10. Milestone

1. Spike renderer Silk.NET su Windows, Android, iOS, Mac Catalyst.
2. Core su dati sintetici: TSDF → mesh → RANSAC → B-Rep → STEP di cubo e
   cilindro forato, verificato in Fusion.
3. Formato `.scan` + import desktop + viewer risultati.
4. Acquisizione iOS (ARKit).
5. Acquisizione Android (ARCore).
6. Modalità Ambiente.
7. Modalità Organico (B-spline).
8. Rifiniture UI, export, profili di qualità.
