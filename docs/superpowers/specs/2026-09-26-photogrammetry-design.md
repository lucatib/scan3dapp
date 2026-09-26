# Photogrammetry — Design

Date: 2026-09-26
Status: accepted (the user chose every option below and asked for implementation without further review)
Supersedes: "Pure photogrammetry (without depth)" in the out-of-scope list of
[2026-09-19-scan3d-design.md](2026-09-19-scan3d-design.md).

## Why

Device tests on a Galaxy Note20 Ultra (2026-09-26) showed ARCore depth is not usable as the geometry source for
CAD: raw depth is 160×90, and from frame to frame the same table moved with a standard deviation of 46 mm and
tilted ~7°. Neither intrinsics nor a global depth calibration fixed it. ARCore's tracking is still valuable as a
metric starting pose for every photo.

## Decisions (made by the user)

1. ARCore keeps running for tracking; **photos become the main geometry source**.
2. The phone shows a **rough 3D shape after Finish**, built by quick photogrammetry on the phone.
3. The **PC does the full reconstruction** afterwards.
4. Photos are taken **inside ARCore at 1080p** (its largest CPU image), automatically while walking.
5. **Everything is our own C#**: dense stereo *and* camera-position refinement (bundle adjustment). No external tools.
6. **Photo points and ARCore depth points are merged** into one voxel cloud.

## Architecture

```
Scanner.Capture   GrayImage (8-bit luma), photo previews kept in memory by LiveScanSession
Scanner.Core      Photogrammetry/  — all platform-independent, same code on phone and PC:
                    PhotoView            image + intrinsics + camera→world (OpenCV axes, metres)
                    TexturedScene        synthetic textured renderer for tests
                    PlaneSweepStereo     depth map per reference view (NCC, fronto-parallel planes)
                    DepthMapFusion       multi-view consistency filter → points
                    SourceMerge          photo points first; ARCore depth only fills gaps
                    Corners / Matcher /  Shi-Tomasi corners, pose-guided NCC matching, tracks,
                    BundleAdjuster       triangulation, Levenberg-Marquardt with Schur complement
Scanner.App       1080p camera config; keeps a 480-px-wide luma copy of each photo;
                  on Finish runs the quick preview (no bundle adjustment) and shows the merged cloud
Scanner.Desktop   console tool (Windows): loads a session, decodes photos, refines poses,
                  full-resolution stereo, merge, isolate, writes PLY + a report
```

## Algorithms

**Plane-sweep stereo.** For a reference view, K neighbour views are chosen by the angle their rays make at the
target (3°–35°, best near 15°) and by distance. Depth hypotheses are sampled uniformly in inverse depth over
`[distance to target − 0.35 m, + 0.35 m]`. For each hypothesis every neighbour is warped into the reference with
the plane homography `H = K_n (R − t nᵀ / d) K_r⁻¹` (bilinear), and a windowed NCC is computed with integral
images, so a hypothesis costs O(pixels) regardless of the window. The cost aggregates the best two neighbours
(occlusion-robust). The best depth is refined with a parabola; pixels whose reference patch has too little
texture, or whose best score is weak, are left empty.

**Fusion.** Each depth map is back-projected; a point is kept when at least two other depth maps agree with it
(relative depth difference ≤ 1 %). Points go into a voxel accumulator (the existing 5 mm grid on the phone).

**Merge.** Photo points are accumulated first. An ARCore depth point is added only when no photo point lies
within 1 cm — ARCore fills plain areas photos cannot match, and never blurs the precise photo points.

**Bundle adjustment (PC).** Shi-Tomasi corners, bucketed over the image. Matches between neighbour pairs are
searched near the epipolar line predicted by the ARCore poses and scored by NCC (mutual best, ratio test).
Matches are chained into tracks, triangulated, and refined with Levenberg-Marquardt: 6 parameters per camera,
3 per point, Huber loss on reprojection error, plus a prior tying each camera centre to its ARCore position
(σ 2 cm) — which fixes the gauge and keeps ARCore's metric scale. The reduced camera system is dense and small.

## Phone preview budget

Up to 12 reference views of 480×270 luma, 4 neighbours each, 96 depth hypotheses, ARCore poses as they are.
Target: tens of seconds on the phone, with a status line while it runs.

## Testing

Synthetic textured scenes rendered from known cameras drive the unit tests: stereo depth error against ground
truth, fusion keeping only consistent points, the merge rule, matching, and bundle adjustment recovering
perturbed poses. Real sessions from the device are run through `Scanner.Desktop` as the end-to-end check.

## Milestones

1. Capture: 1080p camera config, photos every 0.5 s (up to 90), in-memory luma previews.
2. Core stereo: GrayImage, textured synthetic renderer, plane sweep, fusion.
3. Merge of photo and ARCore points.
4. Phone preview after Finish.
5. Bundle adjustment.
6. Desktop tool, run on real sessions.
