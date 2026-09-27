using System.Numerics;
using Scanner.Capture;
using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Photogrammetry;

/// <param name="MaxIterations">Levenberg-Marquardt iterations, rejected steps included, so the work is bounded.</param>
/// <param name="HuberPx">Reprojection error, in pixels, beyond which an observation's cost grows linearly instead of
/// quadratically: a wrong match then pulls with a bounded force rather than one that grows with its error.
/// <see cref="float.PositiveInfinity"/> gives plain least squares.</param>
/// <param name="CenterPriorSigma">How far, in metres, each camera centre is expected to lie from where it started
/// (ARCore's tracking error). Reprojections alone leave the scene free to move, turn and scale; these priors pin it
/// to ARCore's frame, keeping its metric scale and its gravity direction.</param>
/// <param name="InitialLambda">Starting Levenberg-Marquardt damping, relative to the diagonal of the normal
/// equations.</param>
public sealed record BundleOptions(int MaxIterations = 30, float HuberPx = 2f, float CenterPriorSigma = 0.02f,
    double InitialLambda = 1e-3);

/// <param name="CameraToWorld">The refined pose of every view, in the pipeline convention (OpenCV axes, row vectors,
/// orthonormal rotation). A view no usable observation reaches keeps its pose.</param>
/// <param name="Points">The refined point of every track. A track seen by fewer than two photos keeps its point.</param>
/// <param name="InitialRmsPx">Root mean square reprojection error, in pixels, before adjustment: plain, not robust,
/// over the observations the adjustment used.</param>
/// <param name="FinalRmsPx">The same after adjustment. Gross outliers keep it high even when the geometry is right.</param>
/// <param name="Iterations">Levenberg-Marquardt iterations run, rejected steps included.</param>
/// <param name="Converged">True when, before the iteration limit, an accepted step lowered the cost by less than a
/// millionth of it, or the step itself became too small to change anything.</param>
public sealed record BundleResult(Matrix4x4[] CameraToWorld, Vector3[] Points, double InitialRmsPx, double FinalRmsPx,
    int Iterations, bool Converged);

/// <summary>
/// Bundle adjustment: refines camera poses and track points together so that the points reproject onto their
/// observations. ARCore places each photo to within 1-2 cm and about a degree; plane-sweep stereo needs the poses
/// consistent to about a millimetre, and the reprojections of hundreds of matched corners hold that information.
/// </summary>
/// <remarks>
/// Each camera has six parameters: a small rotation w applied on the left of its world→camera rotation
/// (R ← exp([w]×)·R, a turn about the camera's own axes) and its centre C. Each point has three. Intrinsics are
/// fixed. The cost is Σ ρ(|reprojection error| in px) + Σ |C − C_initial|² / σ², with ρ the Huber loss, minimized by
/// Levenberg-Marquardt on the normal equations with Marquardt's diagonal scaling, so the damping means the same for
/// radians, metres and pixels. The point blocks are 3×3 and independent, so they are eliminated (Schur complement)
/// and only the reduced camera system, 6N×6N and dense, is factored; the points follow by back-substitution.
/// After every accepted step the whole scene is also moved by the similarity that best fits the centres to their
/// priors, the one part of the problem the reprojections cannot see and damped steps crawl along.
/// Observations of a point behind or almost on its camera's plane at the start are left out, as are tracks that
/// keep fewer than two photos: a single view says nothing about a free point.
/// </remarks>
public static class BundleAdjuster
{
    // Points must stay this far (metres) in front of every camera that observes them; a step that breaks it is rejected.
    private const double MinDepth = 1e-4;
    // An accepted step that lowers the cost by less than this fraction is the end.
    private const double FunctionTolerance = 1e-6;
    // A step this small relative to the parameters cannot change anything.
    private const double StepTolerance = 1e-10;
    // Bounds of the diagonal used to scale the damping; the floor keeps a parameter with no information damped.
    private const double MinDiagonal = 1e-6, MaxDiagonal = 1e32;

    public static BundleResult Adjust(IReadOnlyList<PhotoView> views, IReadOnlyList<Track> tracks, BundleOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(views);
        ArgumentNullException.ThrowIfNull(tracks);
        options ??= new BundleOptions();
        if (options.MaxIterations < 0) throw new ArgumentOutOfRangeException(nameof(options), "MaxIterations must not be negative.");
        if (!(options.HuberPx > 0)) throw new ArgumentOutOfRangeException(nameof(options), "HuberPx must be positive.");
        if (!(options.CenterPriorSigma > 0)) throw new ArgumentOutOfRangeException(nameof(options), "CenterPriorSigma must be positive.");
        if (!(options.InitialLambda > 0) || double.IsInfinity(options.InitialLambda))
            throw new ArgumentOutOfRangeException(nameof(options), "InitialLambda must be positive and finite.");

        return new Problem(views, tracks, options).Solve();
    }

    private sealed class Problem
    {
        private readonly IReadOnlyList<PhotoView> views;
        private readonly IReadOnlyList<Track> tracks;
        private readonly BundleOptions options;
        private readonly double huber, priorWeight;

        // Cameras in the adjustment (views with at least one observation), by camera index.
        private readonly int cameraCount;
        private readonly int[] cameraView;
        private readonly double[] intrinsics; // fx, fy, cx, cy
        private readonly double[] prior;      // initial centre
        private double[] rotation;            // world→camera, row-major: the rows are the camera axes
        private double[] centre;
        private double[] trialRotation, trialCentre;

        // Points in the adjustment (tracks seen by two photos or more), by point index.
        private readonly int pointCount;
        private readonly int[] pointTrack;
        private double[] point;
        private double[] trialPoint;

        // Observations grouped by point (pointStart[p] to pointStart[p+1]) and, as indices, by camera.
        private readonly int[] pointStart, observationCamera, observationPoint;
        private readonly double[] observed; // u, v
        private readonly int[] cameraStart, cameraObservations;

        // Per observation: robust cost and squared error at the last evaluation; residual, IRLS weight and
        // Jacobians (2×3 in the camera rotation, 2×3 in the point) at the linearization point; the camera-point
        // block W = w·Jcᵀ·Jp (6×3) of the normal equations and T = W·V⁻¹.
        private readonly double[] cost, squared, residual, weight, jacobianRotation, jacobianPoint, coupling, coupled;

        // Normal equations: camera blocks U (6×6) and gradients, point blocks V (3×3) and gradients, V⁻¹ after damping,
        // the damping diagonals, and the reduced camera system with its right-hand side (which becomes the camera step).
        private readonly double[] cameraHessian, cameraGradient, pointHessian, pointGradient, pointInverse;
        private readonly double[] cameraDiagonal, pointDiagonal, reduced, cameraStep, pointStep;

        public Problem(IReadOnlyList<PhotoView> views, IReadOnlyList<Track> tracks, BundleOptions options)
        {
            this.views = views;
            this.tracks = tracks;
            this.options = options;
            huber = options.HuberPx;
            priorWeight = 1 / ((double)options.CenterPriorSigma * options.CenterPriorSigma);

            var viewRotation = new double[views.Count][];
            for (int v = 0; v < views.Count; v++) viewRotation[v] = Orthonormalize(views[v].CameraToWorld, v);

            // Usable observations: in front of the camera as posed now, and at least two photos per track.
            var kept = new List<Observation>();
            var used = new List<(int Track, Observation[] Observations)>();
            var viewUsed = new bool[views.Count];
            for (int t = 0; t < tracks.Count; t++)
            {
                var x = tracks[t].Point;
                kept.Clear();
                foreach (var o in tracks[t].Observations)
                {
                    if (o.View < 0 || o.View >= views.Count)
                        throw new ArgumentException($"Track {t} is observed in photo {o.View}, but there are {views.Count} photos.", nameof(tracks));
                    var r = viewRotation[o.View];
                    var c = views[o.View].CameraToWorld.Translation;
                    double depth = r[6] * ((double)x.X - c.X) + r[7] * ((double)x.Y - c.Y) + r[8] * ((double)x.Z - c.Z);
                    if (depth > MinDepth && float.IsFinite(o.X) && float.IsFinite(o.Y)) kept.Add(o);
                }
                if (kept.Count < 2 || kept.All(o => o.View == kept[0].View)) continue;
                used.Add((t, kept.ToArray()));
                foreach (var o in kept) viewUsed[o.View] = true;
            }

            var cameraOfView = new int[views.Count];
            var cameraViews = new List<int>();
            for (int v = 0; v < views.Count; v++)
            {
                cameraOfView[v] = viewUsed[v] ? cameraViews.Count : -1;
                if (viewUsed[v]) cameraViews.Add(v);
            }
            cameraCount = cameraViews.Count;
            cameraView = cameraViews.ToArray();
            intrinsics = new double[4 * cameraCount];
            prior = new double[3 * cameraCount];
            rotation = new double[9 * cameraCount];
            centre = new double[3 * cameraCount];
            for (int c = 0; c < cameraCount; c++)
            {
                var view = views[cameraView[c]];
                var k = view.Intrinsics;
                intrinsics[4 * c] = k.Fx;
                intrinsics[4 * c + 1] = k.Fy;
                intrinsics[4 * c + 2] = k.Cx;
                intrinsics[4 * c + 3] = k.Cy;
                viewRotation[cameraView[c]].CopyTo(rotation, 9 * c);
                var translation = view.CameraToWorld.Translation;
                prior[3 * c] = centre[3 * c] = translation.X;
                prior[3 * c + 1] = centre[3 * c + 1] = translation.Y;
                prior[3 * c + 2] = centre[3 * c + 2] = translation.Z;
            }

            pointCount = used.Count;
            pointTrack = new int[pointCount];
            point = new double[3 * pointCount];
            pointStart = new int[pointCount + 1];
            int observationCount = used.Sum(u => u.Observations.Length);
            observationCamera = new int[observationCount];
            observationPoint = new int[observationCount];
            observed = new double[2 * observationCount];
            var perCamera = new int[cameraCount + 1];
            for (int p = 0, o = 0; p < pointCount; p++)
            {
                var (t, observations) = used[p];
                pointTrack[p] = t;
                var x = tracks[t].Point;
                point[3 * p] = x.X;
                point[3 * p + 1] = x.Y;
                point[3 * p + 2] = x.Z;
                pointStart[p] = o;
                foreach (var ob in observations)
                {
                    int c = cameraOfView[ob.View];
                    observationCamera[o] = c;
                    observationPoint[o] = p;
                    observed[2 * o] = ob.X;
                    observed[2 * o + 1] = ob.Y;
                    perCamera[c + 1]++;
                    o++;
                }
                pointStart[p + 1] = o;
            }

            cameraStart = new int[cameraCount + 1];
            for (int c = 0; c < cameraCount; c++) cameraStart[c + 1] = cameraStart[c] + perCamera[c + 1];
            cameraObservations = new int[observationCount];
            var fill = (int[])cameraStart.Clone();
            for (int o = 0; o < observationCount; o++) cameraObservations[fill[observationCamera[o]]++] = o;

            trialRotation = new double[rotation.Length];
            trialCentre = new double[centre.Length];
            trialPoint = new double[point.Length];
            cost = new double[observationCount];
            squared = new double[observationCount];
            residual = new double[2 * observationCount];
            weight = new double[observationCount];
            jacobianRotation = new double[6 * observationCount];
            jacobianPoint = new double[6 * observationCount];
            coupling = new double[18 * observationCount];
            coupled = new double[18 * observationCount];
            cameraHessian = new double[36 * cameraCount];
            cameraGradient = new double[6 * cameraCount];
            cameraDiagonal = new double[6 * cameraCount];
            cameraStep = new double[6 * cameraCount];
            reduced = new double[36 * cameraCount * cameraCount];
            pointHessian = new double[9 * pointCount];
            pointGradient = new double[3 * pointCount];
            pointInverse = new double[9 * pointCount];
            pointDiagonal = new double[3 * pointCount];
            pointStep = new double[3 * pointCount];
        }

        public BundleResult Solve()
        {
            double current = Evaluate(rotation, centre, point, linearize: true);
            double initialRms = Rms();
            double lambda = options.InitialLambda, growth = 2;
            int iterations = 0;
            bool converged = current == 0;

            while (!converged && iterations < options.MaxIterations)
            {
                iterations++;
                if (!TryStep(lambda, out double predicted, out double stepNorm))
                {
                    (lambda, growth) = (lambda * growth, growth * 2);
                    continue;
                }
                if (stepNorm <= StepTolerance * (ParameterNorm() + StepTolerance))
                {
                    converged = true;
                    break;
                }

                ApplyStep();
                double trial = Evaluate(trialRotation, trialCentre, trialPoint, linearize: false);
                if (!(trial < current))
                {
                    (lambda, growth) = (lambda * growth, growth * 2);
                    continue;
                }

                // Nielsen's rule: shrink the damping when the quadratic model predicted the decrease well.
                double decrease = current - trial;
                double gain = predicted > 0 ? decrease / predicted : 0.5;
                lambda *= Math.Max(1.0 / 3, 1 - Math.Pow(2 * gain - 1, 3));
                growth = 2;
                (rotation, trialRotation) = (trialRotation, rotation);
                (centre, trialCentre) = (trialCentre, centre);
                (point, trialPoint) = (trialPoint, point);
                AlignGauge();
                double next = Evaluate(rotation, centre, point, linearize: true);
                bool small = current - next <= FunctionTolerance * current;
                current = next;
                if (small)
                {
                    converged = true;
                    break;
                }
            }

            Evaluate(rotation, centre, point, linearize: false);
            return new BundleResult(Poses(), Points(), initialRms, Rms(), iterations, converged);
        }

        /// <summary>The robust cost at the given parameters (+∞ if a point falls behind a camera), filling the
        /// per-observation costs and squared errors; with <paramref name="linearize"/> also the Jacobians and
        /// normal equations, which assumes the parameters are the current ones.</summary>
        private double Evaluate(double[] rotations, double[] centres, double[] points, bool linearize)
        {
            bool behind = false;
            Parallel.For(0, pointCount, p =>
            {
                double x0 = points[3 * p], x1 = points[3 * p + 1], x2 = points[3 * p + 2];
                for (int o = pointStart[p]; o < pointStart[p + 1]; o++)
                {
                    int c = observationCamera[o], r = 9 * c;
                    double d0 = x0 - centres[3 * c], d1 = x1 - centres[3 * c + 1], d2 = x2 - centres[3 * c + 2];
                    double y0 = rotations[r] * d0 + rotations[r + 1] * d1 + rotations[r + 2] * d2;
                    double y1 = rotations[r + 3] * d0 + rotations[r + 4] * d1 + rotations[r + 5] * d2;
                    double y2 = rotations[r + 6] * d0 + rotations[r + 7] * d1 + rotations[r + 8] * d2;
                    if (!(y2 > MinDepth))
                    {
                        behind = true;
                        cost[o] = double.PositiveInfinity;
                        continue;
                    }

                    double fx = intrinsics[4 * c], fy = intrinsics[4 * c + 1], iz = 1 / y2;
                    double e0 = fx * y0 * iz + intrinsics[4 * c + 2] - observed[2 * o];
                    double e1 = fy * y1 * iz + intrinsics[4 * c + 3] - observed[2 * o + 1];
                    double e2 = e0 * e0 + e1 * e1, e = Math.Sqrt(e2);
                    squared[o] = e2;
                    // Huber on the pixel distance; IRLS weight ρ'(e)/(2e), so that w·e² has ρ's gradient.
                    double w;
                    if (e <= huber)
                    {
                        cost[o] = e2;
                        w = 1;
                    }
                    else
                    {
                        cost[o] = 2 * huber * e - huber * huber;
                        w = huber / e;
                    }
                    if (!linearize) continue;

                    residual[2 * o] = e0;
                    residual[2 * o + 1] = e1;
                    weight[o] = w;
                    // Pinhole derivative A = ∂(u,v)/∂Y = [fx/z 0 −fx·x/z²; 0 fy/z −fy·y/z²] with Y = R(X − C):
                    // ∂Y/∂X = R, ∂Y/∂C = −R, and ∂Y/∂w = −[Y]× because exp([w]×)·Y ≈ Y + w × Y.
                    double a00 = fx * iz, a02 = -fx * y0 * iz * iz, a11 = fy * iz, a12 = -fy * y1 * iz * iz;
                    int j = 6 * o;
                    for (int k = 0; k < 3; k++)
                    {
                        jacobianPoint[j + k] = a00 * rotations[r + k] + a02 * rotations[r + 6 + k];
                        jacobianPoint[j + 3 + k] = a11 * rotations[r + 3 + k] + a12 * rotations[r + 6 + k];
                    }
                    jacobianRotation[j] = a02 * y1;
                    jacobianRotation[j + 1] = a00 * y2 - a02 * y0;
                    jacobianRotation[j + 2] = -a00 * y1;
                    jacobianRotation[j + 3] = a12 * y1 - a11 * y2;
                    jacobianRotation[j + 4] = -a12 * y0;
                    jacobianRotation[j + 5] = a11 * y0;
                }
                if (linearize && !behind) LinearizePoint(p);
            });
            if (behind) return double.PositiveInfinity;

            // Summed in a fixed order, so the same input gives the same result bit for bit.
            double total = 0;
            for (int o = 0; o < cost.Length; o++) total += cost[o];
            if (linearize) LinearizeCameras();
            return total + PriorCost(centres);
        }

        // V = Σ w·Jpᵀ·Jp, its gradient Σ w·Jpᵀ·r, and W = w·Jcᵀ·Jp for each observation, with Jc = [Jw | −Jp].
        private void LinearizePoint(int p)
        {
            var v = pointHessian.AsSpan(9 * p, 9);
            var g = pointGradient.AsSpan(3 * p, 3);
            v.Clear();
            g.Clear();
            for (int o = pointStart[p]; o < pointStart[p + 1]; o++)
            {
                double w = weight[o], r0 = w * residual[2 * o], r1 = w * residual[2 * o + 1];
                int j = 6 * o, m = 18 * o;
                for (int a = 0; a < 3; a++)
                {
                    double p0 = jacobianPoint[j + a], p1 = jacobianPoint[j + 3 + a];
                    double w0 = jacobianRotation[j + a], w1 = jacobianRotation[j + 3 + a];
                    g[a] += p0 * r0 + p1 * r1;
                    for (int b = 0; b < 3; b++)
                    {
                        double q0 = jacobianPoint[j + b], q1 = jacobianPoint[j + 3 + b];
                        double pp = w * (p0 * q0 + p1 * q1);
                        v[3 * a + b] += pp;
                        coupling[m + 3 * a + b] = w * (w0 * q0 + w1 * q1);
                        coupling[m + 9 + 3 * a + b] = -pp;
                    }
                }
            }
        }

        // U = Σ w·Jcᵀ·Jc and its gradient, plus the centre prior.
        private void LinearizeCameras()
        {
            Parallel.For(0, cameraCount, c =>
            {
                var u = cameraHessian.AsSpan(36 * c, 36);
                var g = cameraGradient.AsSpan(6 * c, 6);
                u.Clear();
                g.Clear();
                Span<double> jc = stackalloc double[12];
                for (int i = cameraStart[c]; i < cameraStart[c + 1]; i++)
                {
                    int o = cameraObservations[i], j = 6 * o;
                    for (int row = 0; row < 2; row++)
                    for (int k = 0; k < 3; k++)
                    {
                        jc[6 * row + k] = jacobianRotation[j + 3 * row + k];
                        jc[6 * row + 3 + k] = -jacobianPoint[j + 3 * row + k];
                    }
                    double w = weight[o], r0 = w * residual[2 * o], r1 = w * residual[2 * o + 1];
                    for (int a = 0; a < 6; a++)
                    {
                        g[a] += jc[a] * r0 + jc[6 + a] * r1;
                        double wa0 = w * jc[a], wa1 = w * jc[6 + a];
                        for (int b = 0; b <= a; b++) u[6 * a + b] += wa0 * jc[b] + wa1 * jc[6 + b];
                    }
                }
                for (int a = 0; a < 6; a++)
                for (int b = 0; b < a; b++)
                    u[6 * b + a] = u[6 * a + b];
                for (int k = 0; k < 3; k++)
                {
                    u[7 * (3 + k)] += priorWeight;
                    g[3 + k] += priorWeight * (centre[3 * c + k] - prior[3 * c + k]);
                }
            });
        }

        /// <summary>Solves the damped normal equations (H + λ·D)·δ = −g through the reduced camera system; false when
        /// the damping is too weak for them to be positive definite. Also returns the decrease the quadratic model
        /// predicts, −2gᵀδ − δᵀHδ = δᵀ(λDδ − g) for this cost's scaling, and |δ|.</summary>
        private bool TryStep(double lambda, out double predicted, out double stepNorm)
        {
            predicted = stepNorm = 0;
            int n = 6 * cameraCount;

            bool singular = false;
            Parallel.For(0, pointCount, p =>
            {
                Span<double> v = stackalloc double[9];
                pointHessian.AsSpan(9 * p, 9).CopyTo(v);
                for (int a = 0; a < 3; a++)
                {
                    double d = Math.Clamp(v[4 * a], MinDiagonal, MaxDiagonal);
                    pointDiagonal[3 * p + a] = d;
                    v[4 * a] += lambda * d;
                }
                var inverse = pointInverse.AsSpan(9 * p, 9);
                if (!InvertSymmetric3(v, inverse))
                {
                    singular = true;
                    return;
                }
                for (int o = pointStart[p]; o < pointStart[p + 1]; o++)
                {
                    int m = 18 * o;
                    for (int a = 0; a < 6; a++)
                    for (int k = 0; k < 3; k++)
                        coupled[m + 3 * a + k] = coupling[m + 3 * a] * inverse[k] + coupling[m + 3 * a + 1] * inverse[3 + k]
                                                 + coupling[m + 3 * a + 2] * inverse[6 + k];
                }
            });
            if (singular) return false;

            // S = U + λ·D − Σ W·V⁻¹·Wᵀ and its right-hand side −g_c + Σ W·V⁻¹·g_p, one block row per camera so no
            // two threads write the same entry. Only the lower triangle is filled; the factorization reads no more.
            Array.Clear(reduced);
            Parallel.For(0, cameraCount, c =>
            {
                int top = 6 * c;
                for (int a = 0; a < 6; a++)
                {
                    double d = Math.Clamp(cameraHessian[36 * c + 7 * a], MinDiagonal, MaxDiagonal);
                    cameraDiagonal[top + a] = d;
                    for (int b = 0; b <= a; b++) reduced[(top + a) * n + top + b] = cameraHessian[36 * c + 6 * a + b];
                    reduced[(top + a) * n + top + a] += lambda * d;
                    cameraStep[top + a] = -cameraGradient[top + a];
                }
                for (int i = cameraStart[c]; i < cameraStart[c + 1]; i++)
                {
                    int o = cameraObservations[i], p = observationPoint[o], m = 18 * o;
                    for (int a = 0; a < 6; a++)
                        cameraStep[top + a] += coupled[m + 3 * a] * pointGradient[3 * p] + coupled[m + 3 * a + 1] * pointGradient[3 * p + 1]
                                               + coupled[m + 3 * a + 2] * pointGradient[3 * p + 2];
                    for (int o2 = pointStart[p]; o2 < pointStart[p + 1]; o2++)
                    {
                        int c2 = observationCamera[o2], m2 = 18 * o2;
                        if (c2 > c) continue;
                        for (int a = 0; a < 6; a++)
                        {
                            double t0 = coupled[m + 3 * a], t1 = coupled[m + 3 * a + 1], t2 = coupled[m + 3 * a + 2];
                            int row = (top + a) * n + 6 * c2;
                            for (int b = 0; b < 6; b++)
                                reduced[row + b] -= t0 * coupling[m2 + 3 * b] + t1 * coupling[m2 + 3 * b + 1] + t2 * coupling[m2 + 3 * b + 2];
                        }
                    }
                }
            });
            if (!DenseCholesky.TryFactor(reduced, n)) return false;
            DenseCholesky.Solve(reduced, n, cameraStep);

            // δ_p = V⁻¹·(−g_p − Σ Wᵀ·δ_c).
            Parallel.For(0, pointCount, p =>
            {
                Span<double> b = stackalloc double[3];
                for (int k = 0; k < 3; k++) b[k] = -pointGradient[3 * p + k];
                for (int o = pointStart[p]; o < pointStart[p + 1]; o++)
                {
                    int c = 6 * observationCamera[o], m = 18 * o;
                    for (int a = 0; a < 6; a++)
                    {
                        double step = cameraStep[c + a];
                        for (int k = 0; k < 3; k++) b[k] -= coupling[m + 3 * a + k] * step;
                    }
                }
                for (int k = 0; k < 3; k++)
                    pointStep[3 * p + k] = pointInverse[9 * p + 3 * k] * b[0] + pointInverse[9 * p + 3 * k + 1] * b[1]
                                           + pointInverse[9 * p + 3 * k + 2] * b[2];
            });

            double norm = 0;
            for (int i = 0; i < cameraStep.Length; i++)
            {
                double s = cameraStep[i];
                predicted += s * (lambda * cameraDiagonal[i] * s - cameraGradient[i]);
                norm += s * s;
            }
            for (int i = 0; i < pointStep.Length; i++)
            {
                double s = pointStep[i];
                predicted += s * (lambda * pointDiagonal[i] * s - pointGradient[i]);
                norm += s * s;
            }
            stepNorm = Math.Sqrt(norm);
            return double.IsFinite(stepNorm);
        }

        private void ApplyStep()
        {
            for (int c = 0; c < cameraCount; c++)
            {
                RotateLeft(cameraStep.AsSpan(6 * c, 3), rotation.AsSpan(9 * c, 9), trialRotation.AsSpan(9 * c, 9));
                for (int k = 0; k < 3; k++) trialCentre[3 * c + k] = centre[3 * c + k] + cameraStep[6 * c + 3 + k];
            }
            for (int i = 0; i < point.Length; i++) trialPoint[i] = point[i] + pointStep[i];
        }

        /// <summary>
        /// Moves the whole scene by the similarity that best fits the camera centres to their priors. Reprojections
        /// cannot tell the scene from a scaled, turned and shifted copy of it, so along those seven directions only the
        /// priors shape the cost, and next to the damping of the stiff directions they are so flat that
        /// Levenberg-Marquardt would crawl along them for dozens of iterations (long steps do not help: a linearized
        /// turn of the whole scene is not exactly free). Solved directly, a few Gauss-Newton steps on seven
        /// parameters, the similarity leaves every reprojection as it was and lowers only the prior cost.
        /// </summary>
        private void AlignGauge()
        {
            if (cameraCount == 0) return;
            Span<double> mean = stackalloc double[3];
            for (int c = 0; c < cameraCount; c++)
            for (int k = 0; k < 3; k++)
                mean[k] += centre[3 * c + k] / cameraCount;

            // x' = s·Q·(x − mean) + mean + t, refined from the identity.
            Span<double> q = [1, 0, 0, 0, 1, 0, 0, 0, 1];
            Span<double> turned = stackalloc double[9];
            Span<double> t = stackalloc double[3];
            Span<double> m = stackalloc double[3];
            Span<double> j = stackalloc double[21];
            double s = 1;
            var normal = new double[49];
            var step = new double[7];
            for (int iteration = 0; iteration < 10; iteration++)
            {
                Array.Clear(normal);
                Array.Clear(step);
                for (int c = 0; c < cameraCount; c++)
                {
                    double d0 = centre[3 * c] - mean[0], d1 = centre[3 * c + 1] - mean[1], d2 = centre[3 * c + 2] - mean[2];
                    for (int k = 0; k < 3; k++) m[k] = s * (q[3 * k] * d0 + q[3 * k + 1] * d1 + q[3 * k + 2] * d2);
                    // Rows of ∂x'/∂(φ, σ, t) for a turn exp([φ]×)·Q, a scale s·e^σ and a shift: −[m]×, m, I.
                    j.Clear();
                    j[1] = m[2]; j[2] = -m[1];
                    j[7] = -m[2]; j[9] = m[0];
                    j[14] = m[1]; j[15] = -m[0];
                    for (int k = 0; k < 3; k++)
                    {
                        j[7 * k + 3] = m[k];
                        j[7 * k + 4 + k] = 1;
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        double r = m[k] + mean[k] + t[k] - prior[3 * c + k];
                        for (int a = 0; a < 7; a++)
                        {
                            step[a] -= j[7 * k + a] * r;
                            for (int b = 0; b <= a; b++) normal[7 * a + b] += j[7 * k + a] * j[7 * k + b];
                        }
                    }
                }
                // A little Marquardt damping: cameras in a line leave the turn about that line free.
                for (int a = 0; a < 7; a++) normal[8 * a] += 1e-9 * normal[8 * a] + 1e-12;
                if (!DenseCholesky.TryFactor(normal, 7)) return;
                DenseCholesky.Solve(normal, 7, step);

                RotateLeft(step.AsSpan(0, 3), q, turned);
                turned.CopyTo(q);
                s *= Math.Exp(step[3]);
                for (int k = 0; k < 3; k++) t[k] += step[4 + k];
                if (step.All(x => Math.Abs(x) < 1e-12)) break;
            }

            for (int c = 0; c < cameraCount; c++)
            {
                Transform(centre.AsSpan(3 * c, 3), trialCentre.AsSpan(3 * c, 3), q, s, mean, t);
                // R' = R·Qᵀ, so that R'·(x' − C') = s·R·(x − C): every camera sees the same image.
                var r = rotation.AsSpan(9 * c, 9);
                for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    trialRotation[9 * c + 3 * a + b] = r[3 * a] * q[3 * b] + r[3 * a + 1] * q[3 * b + 1] + r[3 * a + 2] * q[3 * b + 2];
            }
            if (!(PriorCost(trialCentre) < PriorCost(centre))) return;
            for (int p = 0; p < pointCount; p++) Transform(point.AsSpan(3 * p, 3), trialPoint.AsSpan(3 * p, 3), q, s, mean, t);
            (rotation, trialRotation) = (trialRotation, rotation);
            (centre, trialCentre) = (trialCentre, centre);
            (point, trialPoint) = (trialPoint, point);
        }

        private static void Transform(ReadOnlySpan<double> x, Span<double> result, ReadOnlySpan<double> q, double s,
            ReadOnlySpan<double> mean, ReadOnlySpan<double> t)
        {
            double d0 = x[0] - mean[0], d1 = x[1] - mean[1], d2 = x[2] - mean[2];
            for (int k = 0; k < 3; k++) result[k] = s * (q[3 * k] * d0 + q[3 * k + 1] * d1 + q[3 * k + 2] * d2) + mean[k] + t[k];
        }

        private double PriorCost(double[] centres)
        {
            double sum = 0;
            for (int i = 0; i < centres.Length; i++)
            {
                double d = centres[i] - prior[i];
                sum += d * d;
            }
            return priorWeight * sum;
        }

        // The rotation increments are zero at the current parameters, so only positions count.
        private double ParameterNorm()
        {
            double sum = 0;
            foreach (double x in centre) sum += x * x;
            foreach (double x in point) sum += x * x;
            return Math.Sqrt(sum);
        }

        private double Rms()
        {
            if (squared.Length == 0) return 0;
            double sum = 0;
            foreach (double e2 in squared) sum += e2;
            return Math.Sqrt(sum / squared.Length);
        }

        private Matrix4x4[] Poses()
        {
            var poses = views.Select(v => v.CameraToWorld).ToArray();
            for (int c = 0; c < cameraCount; c++)
            {
                var r = rotation.AsSpan(9 * c, 9);
                poses[cameraView[c]] = new Matrix4x4(
                    (float)r[0], (float)r[1], (float)r[2], 0,
                    (float)r[3], (float)r[4], (float)r[5], 0,
                    (float)r[6], (float)r[7], (float)r[8], 0,
                    (float)centre[3 * c], (float)centre[3 * c + 1], (float)centre[3 * c + 2], 1);
            }
            return poses;
        }

        private Vector3[] Points()
        {
            var points = tracks.Select(t => t.Point).ToArray();
            for (int p = 0; p < pointCount; p++)
                points[pointTrack[p]] = new Vector3((float)point[3 * p], (float)point[3 * p + 1], (float)point[3 * p + 2]);
            return points;
        }
    }

    /// <summary>The rotation part of a pose, made exactly orthonormal: single-precision poses are orthonormal to about
    /// 1e-7, and the updates exp([w]×)·R preserve whatever the start was. Newton's iteration for the polar factor,
    /// R ← (R + R⁻ᵀ)/2, converges quadratically from there.</summary>
    private static double[] Orthonormalize(Matrix4x4 m, int view)
    {
        double[] r = [m.M11, m.M12, m.M13, m.M21, m.M22, m.M23, m.M31, m.M32, m.M33];
        Span<double> cofactor = stackalloc double[9];
        for (int iteration = 0; iteration < 10; iteration++)
        {
            cofactor[0] = r[4] * r[8] - r[5] * r[7];
            cofactor[1] = r[5] * r[6] - r[3] * r[8];
            cofactor[2] = r[3] * r[7] - r[4] * r[6];
            cofactor[3] = r[2] * r[7] - r[1] * r[8];
            cofactor[4] = r[0] * r[8] - r[2] * r[6];
            cofactor[5] = r[1] * r[6] - r[0] * r[7];
            cofactor[6] = r[1] * r[5] - r[2] * r[4];
            cofactor[7] = r[2] * r[3] - r[0] * r[5];
            cofactor[8] = r[0] * r[4] - r[1] * r[3];
            double determinant = r[0] * cofactor[0] + r[1] * cofactor[1] + r[2] * cofactor[2];
            if (!(Math.Abs(determinant) > 1e-6))
                throw new ArgumentException($"Photo {view} has a pose whose 3x3 part is not a rotation.", "views");
            double change = 0;
            for (int i = 0; i < 9; i++)
            {
                double next = 0.5 * (r[i] + cofactor[i] / determinant);
                change = Math.Max(change, Math.Abs(next - r[i]));
                r[i] = next;
            }
            if (change < 1e-15) break;
        }
        return r;
    }

    /// <summary>result = exp([w]×)·r by Rodrigues' formula, I + a·[w]× + b·[w]×², with a = sin θ/θ and
    /// b = (1 − cos θ)/θ² = 2·(sin(θ/2)/θ)², the latter form free of cancellation; Taylor terms for tiny θ.</summary>
    private static void RotateLeft(ReadOnlySpan<double> w, ReadOnlySpan<double> r, Span<double> result)
    {
        double x = w[0], y = w[1], z = w[2], t2 = x * x + y * y + z * z;
        double a, b;
        if (t2 < 1e-8)
        {
            a = 1 - t2 / 6;
            b = 0.5 - t2 / 24;
        }
        else
        {
            double t = Math.Sqrt(t2), half = Math.Sin(0.5 * t) / t;
            a = Math.Sin(t) / t;
            b = 2 * half * half;
        }
        // [w]×² = w·wᵀ − θ²·I
        Span<double> e =
        [
            1 + b * (x * x - t2), -a * z + b * x * y, a * y + b * x * z,
            a * z + b * x * y, 1 + b * (y * y - t2), -a * x + b * y * z,
            -a * y + b * x * z, a * x + b * y * z, 1 + b * (z * z - t2),
        ];
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result[3 * i + j] = e[3 * i] * r[j] + e[3 * i + 1] * r[3 + j] + e[3 * i + 2] * r[6 + j];
    }

    /// <summary>Inverse of a symmetric positive-definite 3×3 by cofactors; false when it is singular to working
    /// precision.</summary>
    private static bool InvertSymmetric3(ReadOnlySpan<double> m, Span<double> inverse)
    {
        double c00 = m[4] * m[8] - m[5] * m[7];
        double c01 = m[5] * m[6] - m[3] * m[8];
        double c02 = m[3] * m[7] - m[4] * m[6];
        double determinant = m[0] * c00 + m[1] * c01 + m[2] * c02;
        if (!(determinant > 1e-14 * m[0] * m[4] * m[8]) || !double.IsFinite(determinant)) return false;
        double s = 1 / determinant;
        inverse[0] = c00 * s;
        inverse[1] = inverse[3] = c01 * s;
        inverse[2] = inverse[6] = c02 * s;
        inverse[4] = (m[0] * m[8] - m[2] * m[6]) * s;
        inverse[5] = inverse[7] = (m[2] * m[3] - m[0] * m[5]) * s;
        inverse[8] = (m[0] * m[4] - m[1] * m[3]) * s;
        return true;
    }
}
