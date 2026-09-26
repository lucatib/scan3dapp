using Scanner.Capture;

namespace Scanner.Core.Photogrammetry;

/// <param name="WindowRadius">The matching window is (2r+1)² pixels.</param>
/// <param name="DepthSamples">Depth hypotheses, spaced evenly in inverse depth.</param>
/// <param name="MinScore">Weakest aggregated correlation accepted as a match.</param>
/// <param name="MinTextureStd">Weakest reference-window standard deviation, in gray levels, worth matching: below it
/// every hypothesis correlates about equally and the winner is noise.</param>
/// <param name="BestOf">How many of the best neighbour correlations are averaged per hypothesis; averaging the best
/// two, not all, keeps a pixel occluded in one neighbour from being dragged down by it.</param>
public sealed record StereoOptions(int WindowRadius = 3, int DepthSamples = 96, float MinScore = 0.5f,
    float MinTextureStd = 3f, int BestOf = 2);

/// <summary>
/// Depth map of a reference photo from neighbouring photos with known poses. Every hypothesis is a plane parallel to
/// the reference image; each neighbour is warped onto the reference through that plane's homography and compared
/// with normalized cross-correlation over a window. Window sums are separable box sums, so a hypothesis costs the
/// same whatever the window size.
/// </summary>
public static class PlaneSweepStereo
{
    private const float NoScore = -2f;

    public static DepthMap Compute(PhotoView reference, IReadOnlyList<PhotoView> neighbours, float minDepth, float maxDepth,
        StereoOptions? options = null)
    {
        options ??= new StereoOptions();
        minDepth = MathF.Max(minDepth, 0.05f);
        if (maxDepth <= minDepth) throw new ArgumentException("The depth range is empty.", nameof(maxDepth));

        int w = reference.Image.Width, h = reference.Image.Height, n = w * h, r = options.WindowRadius;
        float area = (2 * r + 1) * (2 * r + 1);
        var depth = new float[n];
        var score = new float[n];
        if (neighbours.Count == 0) return new DepthMap(w, h, depth, score);

        var refPixels = reference.Image.Pixels.Select(p => (float)p).ToArray();
        var tmp = new float[n];
        var sumR = BoxSum(refPixels, new float[n], tmp, w, h, r);
        var sumR2 = BoxSum(refPixels.Select(p => p * p).ToArray(), new float[n], tmp, w, h, r);

        // Pixels worth matching: the whole window inside the image, and enough texture in it.
        var textured = new bool[n];
        float minSpread = options.MinTextureStd * options.MinTextureStd * area;
        for (int y = r; y < h - r; y++)
        for (int x = r; x < w - r; x++)
        {
            int i = y * w + x;
            textured[i] = sumR2[i] - sumR[i] * sumR[i] / area >= minSpread;
        }

        var relative = neighbours.Select(nb => Relative(reference, nb)).ToArray();
        int count = options.DepthSamples;
        float nearInverse = 1f / minDepth, farInverse = 1f / maxDepth;
        float InverseDepth(float index) => farInverse + (nearInverse - farInverse) * index / (count - 1);

        var best = Filled(n, NoScore);
        var bestIndex = new int[n];
        var before = Filled(n, NoScore);
        var after = Filled(n, NoScore);
        var last = Filled(n, NoScore);
        var top1 = new float[n];
        var top2 = new float[n];
        var warped = new float[n];
        var warped2 = new float[n];
        var product = new float[n];
        var valid = new float[n];
        var sumW = new float[n];
        var sumW2 = new float[n];
        var sumRW = new float[n];
        var sumV = new float[n];
        bool average = options.BestOf >= 2 && neighbours.Count >= 2;

        for (int s = 0; s < count; s++)
        {
            float d = 1f / InverseDepth(s);
            Array.Fill(top1, -1f);
            Array.Fill(top2, -1f);
            for (int j = 0; j < neighbours.Count; j++)
            {
                var homography = Homography(reference.Intrinsics, neighbours[j].Intrinsics, relative[j], d);
                Warp(neighbours[j].Image, homography, refPixels, warped, warped2, product, valid, w, h);
                BoxSum(warped, sumW, tmp, w, h, r);
                BoxSum(warped2, sumW2, tmp, w, h, r);
                BoxSum(product, sumRW, tmp, w, h, r);
                BoxSum(valid, sumV, tmp, w, h, r);
                Parallel.For(0, h, y =>
                {
                    for (int x = 0, i = y * w; x < w; x++, i++)
                    {
                        if (!textured[i] || sumV[i] < area - 0.5f) continue;
                        double varR = sumR2[i] - (double)sumR[i] * sumR[i] / area;
                        double varW = sumW2[i] - (double)sumW[i] * sumW[i] / area;
                        if (varW <= 1e-3 * area) continue;
                        float ncc = (float)((sumRW[i] - (double)sumR[i] * sumW[i] / area) / Math.Sqrt(varR * varW));
                        if (ncc > top1[i]) { top2[i] = top1[i]; top1[i] = ncc; }
                        else if (ncc > top2[i]) top2[i] = ncc;
                    }
                });
            }

            Parallel.For(0, h, y =>
            {
                for (int x = 0, i = y * w; x < w; x++, i++)
                {
                    if (!textured[i]) continue;
                    float value = average ? (top1[i] + top2[i]) * 0.5f : top1[i];
                    if (value > best[i])
                    {
                        best[i] = value;
                        bestIndex[i] = s;
                        before[i] = last[i];
                        after[i] = NoScore;
                    }
                    else if (bestIndex[i] == s - 1) after[i] = value;
                    last[i] = value;
                }
            });
        }

        for (int i = 0; i < n; i++)
        {
            int b = bestIndex[i];
            // A best hypothesis at either end of the range is a surface outside it, not a measurement.
            if (!textured[i] || best[i] < options.MinScore || b <= 0 || b >= count - 1) continue;
            float offset = 0;
            float curvature = before[i] - 2 * best[i] + after[i];
            if (before[i] > NoScore && after[i] > NoScore && curvature < 0)
                offset = Math.Clamp(0.5f * (before[i] - after[i]) / curvature, -0.5f, 0.5f);
            depth[i] = 1f / InverseDepth(b + offset);
            score[i] = best[i];
        }
        return new DepthMap(w, h, depth, score);
    }

    /// <summary>Rotation R (row-major 3x3) and translation t taking reference-camera points to the neighbour camera.</summary>
    private static (double[] R, double[] T) Relative(PhotoView reference, PhotoView neighbour)
    {
        var a = Axes(reference);
        var b = Axes(neighbour);
        var rotation = new double[9];
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rotation[i * 3 + j] = b[i * 3] * a[j * 3] + b[i * 3 + 1] * a[j * 3 + 1] + b[i * 3 + 2] * a[j * 3 + 2];
        var delta = reference.CameraToWorld.Translation - neighbour.CameraToWorld.Translation;
        var translation = new double[3];
        for (int i = 0; i < 3; i++) translation[i] = b[i * 3] * delta.X + b[i * 3 + 1] * delta.Y + b[i * 3 + 2] * delta.Z;
        return (rotation, translation);
    }

    // The camera's axes in world coordinates, one per row: the top-left 3x3 of a row-vector camera→world matrix.
    private static double[] Axes(PhotoView view)
    {
        var m = view.CameraToWorld;
        return [m.M11, m.M12, m.M13, m.M21, m.M22, m.M23, m.M31, m.M32, m.M33];
    }

    /// <summary>H = K_n (d R + t e3ᵀ) K_r⁻¹: reference pixel → neighbour pixel through the plane z = d of the reference.</summary>
    private static double[] Homography(CameraIntrinsics kr, CameraIntrinsics kn, (double[] R, double[] T) pose, float d)
    {
        var m = new double[9];
        for (int i = 0; i < 9; i++) m[i] = d * pose.R[i];
        for (int i = 0; i < 3; i++) m[i * 3 + 2] += pose.T[i];

        // m · K_r⁻¹, with K_r⁻¹ = [1/fx 0 -cx/fx; 0 1/fy -cy/fy; 0 0 1].
        var mk = new double[9];
        for (int i = 0; i < 3; i++)
        {
            mk[i * 3] = m[i * 3] / kr.Fx;
            mk[i * 3 + 1] = m[i * 3 + 1] / kr.Fy;
            mk[i * 3 + 2] = m[i * 3 + 2] - m[i * 3] * kr.Cx / kr.Fx - m[i * 3 + 1] * kr.Cy / kr.Fy;
        }

        // K_n · (m K_r⁻¹), with K_n = [fx 0 cx; 0 fy cy; 0 0 1].
        var hm = new double[9];
        for (int j = 0; j < 3; j++)
        {
            hm[j] = kn.Fx * mk[j] + kn.Cx * mk[6 + j];
            hm[3 + j] = kn.Fy * mk[3 + j] + kn.Cy * mk[6 + j];
            hm[6 + j] = mk[6 + j];
        }
        return hm;
    }

    private static void Warp(GrayImage source, double[] hm, float[] reference, float[] warped, float[] warped2,
        float[] product, float[] valid, int w, int h)
    {
        int sw = source.Width, sh = source.Height;
        byte[] px = source.Pixels;
        Parallel.For(0, h, y =>
        {
            double qx = hm[1] * y + hm[2], qy = hm[4] * y + hm[5], qz = hm[7] * y + hm[8];
            for (int x = 0, i = y * w; x < w; x++, i++, qx += hm[0], qy += hm[3], qz += hm[6])
            {
                if (qz > 1e-9)
                {
                    double fx = qx / qz, fy = qy / qz;
                    if (fx >= 0 && fy >= 0 && fx < sw - 1 && fy < sh - 1)
                    {
                        int x0 = (int)fx, y0 = (int)fy;
                        float ax = (float)(fx - x0), ay = (float)(fy - y0);
                        int o = y0 * sw + x0;
                        float top = px[o] + (px[o + 1] - px[o]) * ax;
                        float bottom = px[o + sw] + (px[o + sw + 1] - px[o + sw]) * ax;
                        float value = top + (bottom - top) * ay;
                        warped[i] = value;
                        warped2[i] = value * value;
                        product[i] = value * reference[i];
                        valid[i] = 1;
                        continue;
                    }
                }
                warped[i] = warped2[i] = product[i] = valid[i] = 0;
            }
        });
    }

    /// <summary>Sum over the (2r+1)² window around each pixel, clipped at the image border; separable.</summary>
    private static float[] BoxSum(float[] source, float[] destination, float[] tmp, int w, int h, int r)
    {
        Parallel.For(0, h, y =>
        {
            int row = y * w;
            for (int x = 0; x < w; x++)
            {
                float sum = 0;
                for (int k = Math.Max(0, x - r), end = Math.Min(w - 1, x + r); k <= end; k++) sum += source[row + k];
                tmp[row + x] = sum;
            }
        });
        Parallel.For(0, h, y =>
        {
            int row = y * w;
            Array.Clear(destination, row, w);
            for (int k = Math.Max(0, y - r), end = Math.Min(h - 1, y + r); k <= end; k++)
            {
                int source = k * w;
                for (int x = 0; x < w; x++) destination[row + x] += tmp[source + x];
            }
        });
        return destination;
    }

    private static float[] Filled(int n, float value)
    {
        var array = new float[n];
        Array.Fill(array, value);
        return array;
    }
}
