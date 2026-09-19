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
