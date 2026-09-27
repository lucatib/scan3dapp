using System.Numerics;
using System.Runtime.InteropServices;

namespace Scanner.Core.LinearAlgebra;

/// <summary>
/// Cholesky factorization A = L·Lᵀ of a dense symmetric positive-definite matrix, and solves with the factor.
/// Matrices are flattened row-major n×n arrays and only their lower triangle is read, so a caller assembling a
/// symmetric system (the reduced camera system of bundle adjustment, say) fills half of it. Rows are what the inner
/// loops walk, so every dot product runs over contiguous memory; the rows below each pivot are independent and are
/// split across threads once the matrix is large enough to pay for it.
/// </summary>
public static class DenseCholesky
{
    // A pivot that keeps fewer than about twelve digits of its diagonal entry means the matrix is singular to working
    // precision; carrying on would divide by rounding noise and return a solution made of it.
    private const double RelativePivotTolerance = 1e-12;

    // Multiply-adds below one pivot from which the rows are worth sharing out between threads.
    private const long ParallelWork = 32_768;

    /// <summary>Factors <paramref name="matrix"/> in place: its lower triangle becomes L, the upper one is left as it
    /// was. False, with the matrix partly overwritten, when it is not positive definite (to working precision).</summary>
    public static bool TryFactor(double[] matrix, int n)
    {
        CheckSize(matrix, n);
        for (int j = 0; j < n; j++)
        {
            int rowJ = j * n;
            double diagonal = matrix[rowJ + j];
            double pivot = diagonal - Dot(matrix, rowJ, matrix, rowJ, j);
            // Written so that NaN fails too.
            if (!(diagonal > 0) || !(pivot > diagonal * RelativePivotTolerance) || double.IsInfinity(pivot)) return false;
            double l = Math.Sqrt(pivot);
            matrix[rowJ + j] = l;
            double inverse = 1 / l;

            int below = n - j - 1;
            if ((long)below * j >= ParallelWork)
                Parallel.For(j + 1, n, i => Eliminate(matrix, n, i, j, inverse));
            else
                for (int i = j + 1; i < n; i++) Eliminate(matrix, n, i, j, inverse);
        }
        return true;
    }

    /// <summary>Solves L·Lᵀ·x = b in place (<paramref name="rhs"/> becomes x), given the factor from
    /// <see cref="TryFactor"/>.</summary>
    public static void Solve(double[] factor, int n, double[] rhs)
    {
        CheckSize(factor, n);
        if (rhs.Length != n) throw new ArgumentException("The right-hand side does not match the matrix size.", nameof(rhs));

        // L·y = b, row by row.
        for (int i = 0; i < n; i++)
            rhs[i] = (rhs[i] - Dot(factor, i * n, rhs, 0, i)) / factor[i * n + i];

        // Lᵀ·x = y, from the bottom: once x_i is known, its column of Lᵀ (row i of L) is subtracted from the rows above.
        for (int i = n - 1; i >= 0; i--)
        {
            int row = i * n;
            double x = rhs[i] / factor[row + i];
            rhs[i] = x;
            for (int k = 0; k < i; k++) rhs[k] -= factor[row + k] * x;
        }
    }

    /// <summary>Solves A·x = b for a symmetric positive-definite A (only its lower triangle is read), leaving both
    /// inputs untouched.</summary>
    /// <exception cref="ArgumentException">A is not positive definite.</exception>
    public static double[] Solve(double[,] matrix, double[] rhs)
    {
        int n = matrix.GetLength(0);
        if (matrix.GetLength(1) != n) throw new ArgumentException("The matrix is not square.", nameof(matrix));
        if (rhs.Length != n) throw new ArgumentException("The right-hand side does not match the matrix size.", nameof(rhs));

        var factor = new double[n * n];
        for (int i = 0; i < n; i++)
        for (int j = 0; j <= i; j++)
            factor[i * n + j] = matrix[i, j];
        if (!TryFactor(factor, n)) throw new ArgumentException("The matrix is not positive definite.", nameof(matrix));

        var x = (double[])rhs.Clone();
        Solve(factor, n, x);
        return x;
    }

    // L[i,j] = (A[i,j] - Σ_k<j L[i,k]·L[j,k]) / L[j,j]; row i left of column j is already final.
    private static void Eliminate(double[] matrix, int n, int i, int j, double inverse)
    {
        int rowI = i * n;
        matrix[rowI + j] = (matrix[rowI + j] - Dot(matrix, rowI, matrix, j * n, j)) * inverse;
    }

    private static double Dot(double[] a, int aStart, double[] b, int bStart, int length)
    {
        var x = a.AsSpan(aStart, length);
        var y = b.AsSpan(bStart, length);
        double sum = 0;
        int i = 0;
        if (Vector.IsHardwareAccelerated && length >= 2 * Vector<double>.Count)
        {
            var vx = MemoryMarshal.Cast<double, Vector<double>>(x);
            var vy = MemoryMarshal.Cast<double, Vector<double>>(y);
            var accumulator = Vector<double>.Zero;
            for (int k = 0; k < vx.Length; k++) accumulator += vx[k] * vy[k];
            sum = Vector.Sum(accumulator);
            i = vx.Length * Vector<double>.Count;
        }
        for (; i < length; i++) sum += x[i] * y[i];
        return sum;
    }

    private static void CheckSize(double[] matrix, int n)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(n);
        if (matrix.Length != n * n) throw new ArgumentException("The array is not an n×n matrix.", nameof(matrix));
    }
}
