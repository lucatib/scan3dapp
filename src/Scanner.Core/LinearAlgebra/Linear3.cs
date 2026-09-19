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
