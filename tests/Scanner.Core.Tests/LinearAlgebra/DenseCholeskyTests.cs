using Scanner.Core.LinearAlgebra;

namespace Scanner.Core.Tests.LinearAlgebra;

public class DenseCholeskyTests
{
    [Fact]
    public void Factors_a_known_matrix_into_its_lower_triangle()
    {
        double[] a = [4, 12, -16, 12, 37, -43, -16, -43, 98];

        Assert.True(DenseCholesky.TryFactor(a, 3));

        double[] lower = [2, 6, 1, -8, 5, 3];
        int k = 0;
        for (int i = 0; i < 3; i++)
        for (int j = 0; j <= i; j++)
            Assert.Equal(lower[k++], a[i * 3 + j], 12);
    }

    [Fact]
    public void Reads_only_the_lower_triangle()
    {
        // The upper triangle holds garbage; the lower one is the matrix of the test above.
        double[] a = [4, 99, -99, 12, 37, 1e9, -16, -43, 98];

        Assert.True(DenseCholesky.TryFactor(a, 3));
        Assert.Equal(3, a[8], 12);
    }

    [Fact]
    public void Solves_a_known_system()
    {
        var a = new double[,] { { 4, 12, -16 }, { 12, 37, -43 }, { -16, -43, 98 } };
        double[] x = [1, -2, 0.5];
        var b = Multiply(a, x);

        var solution = DenseCholesky.Solve(a, b);

        for (int i = 0; i < 3; i++) Assert.Equal(x[i], solution[i], 10);
        Assert.Equal(4, a[0, 0]); // the input is left alone
    }

    // 400 unknowns take the parallel path of the factorization.
    [Theory]
    [InlineData(1)]
    [InlineData(7)]
    [InlineData(60)]
    [InlineData(400)]
    public void Solves_a_random_positive_definite_system(int n)
    {
        var random = new Random(n);
        var m = new double[n, n];
        for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            m[i, j] = random.NextDouble() * 2 - 1;
        // M·Mᵀ + I: symmetric, positive definite, and not diagonally dominant.
        var a = new double[n, n];
        for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            double sum = i == j ? 1 : 0;
            for (int k = 0; k < n; k++) sum += m[i, k] * m[j, k];
            a[i, j] = sum;
        }
        var x = Enumerable.Range(0, n).Select(_ => random.NextDouble() * 10 - 5).ToArray();
        var b = Multiply(a, x);

        var solution = DenseCholesky.Solve(a, b);

        double worst = x.Zip(solution, (e, s) => Math.Abs(e - s)).Max();
        Assert.True(worst < 1e-8, $"largest error {worst:E2}");
    }

    [Fact]
    public void Rejects_a_matrix_that_is_not_positive_definite()
    {
        // Indefinite (eigenvalues 3 and -1), singular (eigenvalues 2 and 0), and negative on the diagonal.
        double[] indefinite = [1, 2, 2, 1];
        double[] singular = [1, 1, 1, 1];
        double[] negative = [-1, 0, 0, 1];

        Assert.False(DenseCholesky.TryFactor(indefinite, 2));
        Assert.False(DenseCholesky.TryFactor(singular, 2));
        Assert.False(DenseCholesky.TryFactor(negative, 2));
        Assert.Throws<ArgumentException>(() => DenseCholesky.Solve(new double[,] { { 1, 2 }, { 2, 1 } }, [1, 1]));
    }

    [Fact]
    public void Rejects_a_rank_deficient_matrix_despite_rounding()
    {
        // v·vᵀ + w·wᵀ in 3D has rank 2; rounding leaves a last pivot of about 1e-16 instead of exactly zero.
        double[] v = [0.3, -1.7, 2.9], w = [1.1, 0.4, -0.6];
        var a = new double[9];
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            a[i * 3 + j] = v[i] * v[j] + w[i] * w[j];

        Assert.False(DenseCholesky.TryFactor(a, 3));
    }

    [Fact]
    public void Checks_the_sizes()
    {
        Assert.Throws<ArgumentException>(() => DenseCholesky.TryFactor(new double[8], 3));
        Assert.Throws<ArgumentException>(() => DenseCholesky.Solve(new double[2, 3], [1, 2]));
        Assert.Throws<ArgumentException>(() => DenseCholesky.Solve(new double[2, 2], [1, 2, 3]));
    }

    private static double[] Multiply(double[,] a, double[] x)
    {
        int n = x.Length;
        var b = new double[n];
        for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            b[i] += a[i, j] * x[j];
        return b;
    }
}
