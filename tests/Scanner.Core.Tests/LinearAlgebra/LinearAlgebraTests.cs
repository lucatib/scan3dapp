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
