package com.alex.meca500.kinematics;

/**
 * Immutable 4x4 homogeneous transformation matrix.
 * Provides DH link factory, matrix multiply, and a 6x6 LU solver used by IKSolver.
 */
public final class Transform4x4 {

    private final double[][] m; // row-major [4][4]

    private Transform4x4(double[][] m) {
        this.m = m;
    }

    public static Transform4x4 identity() {
        double[][] m = new double[4][4];
        m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0;
        return new Transform4x4(m);
    }

    /**
     * Builds one DH link transform (standard/Craig convention):
     *   T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
     */
    public static Transform4x4 fromDH(double a, double alpha, double d, double theta) {
        double ct = Math.cos(theta), st = Math.sin(theta);
        double ca = Math.cos(alpha), sa = Math.sin(alpha);

        double[][] m = new double[4][4];
        m[0][0] =  ct;      m[0][1] = -st * ca;  m[0][2] =  st * sa;  m[0][3] = a * ct;
        m[1][0] =  st;      m[1][1] =  ct * ca;  m[1][2] = -ct * sa;  m[1][3] = a * st;
        m[2][0] =  0;       m[2][1] =  sa;        m[2][2] =  ca;       m[2][3] = d;
        m[3][0] =  0;       m[3][1] =  0;         m[3][2] =  0;        m[3][3] = 1;
        return new Transform4x4(m);
    }

    public Transform4x4 multiply(Transform4x4 other) {
        double[][] r = new double[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    r[i][j] += m[i][k] * other.m[k][j];
        return new Transform4x4(r);
    }

    /** Returns the translation vector {x, y, z}. */
    public double[] getPosition() {
        return new double[]{ m[0][3], m[1][3], m[2][3] };
    }

    /** Returns the Z-axis of the rotation (3rd column of R). */
    public double[] getZAxis() {
        return new double[]{ m[0][2], m[1][2], m[2][2] };
    }

    /** Returns a 3x3 copy of the rotation submatrix. */
    public double[][] getRotation() {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r[i][j] = m[i][j];
        return r;
    }

    public double get(int row, int col) { return m[row][col]; }

    // -------------------------------------------------------------------------
    // 6x6 Linear system solver (Crout LU with partial pivoting)
    // Used by IKSolver to solve (J*Jt + lambda^2*I) * y = e
    // -------------------------------------------------------------------------

    /**
     * Solves A*x = b for x using LU decomposition with partial pivoting.
     * A is modified in place; b is not modified.
     * n = A.length must equal b.length.
     */
    public static double[] solveLU(double[][] A, double[] b) {
        int n = b.length;
        double[][] a = new double[n][n];
        for (int i = 0; i < n; i++)
            a[i] = A[i].clone();

        int[] piv = new int[n];
        for (int i = 0; i < n; i++) piv[i] = i;

        for (int col = 0; col < n; col++) {
            // Partial pivot
            int maxRow = col;
            double maxVal = Math.abs(a[col][col]);
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(a[row][col]) > maxVal) {
                    maxVal = Math.abs(a[row][col]);
                    maxRow = row;
                }
            }
            double[] tmp = a[col]; a[col] = a[maxRow]; a[maxRow] = tmp;
            int t = piv[col]; piv[col] = piv[maxRow]; piv[maxRow] = t;

            if (Math.abs(a[col][col]) < 1e-14) continue; // singular / near-singular

            for (int row = col + 1; row < n; row++) {
                double factor = a[row][col] / a[col][col];
                for (int k = col; k < n; k++)
                    a[row][k] -= factor * a[col][k];
                a[row][col] = factor; // store L below diagonal
            }
        }

        // Reorder b according to pivots, then forward/back substitution
        double[] bp = new double[n];
        for (int i = 0; i < n; i++) bp[i] = b[piv[i]];

        // Forward substitution (L)
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            y[i] = bp[i];
            for (int j = 0; j < i; j++)
                y[i] -= a[i][j] * y[j];
        }

        // Back substitution (U)
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            x[i] = y[i];
            for (int j = i + 1; j < n; j++)
                x[i] -= a[i][j] * x[j];
            x[i] /= a[i][i];
        }
        return x;
    }
}
