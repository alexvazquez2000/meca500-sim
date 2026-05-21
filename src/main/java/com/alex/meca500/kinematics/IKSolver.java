package com.alex.meca500.kinematics;

/**
 * Numerical inverse kinematics via damped least squares (Levenberg-Marquardt).
 *
 * Works for any robot described by DH parameters.
 * The 6D error vector is [dx, dy, dz (mm), dAlpha, dBeta, dGamma (rad)].
 */
public final class IKSolver {

    private static final double LAMBDA    = 0.05;  // damping factor
    private static final int    MAX_ITER  = 50;
    private static final double POS_TOL   = 0.1;   // mm
    private static final double ORI_TOL   = 0.001; // rad (~0.057 deg)
    private static final double MAX_STEP  = 0.1;   // rad per joint per iteration

    private IKSolver() {}

    /**
     * Solves IK for the given target pose.
     *
     * @param params      DH parameters of the robot
     * @param target      desired TCP pose
     * @param seedAngles  starting joint angles (radians); not modified
     * @param jointMin    per-joint lower limits (radians)
     * @param jointMax    per-joint upper limits (radians)
     * @return            best-effort joint angles in radians
     */
    public static double[] solve(DHParameter[] params, TcpPose target,
                                 double[] seedAngles,
                                 double[] jointMin, double[] jointMax) {
        int n = params.length;
        double[] q = seedAngles.clone();
        double[] tgt = target.toArray(); // {x, y, z, alphaRad, betaRad, gammaRad}

        for (int iter = 0; iter < MAX_ITER; iter++) {
            TcpPose cur = DHKinematics.computeTcpPose(params, q);
            double[] curArr = cur.toArray();

            // 6D pose error
            double[] e = new double[6];
            e[0] = tgt[0] - curArr[0];
            e[1] = tgt[1] - curArr[1];
            e[2] = tgt[2] - curArr[2];
            e[3] = wrapAngle(tgt[3] - curArr[3]);
            e[4] = wrapAngle(tgt[4] - curArr[4]);
            e[5] = wrapAngle(tgt[5] - curArr[5]);

            double posErr = norm3(e[0], e[1], e[2]);
            double oriErr = norm3(e[3], e[4], e[5]);
            if (posErr < POS_TOL && oriErr < ORI_TOL) break;

            double[][] J = DHKinematics.jacobian(params, q);

            // A = J * Jt + lambda^2 * I_6
            double[][] A = new double[6][6];
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++) {
                    double sum = 0;
                    for (int k = 0; k < n; k++) sum += J[r][k] * J[c][k];
                    A[r][c] = sum;
                }
                A[r][r] += LAMBDA * LAMBDA;
            }

            // y = (J * Jt + lambda^2 * I)^-1 * e
            double[] y = Transform4x4.solveLU(A, e);

            // dq = Jt * y
            double[] dq = new double[n];
            for (int i = 0; i < n; i++) {
                for (int r = 0; r < 6; r++) dq[i] += J[r][i] * y[r];
                dq[i] = clamp(dq[i], -MAX_STEP, MAX_STEP);
                q[i] = clamp(q[i] + dq[i], jointMin[i], jointMax[i]);
            }
        }
        return q;
    }

    private static double wrapAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double norm3(double a, double b, double c) {
        return Math.sqrt(a * a + b * b + c * c);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
