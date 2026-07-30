package com.alex.meca500.kinematics;

/**
 * Numerical inverse kinematics via damped least squares (Levenberg-Marquardt).
 *
 * Works for any robot described by DH parameters.
 *
 * Unit scaling: the Jacobian's linear rows (mm/rad) and angular rows (1/rad)
 * have very different magnitudes, making the raw DLS poorly conditioned.
 * We scale orientation error and Jacobian rows by ORI_SCALE (mm/rad) so all
 * components are in consistent mm units before solving.
 *
 * For interactive slider control the solver intentionally runs only a few
 * iterations per call, producing small smooth joint updates rather than
 * jumping to a potentially far-away solution in one shot.
 */
public final class IKSolver {

	// Treats 1 rad of orientation error as equivalent to this many mm of position error.
	// Scales orientation Jacobian rows so J*Jt is in uniform mm² units.
	private static final double ORI_SCALE = 100.0; // mm/rad

	// Damping in mm units (after ORI_SCALE is applied). A value of ~10 mm
	// adds modest regularisation without over-damping near singularities.
	private static final double LAMBDA    = 10.0;

	// Deliberately few iterations per slider event: keeps joint movement small
	// and predictable, preventing configuration jumps when dragging.
	private static final int    MAX_ITER  = 5;

	// Maximum joint change per iteration (~1.1°). Total max per call: 5 × 0.02 ≈ 5.7°.
	private static final double MAX_STEP  = 0.02;  // rad

	private static final double POS_TOL   = 0.1;   // mm
	private static final double ORI_TOL   = 0.001; // rad

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

			// 6D pose error — orientation components scaled to mm units
			double[] e = new double[6];
			e[0] = tgt[0] - curArr[0];
			e[1] = tgt[1] - curArr[1];
			e[2] = tgt[2] - curArr[2];
			e[3] = wrapAngle(tgt[3] - curArr[3]) * ORI_SCALE;
			e[4] = wrapAngle(tgt[4] - curArr[4]) * ORI_SCALE;
			e[5] = wrapAngle(tgt[5] - curArr[5]) * ORI_SCALE;

			double posErr = norm3(e[0], e[1], e[2]);
			double oriErr = norm3(e[3], e[4], e[5]);
			if (posErr < POS_TOL && oriErr < ORI_TOL * ORI_SCALE) break;

			double[][] J = DHKinematics.jacobian(params, q);

			// Scale orientation rows of J to match ORI_SCALE applied to e
			for (int k = 0; k < n; k++) {
				J[3][k] *= ORI_SCALE;
				J[4][k] *= ORI_SCALE;
				J[5][k] *= ORI_SCALE;
			}

			// A = J * Jt + lambda^2 * I_6  (all terms now in mm^2)
			double[][] A = new double[6][6];
			for (int r = 0; r < 6; r++) {
				for (int c = 0; c < 6; c++) {
					double sum = 0;
					for (int k = 0; k < n; k++) sum += J[r][k] * J[c][k];
					A[r][c] = sum;
				}
				A[r][r] += LAMBDA * LAMBDA;
			}

			// y = A^-1 * e,  then  dq = Jt * y
			double[] y  = Transform4x4.solveLU(A, e);
			double[] dq = new double[n];
			for (int i = 0; i < n; i++) {
				for (int r = 0; r < 6; r++) dq[i] += J[r][i] * y[r];
				dq[i] = clamp(dq[i], -MAX_STEP, MAX_STEP);
				q[i]  = clamp(q[i] + dq[i], jointMin[i], jointMax[i]);
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
