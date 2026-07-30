package com.alex.meca500.kinematics;

/**
 * Stateless forward kinematics and geometric Jacobian for any DH-parameterised robot.
 * All methods are pure functions — no state, no JavaFX dependency.
 */
public final class DHKinematics {

	private DHKinematics() {}

	/**
	 * Computes the end-effector transform T_0_n.
	 *
	 * @param params DH parameters (length = n joints)
	 * @param angles joint angles in radians (length = n joints)
	 */
	public static Transform4x4 forwardKinematics(DHParameter[] params, double[] angles) {
		Transform4x4 t = Transform4x4.identity();
		for (int i = 0; i < params.length; i++) {
			DHParameter p = params[i];
			t = t.multiply(Transform4x4.fromDH(p.a(), p.alpha(), p.d(), angles[i] + p.thetaOffset()));
		}
		return t;
	}

	/**
	 * Computes partial FK chain: T_all[0] = identity (base), T_all[i] = T_0_i.
	 * Length of the returned array is params.length + 1.
	 * Used by the Jacobian to avoid redundant recalculation.
	 */
	public static Transform4x4[] forwardKinematicsAll(DHParameter[] params, double[] angles) {
		Transform4x4[] chain = new Transform4x4[params.length + 1];
		chain[0] = Transform4x4.identity();
		for (int i = 0; i < params.length; i++) {
			DHParameter p = params[i];
			chain[i + 1] = chain[i].multiply(
					Transform4x4.fromDH(p.a(), p.alpha(), p.d(), angles[i] + p.thetaOffset())
					);
		}
		return chain;
	}

	/**
	 * Computes the geometric Jacobian as a 6×n matrix (double[6][n]).
	 * Rows 0-2: linear velocity contribution of each joint.
	 * Rows 3-5: angular velocity contribution.
	 *
	 * For revolute joint i:
	 *   J_v_i = z_{i-1} × (p_e - p_{i-1})
	 *   J_ω_i = z_{i-1}
	 */
	public static double[][] jacobian(DHParameter[] params, double[] angles) {
		int n = params.length;
		Transform4x4[] chain = forwardKinematicsAll(params, angles);
		double[] pe = chain[n].getPosition();

		double[][] J = new double[6][n];
		for (int i = 0; i < n; i++) {
			double[] z = chain[i].getZAxis();
			double[] p = chain[i].getPosition();

			// dp = p_e - p_{i-1}
			double dx = pe[0] - p[0];
			double dy = pe[1] - p[1];
			double dz = pe[2] - p[2];

			// cross product z × dp
			J[0][i] = z[1] * dz - z[2] * dy;
			J[1][i] = z[2] * dx - z[0] * dz;
			J[2][i] = z[0] * dy - z[1] * dx;

			J[3][i] = z[0];
			J[4][i] = z[1];
			J[5][i] = z[2];
		}
		return J;
	}

	/** Convenience: compute TcpPose from joint angles. */
	public static TcpPose computeTcpPose(DHParameter[] params, double[] angles) {
		return TcpPose.fromMatrix(forwardKinematics(params, angles));
	}

}
