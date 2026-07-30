package com.alex.meca500.kinematics;

/**
 * End-effector pose: position (mm) and ZYX Euler angles (degrees).
 *
 * Convention (ZYX, extrinsic / Tait-Bryan):
 *   alpha = rotation about Z (yaw)
 *   beta  = rotation about Y (pitch)
 *   gamma = rotation about X (roll)
 *
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public record TcpPose(double x, double y, double z,
		double alpha, double beta, double gamma) {

	/** Returns {x, y, z, alphaRad, betaRad, gammaRad}. */
	public double[] toArray() {
		return new double[]{
				x, y, z,
				Math.toRadians(alpha),
				Math.toRadians(beta),
				Math.toRadians(gamma)
		};
	}

	/** Builds R = Rz(alpha) * Ry(beta) * Rx(gamma), the inverse of {@link #fromMatrix}'s ZYX extraction. */
	public double[][] toRotationMatrix() {
		double a = Math.toRadians(alpha), b = Math.toRadians(beta), g = Math.toRadians(gamma);
		double ca = Math.cos(a), sa = Math.sin(a);
		double cb = Math.cos(b), sb = Math.sin(b);
		double cg = Math.cos(g), sg = Math.sin(g);
		return new double[][] {
			{ ca * cb,  -sa * cg + ca * sb * sg,   sa * sg + ca * sb * cg },
			{ sa * cb,   ca * cg + sa * sb * sg,  -ca * sg + sa * sb * cg },
			{ -sb,       cb * sg,                  cb * cg                }
		};
	}

	/**
	 * Extracts position and ZYX Euler angles from a homogeneous transform.
	 * Handles the gimbal-lock singularity (beta = ±90°).
	 */
	public static TcpPose fromMatrix(Transform4x4 t) {
		double[] pos = t.getPosition();
		double[][] R = t.getRotation();

		double beta = Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0]));

		double alpha, gamma;
		if (Math.abs(Math.cos(beta)) < 1e-6) {
			// Gimbal lock: set gamma = 0, solve alpha
			gamma = 0;
			alpha = Math.atan2(-R[0][1], R[1][1]);
		} else {
			alpha = Math.atan2(R[1][0], R[0][0]);
			gamma = Math.atan2(R[2][1], R[2][2]);
		}

		return new TcpPose(
				pos[0], pos[1], pos[2],
				Math.toDegrees(alpha),
				Math.toDegrees(beta),
				Math.toDegrees(gamma)
				);
	}

}
