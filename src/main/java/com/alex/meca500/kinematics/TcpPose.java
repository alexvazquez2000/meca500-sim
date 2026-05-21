package com.alex.meca500.kinematics;

/**
 * End-effector pose: position (mm) and ZYX Euler angles (degrees).
 *
 * Convention (ZYX, extrinsic / Tait-Bryan):
 *   alpha = rotation about Z (yaw)
 *   beta  = rotation about Y (pitch)
 *   gamma = rotation about X (roll)
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
