package com.alex.meca500.kinematics;

/**
 * Robot-agnostic interface for a serial manipulator described by DH parameters.
 * Implement this for each robot (Meca500, UR30, etc.) to share FK/IK logic.
 */
public interface KinematicsModel {

	/** DH parameters in joint order (index 0 = joint 1). */
	DHParameter[] getDHParameters();

	/** Current joint angles in radians, index 0 = joint 1. */
	double[] getJointAngles();

	/** Sets joint i (0-based) in radians and updates the visual model. */
	void setJointAngle(int i, double angleRad);

	/** Per-joint lower limits in radians. */
	double[] getJointMin();

	/** Per-joint upper limits in radians. */
	double[] getJointMax();

	default int getDoF() {
		return getDHParameters().length;
	}

}
