package com.alex.meca500.api;

import com.alex.meca500.kinematics.DHParameter;
import com.alex.meca500.kinematics.KinematicsModel;

/** Minimal in-memory KinematicsModel for testing RobotController without JavaFX. */
final class FakeKinematicsModel implements KinematicsModel {

	private final double[] angles = new double[6];
	private final double[] min = new double[6];
	private final double[] max = new double[6];

	FakeKinematicsModel() {
		for (int i = 0; i < 6; i++) {
			min[i] = Math.toRadians(-175);
			max[i] = Math.toRadians(175);
		}
	}

	@Override public DHParameter[] getDHParameters() {
		DHParameter[] p = new DHParameter[6];
		for (int i = 0; i < 6; i++) p[i] = new DHParameter(0, 0, 0, 0);
		return p;
	}

	@Override public double[] getJointAngles() { return angles.clone(); }

	@Override public void setJointAngle(int i, double angleRad) { angles[i] = angleRad; }

	@Override public double[] getJointMin() { return min; }

	@Override public double[] getJointMax() { return max; }
}
