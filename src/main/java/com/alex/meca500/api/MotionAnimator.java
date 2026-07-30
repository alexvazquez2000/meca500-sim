package com.alex.meca500.api;

import java.util.function.Consumer;

import javafx.animation.AnimationTimer;

import com.alex.meca500.kinematics.DHKinematics;
import com.alex.meca500.kinematics.DHParameter;
import com.alex.meca500.kinematics.IKSolver;
import com.alex.meca500.kinematics.TcpPose;
import com.alex.meca500.kinematics.Transform4x4;

/**
 * Drives at most one animated joint or linear move at a time (no queueing).
 * All public methods must only be called from the JavaFX Application Thread;
 * {@link AnimationTimer#handle} is guaranteed by JavaFX to run there too, so
 * no further thread marshaling happens once a move is started.
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
final class MotionAnimator {

	private static final double JOINT_VELOCITY_DEG_PER_SEC = 90.0;
	private static final long   LIN_MAX_DURATION_NANOS = 5_000_000_000L;
	private static final double LIN_POS_TOL_MM  = 0.5;
	private static final double LIN_ORI_TOL_RAD = 0.01;

	private enum Mode { NONE, JOINTS, LIN }

	private Mode mode = Mode.NONE;
	private boolean paused = false;

	// JOINTS mode state
	private double[] jointsStartRad;
	private double[] jointsTargetRad;
	private long jointsStartNanos;
	private long jointsPausedElapsedNanos;

	// LIN mode state
	private double[] linSeedRad;
	private TcpPose linTarget;
	private DHParameter[] linDh;
	private double[] linJointMin;
	private double[] linJointMax;
	private long linStartNanos;
	private long linPausedElapsedNanos;

	private Consumer<double[]> frameSink;
	private Runnable onDone;
	private AnimationTimer timer;

	boolean isActive() { return mode != Mode.NONE && !paused; }
	boolean isPaused() { return mode != Mode.NONE && paused; }

	void startJointMove(double[] startRad, double[] targetRad, Consumer<double[]> frameSink, Runnable onDone) {
		stopTimerOnly();
		mode = Mode.JOINTS;
		paused = false;
		jointsStartRad = startRad.clone();
		jointsTargetRad = targetRad.clone();
		jointsStartNanos = System.nanoTime();
		jointsPausedElapsedNanos = 0;
		this.frameSink = frameSink;
		this.onDone = onDone;
		startTimer(this::tickJoints);
	}

	void startLinMove(double[] seedRad, TcpPose target, DHParameter[] dh,
			double[] jointMin, double[] jointMax,
			Consumer<double[]> frameSink, Runnable onDone) {
		stopTimerOnly();
		mode = Mode.LIN;
		paused = false;
		linSeedRad = seedRad.clone();
		linTarget = target;
		linDh = dh;
		linJointMin = jointMin;
		linJointMax = jointMax;
		linStartNanos = System.nanoTime();
		linPausedElapsedNanos = 0;
		this.frameSink = frameSink;
		this.onDone = onDone;
		startTimer(this::tickLin);
	}

	/** Freezes the active move in place. No-op if nothing is active. */
	void pause() {
		if (mode == Mode.NONE || paused) return;
		paused = true;
		if (mode == Mode.JOINTS) jointsPausedElapsedNanos = System.nanoTime() - jointsStartNanos;
		else linPausedElapsedNanos = System.nanoTime() - linStartNanos;
		stopTimerOnly();
	}

	/** Continues a paused move toward its original target. No-op if nothing is paused. */
	void resume() {
		if (mode == Mode.NONE || !paused) return;
		paused = false;
		if (mode == Mode.JOINTS) {
			jointsStartNanos = System.nanoTime() - jointsPausedElapsedNanos;
			startTimer(this::tickJoints);
		} else {
			// LIN mode is memoryless per solve() call -- resuming with the same
			// seed just keeps iterating toward the same target with no jump.
			linStartNanos = System.nanoTime() - linPausedElapsedNanos;
			startTimer(this::tickLin);
		}
	}

	private void startTimer(Runnable frameHandler) {
		timer = new AnimationTimer() {
			@Override public void handle(long now) { frameHandler.run(); }
		};
		timer.start();
	}

	private void stopTimerOnly() {
		if (timer != null) { timer.stop(); timer = null; }
	}

	private void finish() {
		stopTimerOnly();
		mode = Mode.NONE;
		paused = false;
		Runnable done = onDone;
		onDone = null;
		frameSink = null;
		if (done != null) done.run();
	}

	private void tickJoints() {
		long elapsedNanos = System.nanoTime() - jointsStartNanos;
		double elapsedSec = elapsedNanos / 1_000_000_000.0;

		double maxDeltaDeg = 0;
		double[] deltaRad = new double[jointsStartRad.length];
		for (int i = 0; i < deltaRad.length; i++) {
			deltaRad[i] = jointsTargetRad[i] - jointsStartRad[i];
			maxDeltaDeg = Math.max(maxDeltaDeg, Math.abs(Math.toDegrees(deltaRad[i])));
		}

		double durationSec = maxDeltaDeg / JOINT_VELOCITY_DEG_PER_SEC;
		double frac = durationSec <= 1e-9 ? 1.0 : Math.min(1.0, elapsedSec / durationSec);

		double[] anglesRad = new double[jointsStartRad.length];
		for (int i = 0; i < anglesRad.length; i++) {
			anglesRad[i] = jointsStartRad[i] + deltaRad[i] * frac;
		}
		frameSink.accept(anglesRad);

		if (frac >= 1.0) finish();
	}

	private void tickLin() {
		long elapsedNanos = System.nanoTime() - linStartNanos;

		double[] next = IKSolver.solve(linDh, linTarget, linSeedRad, linJointMin, linJointMax);
		linSeedRad = next;
		frameSink.accept(next);

		Transform4x4 t = DHKinematics.forwardKinematics(linDh, next);
		double[] pos = t.getPosition();
		double[] tgt = linTarget.toArray();
		double dx = tgt[0] - pos[0], dy = tgt[1] - pos[1], dz = tgt[2] - pos[2];
		double posErr = Math.sqrt(dx * dx + dy * dy + dz * dz);

		// Geodesic rotation angle between target and achieved orientation, via the
		// standard trace formula -- exact for any angle (not just small ones) and
		// has none of the gimbal-lock/branch-cut issues Euler-angle differencing has.
		double[][] rTgt = linTarget.toRotationMatrix();
		double[][] rCur = t.getRotation();
		double trace = 0;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				trace += rTgt[i][j] * rCur[i][j];
		double cosAngle = Math.max(-1.0, Math.min(1.0, (trace - 1.0) / 2.0));
		double oriErr = Math.acos(cosAngle);

		boolean converged = posErr < LIN_POS_TOL_MM && oriErr < LIN_ORI_TOL_RAD;
		boolean timedOut = elapsedNanos >= LIN_MAX_DURATION_NANOS;
		if (converged || timedOut) finish();
	}

}
