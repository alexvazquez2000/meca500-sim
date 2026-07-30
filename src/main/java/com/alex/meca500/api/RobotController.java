package com.alex.meca500.api;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import javafx.application.Platform;

import com.alex.meca500.kinematics.DHKinematics;
import com.alex.meca500.kinematics.KinematicsModel;
import com.alex.meca500.kinematics.TcpPose;

/**
 * Thread-safety boundary between REST/HTTP handler threads and the JavaFX
 * Application Thread. Status flags and the joint-angle snapshot are plain
 * atomics safe to read from any thread; anything that touches the robot
 * model or scene graph is marshaled onto the FX thread via Platform.runLater.
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public final class RobotController {

	private final KinematicsModel robot;
	private final MotionAnimator animator = new MotionAnimator();
	private final Consumer<double[]> jointFrameSink;
	private final Consumer<Boolean> connectionUiSink;

	private final AtomicReference<double[]> anglesSnapshotRad;

	private final AtomicBoolean connected = new AtomicBoolean(false);
	private final AtomicBoolean activated = new AtomicBoolean(false);
	private final AtomicBoolean homed     = new AtomicBoolean(false);
	private final AtomicBoolean paused    = new AtomicBoolean(false);
	private final AtomicBoolean moving    = new AtomicBoolean(false);

	/**
	 * @param jointFrameSink   applies a new set of joint angles (radians) to the robot/UI; must only be invoked on the FX thread
	 * @param connectionUiSink updates the on-screen connection indicator; must only be invoked on the FX thread
	 */
	public RobotController(KinematicsModel robot, Consumer<double[]> jointFrameSink, Consumer<Boolean> connectionUiSink) {
		this.robot = robot;
		this.jointFrameSink = jointFrameSink;
		this.connectionUiSink = connectionUiSink;
		this.anglesSnapshotRad = new AtomicReference<>(robot.getJointAngles());
	}

	// ------------------------------------------------------------------ //
	// Connection                                                          //
	// ------------------------------------------------------------------ //

	public void connect() {
		connected.set(true);
		Platform.runLater(() -> connectionUiSink.accept(true));
	}

	public void disconnect() {
		connected.set(false);
		Platform.runLater(() -> connectionUiSink.accept(false));
	}

	// ------------------------------------------------------------------ //
	// Activation                                                          //
	// ------------------------------------------------------------------ //

	public void activateAndHome() {
		activated.set(true);
		homed.set(true);
	}

	public void deactivateRobot() {
		activated.set(false);
		homed.set(false);
	}

	/** Blocks the calling thread, polling until deactivated or the (capped) timeout elapses. Returns the final deactivated state. */
	public boolean waitDeactivated(long timeoutMs) {
		long cappedMs = Math.max(0, Math.min(timeoutMs, 30_000));
		long deadline = System.currentTimeMillis() + cappedMs;
		while (activated.get() && System.currentTimeMillis() < deadline) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				break;
			}
		}
		return !activated.get();
	}

	// ------------------------------------------------------------------ //
	// Motion                                                              //
	// ------------------------------------------------------------------ //

	public enum MoveOutcome { ACCEPTED, NOT_READY, BUSY, OUT_OF_RANGE }

	public MoveOutcome moveJoints(double[] targetDeg) {
		if (!activated.get() || !homed.get()) return MoveOutcome.NOT_READY;
		if (moving.get() || paused.get()) return MoveOutcome.BUSY;

		double[] min = robot.getJointMin();
		double[] max = robot.getJointMax();
		double[] targetRad = new double[6];
		for (int i = 0; i < 6; i++) {
			double rad = Math.toRadians(targetDeg[i]);
			if (rad < min[i] || rad > max[i]) return MoveOutcome.OUT_OF_RANGE;
			targetRad[i] = rad;
		}

		moving.set(true);
		Platform.runLater(() -> {
			double[] startRad = robot.getJointAngles();
			animator.startJointMove(startRad, targetRad, this::onFrame, this::onMotionDone);
		});
		return MoveOutcome.ACCEPTED;
	}

	public MoveOutcome moveLin(TcpPose target) {
		if (!activated.get() || !homed.get()) return MoveOutcome.NOT_READY;
		if (moving.get() || paused.get()) return MoveOutcome.BUSY;

		moving.set(true);
		Platform.runLater(() -> {
			double[] seedRad = robot.getJointAngles();
			animator.startLinMove(seedRad, target, robot.getDHParameters(),
					robot.getJointMin(), robot.getJointMax(), this::onFrame, this::onMotionDone);
		});
		return MoveOutcome.ACCEPTED;
	}

	/** Freezes the active move in place. Idempotent no-op if nothing is currently moving. */
	public void clearMotion() {
		if (!moving.get()) return;
		Platform.runLater(() -> {
			if (animator.isActive()) {
				animator.pause();
				moving.set(false);
				paused.set(true);
			}
		});
	}

	/** Continues a paused move toward its original target. Idempotent no-op if nothing is paused. */
	public void resumeMotion() {
		if (!paused.get()) return;
		Platform.runLater(() -> {
			if (animator.isPaused()) {
				animator.resume();
				paused.set(false);
				moving.set(true);
			}
		});
	}

	/** Runs on the FX thread once per animation frame. */
	private void onFrame(double[] anglesRad) {
		jointFrameSink.accept(anglesRad);
		anglesSnapshotRad.set(anglesRad.clone());
	}

	/** Runs on the FX thread when a move completes (converged or timed out). */
	private void onMotionDone() {
		moving.set(false);
		paused.set(false);
	}

	// ------------------------------------------------------------------ //
	// Status / reads -- safe from any thread                             //
	// ------------------------------------------------------------------ //

	public record StatusSnapshot(boolean connected, boolean activated, boolean homed,
			boolean simulation, boolean paused, boolean moving, boolean eom, boolean error) {}

	public StatusSnapshot getStatus() {
		boolean isMoving = moving.get();
		return new StatusSnapshot(
				connected.get(), activated.get(), homed.get(),
				true, paused.get(), isMoving, !isMoving, false);
	}

	public double[] getJointsDeg() {
		double[] rad = anglesSnapshotRad.get();
		double[] deg = new double[rad.length];
		for (int i = 0; i < rad.length; i++) deg[i] = Math.toDegrees(rad[i]);
		return deg;
	}

	public TcpPose getPose() {
		return DHKinematics.computeTcpPose(robot.getDHParameters(), anglesSnapshotRad.get());
	}

}
