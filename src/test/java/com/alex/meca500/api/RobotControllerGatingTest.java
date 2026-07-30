package com.alex.meca500.api;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import com.alex.meca500.kinematics.TcpPose;

/**
 * Exercises only the synchronous gating/validation logic in RobotController --
 * the paths that return before scheduling any Platform.runLater work -- so
 * these tests don't require the JavaFX toolkit to be initialized.
 */
class RobotControllerGatingTest {

	private RobotController newController() {
		return new RobotController(new FakeKinematicsModel(), angles -> {}, connected -> {});
	}

	@Test
	void moveJointsRejectedWhenNotActivated() {
		RobotController c = newController();
		assertEquals(RobotController.MoveOutcome.NOT_READY,
				c.moveJoints(new double[]{0, 0, 0, 0, 0, 0}));
	}

	@Test
	void moveLinRejectedWhenNotActivated() {
		RobotController c = newController();
		assertEquals(RobotController.MoveOutcome.NOT_READY,
				c.moveLin(new TcpPose(0, 0, 0, 0, 0, 0)));
	}

	@Test
	void statusReflectsActivateAndDeactivate() {
		RobotController c = newController();
		assertFalse(c.getStatus().activated());

		c.activateAndHome();
		assertTrue(c.getStatus().activated());
		assertTrue(c.getStatus().homed());

		c.deactivateRobot();
		assertFalse(c.getStatus().activated());
		assertFalse(c.getStatus().homed());
	}

	@Test
	void moveJointsOutOfRangeRejectedEvenWhenActivated() {
		RobotController c = newController();
		c.activateAndHome();
		assertEquals(RobotController.MoveOutcome.OUT_OF_RANGE,
				c.moveJoints(new double[]{200, 0, 0, 0, 0, 0}));
	}

	@Test
	void clearMotionIsNoOpWhenIdle() {
		RobotController c = newController();
		c.clearMotion(); // must not throw even without the FX toolkit running
		assertFalse(c.getStatus().paused());
	}

	@Test
	void resumeMotionIsNoOpWhenNothingPaused() {
		RobotController c = newController();
		c.resumeMotion(); // must not throw even without the FX toolkit running
		assertFalse(c.getStatus().paused());
	}

	@Test
	void waitDeactivatedReturnsImmediatelyWhenAlreadyDeactivated() {
		RobotController c = newController();
		long start = System.currentTimeMillis();
		boolean result = c.waitDeactivated(5000);
		long elapsed = System.currentTimeMillis() - start;
		assertTrue(result);
		assertTrue(elapsed < 1000, "should not block when already deactivated");
	}

	@Test
	void jointsAndPoseReadableWithoutFxToolkit() {
		RobotController c = newController();
		assertEquals(6, c.getJointsDeg().length);
		assertNotNull(c.getPose());
	}
}
