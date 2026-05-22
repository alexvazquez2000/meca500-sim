package com.alex.meca500;

import com.alex.meca500.kinematics.DHParameter;
import com.alex.meca500.kinematics.KinematicsModel;

import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;

/**
 * Meca500 6-DOF robot arm — visual model and kinematics model.
 *
 * STLs are exported from the assembly STEP in world coordinates (Y-up, mm).
 * A Rotate(180, Y) on the world Group in MainApp converts to JavaFX Y-down.
 * Pivot coordinates below are in the STEP assembly world frame.
 *
 * DH kinematics are expressed in the robot's mathematical frame (Z-up).
 *
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class Meca500 implements KinematicsModel {

    // Meca500 R3 standard DH parameters: a (mm), alpha (rad), d (mm), theta_offset (rad)
    private static final DHParameter[] DH_PARAMS = {
        new DHParameter(  0,  Math.PI / 2,  135, 0),
        new DHParameter(210,           0,    0, 0),
        new DHParameter( 75,  Math.PI / 2,   0, 0),
        new DHParameter(  0, -Math.PI / 2,  210, 0),
        new DHParameter(  0,  Math.PI / 2,   0, 0),
        new DHParameter(  0,           0,   60, 0)
    };

    private static final double LIM = Math.toRadians(175);
    private static final double[] JOINT_MIN = { -LIM, -LIM, -LIM, -LIM, -LIM, -LIM };
    private static final double[] JOINT_MAX = {  LIM,  LIM,  LIM,  LIM,  LIM,  LIM };

    private final double[] currentAngles = new double[6]; // radians

    // Set to true to show colored spheres at each joint pivot for debugging
    public static final boolean SHOW_PIVOTS = true;

    private static final Color[] PIVOT_COLORS = {
        Color.RED, Color.ORANGE, Color.YELLOW,
        Color.GREEN, Color.CYAN, Color.MAGENTA
    };

    private RobotPart[] joints = new RobotPart[6];
    private Group root = new Group();

    public Meca500(RobotPart[] parts) {

        // --- Joint parameters: (axis, pivotX, pivotY, pivotZ) ---
        // Pivots are in STEP assembly world coordinates (mm, Y-up).
        // For Y-axis joints: only pivotX and pivotZ matter (pivotY is free).
        // For X-axis joints: only pivotY and pivotZ matter (pivotX is free).
        // All values are rough estimates from bounding-box analysis — tune with debug spheres.

        configureJoint(parts[1], Rotate.Y_AXIS, 15.54f,  -4.37f, 0);      // J1 base swivel
        configureJoint(parts[2], Rotate.X_AXIS, 15.54f,   -49.38f,   0);      // J2 shoulder (x=(48.04 -16.96)/2 = 15.54
        configureJoint(parts[3], Rotate.X_AXIS, 15.54f, -184.38f,    0);      // J3 elbow (x=(47.54 - 16.46)/2=
        configureJoint(parts[4], Rotate.Z_AXIS, 15.54f, -222.38f,  (61.0f + 62.5f)/2.0f );   // J4 forearm roll
        configureJoint(parts[5], Rotate.X_AXIS, 15.54f, -222.38f,  119.0f);   // J5 wrist pitch
        configureJoint(parts[6], Rotate.Z_AXIS, 15.54f, -222.38f,    184.6f);   // J6 flange spin (the join is at 179.0 but End is 184)
        
        // Build parent-child hierarchy
        root.getChildren().add(parts[0].getNode());
        parts[0].getNode().getChildren().add(parts[1].getNode());
        parts[1].getNode().getChildren().add(parts[2].getNode());
        parts[2].getNode().getChildren().add(parts[3].getNode());
        parts[3].getNode().getChildren().add(parts[4].getNode());
        parts[4].getNode().getChildren().add(parts[5].getNode());
        parts[5].getNode().getChildren().add(parts[6].getNode());

        for (int i = 0; i < 6; i++) {
            joints[i] = parts[i + 1];
        }

        if (SHOW_PIVOTS) addPivotMarkers(parts);
    }

    private void configureJoint(RobotPart part, Point3D axis,
                                 float px, float py, float pz) {
        part.setJointParams(axis, px, py, pz);
    }

    private void addPivotMarkers(RobotPart[] parts) {
        for (int i = 0; i < 6; i++) {
            Sphere s = new Sphere(4);
            PhongMaterial mat = new PhongMaterial(PIVOT_COLORS[i]);
            s.setMaterial(mat);
            // Place the sphere at the pivot position inside the link's local frame
            float[] pivot = parts[i + 1].getPivot();
            s.setTranslateX(pivot[0]);
            s.setTranslateY(pivot[1]);
            s.setTranslateZ(pivot[2]);
            parts[i + 1].getNode().getChildren().add(s);
        }
    }

    public Group getNode() {
        return root;
    }

    /** Sets joint i (0-based) in degrees and updates the visual model. */
    public void setJoint(int i, double angle) {
        joints[i].setRotation(angle);
    }

    // --- KinematicsModel ---

    @Override
    public DHParameter[] getDHParameters() { return DH_PARAMS; }

    @Override
    public double[] getJointAngles() { return currentAngles.clone(); }

    @Override
    public void setJointAngle(int i, double angleRad) {
        currentAngles[i] = angleRad;
        setJoint(i, Math.toDegrees(angleRad));
    }

    @Override
    public double[] getJointMin() { return JOINT_MIN; }

    @Override
    public double[] getJointMax() { return JOINT_MAX; }
}
