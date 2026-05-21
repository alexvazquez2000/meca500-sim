package com.alex.meca500;

import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
/**
 * Meca500 6-DOF robot arm.
 *
 * STLs are exported from the assembly STEP in world coordinates (Y-up, mm).
 * A Rotate(180, X) on the world Group in MainApp converts to JavaFX Y-down.
 * Pivot coordinates below are in the STEP assembly world frame.
 *
 * Joint axes (in STEP/local frame):
 *   J1 (base swivel)  – Y axis
 *   J2 (shoulder)     – X axis
 *   J3 (elbow)        – X axis
 *   J4 (forearm roll) – Y axis
 *   J5 (wrist pitch)  – X axis
 *   J6 (flange spin)  – Y axis
 *
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class Meca500 {

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

        configureJoint(parts[1], Rotate.Y_AXIS, 15.54f, -4.62f, 0);      // J1 base swivel
        configureJoint(parts[2], Rotate.X_AXIS, 53.04f, 69.38f,    0);      // J2 shoulder
        configureJoint(parts[3], Rotate.X_AXIS,   53.54f, 184.38f,    0);      // J3 elbow
        configureJoint(parts[4], Rotate.X_AXIS,  15.54f, 222.38f,  54.0f);   // J4 forearm roll
        configureJoint(parts[5], Rotate.X_AXIS,   0,      222.0f,  -94.0f);   // J5 wrist pitch
        configureJoint(parts[6], Rotate.Z_AXIS,  15.54f,    0,    -184.6f);   // J6 flange spin
        
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

    public void setJoint(int i, double angle) {
        joints[i].setRotation(angle);
    }
}
