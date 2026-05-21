package com.alex.meca500;

import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

/**
 * Meca500 6-DOF robot arm.
 *
 * Pivot coordinates are in each link's own STL coordinate frame.
 * The Y axis is vertical (positive Y = down in JavaFX / physically down).
 * The arm hangs in the -Y direction (physically upward) at zero position.
 *
 * Joint axes:
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

        parts[4].getMesh().getTransforms().add(new Rotate(180, Rotate.Y_AXIS)); // STL exported 180° off around Y

        parts[5].getMesh().getTransforms().add(new Rotate(180, Rotate.Y_AXIS)); // STL exported 180° off around Y


        // --- Joint parameters: (axis, pivotX, pivotY, pivotZ) ---
        // Pivot is in each link's own STL local coordinate frame.
        // Y-axis rotations: only pivotX and pivotZ matter (pivotY is free).
        // X-axis rotations: only pivotY and pivotZ matter (pivotX is free).

//don't delete yet, this were the values that Claude first gave for the pivots - figure out how to get better values
        //The STEP files show the pivots at J1=(15.54, -4.62, 0), J2=(53.04, 49.38,0), J3=(53.54, 184.38, 0) J4=(15.54, 222.38, -54.0)
//        configureJoint(parts[1], Rotate.Y_AXIS,    0,       0,     0);      // J1 base swivel
//        configureJoint(parts[2], Rotate.X_AXIS,    0,     -17.4f, -32.0f);   // J2 shoulder
//        configureJoint(parts[3], Rotate.X_AXIS,    0,    -144.1f,   0);      // J3 elbow
//        configureJoint(parts[4], Rotate.X_AXIS,    0,     -33.0f,   0);      // J4 forearm roll (pivot Y negated: mesh exported 180° around Z)
//        configureJoint(parts[5], Rotate.X_AXIS,    0,    -199.9f, -143.75f); // J5 wrist pitch
//        configureJoint(parts[6], Rotate.Z_AXIS,   76.4f,  51.5f, -164.2f);  // J6 flange spin

        configureJoint(parts[1], Rotate.Y_AXIS,    -12f,       3.7f,      -10f);      // J1 base swivel
        parts[1].setJointParams(Rotate.X_AXIS, 0, 0, 0);
        parts[1].getMesh().getTransforms().add(new Rotate(-90, Rotate.X_AXIS));
        parts[1].getMesh().getTransforms().add(new Rotate(90, Rotate.Z_AXIS));
        parts[1].getMesh().getTransforms().add(new Translate(0, 14.7, -48.5));

        configureJoint(parts[2], Rotate.X_AXIS,    0,     -17.4f, -32.0f);   // J2 shoulder
        configureJoint(parts[3], Rotate.X_AXIS,    0,    -144.1f,   0);      // J3 elbow
        configureJoint(parts[4], Rotate.X_AXIS,    0,     -33.0f,   0);      // J4 forearm roll (pivot Y negated: mesh exported 180° around Z)
        configureJoint(parts[5], Rotate.X_AXIS,    0,    -199.9f, -143.75f); // J5 wrist pitch
        
        float tx = -86.4f;
        float ty = -270.0f;
        float tz = -20.0f; 
        configureJoint(parts[6], Rotate.Z_AXIS,   tx + 76.4f,  ty + 51.5f, tz + -164.2f);  // J6 flange spin
        parts[6].getMesh().getTransforms().add(new Translate(tx, ty, tz));
        
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
