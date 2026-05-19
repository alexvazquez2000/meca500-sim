package com.alex.meca500;

import javafx.scene.Group;

/**
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class Meca500 {

    private RobotPart[] joints = new RobotPart[6];
    private Group root = new Group();

    public Meca500(RobotPart[] parts) {
        for (int i = 0; i < 6; i++) {
            joints[i] = parts[i];
        }

        // Simple hierarchy (refine later)
        root.getChildren().add(parts[0].getNode());
        for (int i = 1; i < 6; i++) {
            parts[i-1].getNode().getChildren().add(parts[i].getNode());
        }
    }

    public Group getNode() {
        return root;
    }

    public void setJoint(int i, double angle) {
        joints[i].setRotation(angle);
    }
}
