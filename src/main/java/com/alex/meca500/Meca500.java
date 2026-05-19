package com.alex.meca500;

import javafx.scene.Group;

/**
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class Meca500 {

    private RobotPart[] joints = new RobotPart[6];
    private Group root = new Group();

    public Meca500(RobotPart[] parts) {
        root.getChildren().add(parts[0].getNode()); // base

        parts[0].getNode().getChildren().add(parts[1].getNode());
        parts[1].getNode().getChildren().add(parts[2].getNode());
        parts[2].getNode().getChildren().add(parts[3].getNode());
        parts[3].getNode().getChildren().add(parts[4].getNode());
        parts[4].getNode().getChildren().add(parts[5].getNode());
        parts[5].getNode().getChildren().add(parts[6].getNode());

        for (int i = 0; i < 6; i++) {
            joints[i] = parts[i + 1];
        }
    }

    public Group getNode() {
        return root;
    }

    public void setJoint(int i, double angle) {
        joints[i].setRotation(angle);
    }
}
