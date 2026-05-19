package com.alex.meca500;

import javafx.scene.Group;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Rotate;

/**
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class RobotPart {
    private MeshView mesh;
    private Group node;
    private Rotate rotation;

    public RobotPart(MeshView mesh) {
        this.mesh = mesh;
        this.node = new Group(mesh);

        rotation = new Rotate(0, Rotate.Z_AXIS);
        node.getTransforms().add(rotation);
    }

    public Group getNode() {
        return node;
    }

    public void setRotation(double angle) {
        rotation.setAngle(angle);
    }
}