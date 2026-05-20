package com.alex.meca500;

import javafx.geometry.Point3D;
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
        this(mesh, Rotate.Z_AXIS, 0, 0, 0);
    }

    public RobotPart(MeshView mesh, Point3D axis, double pivotX, double pivotY, double pivotZ) {
        this.mesh = mesh;
        this.node = new Group(mesh);

        rotation = new Rotate(0, pivotX, pivotY, pivotZ, axis);
        node.getTransforms().add(rotation);
    }

    public MeshView getMesh() {
        return mesh;
    }

    public Group getNode() {
        return node;
    }

    public void setRotation(double angle) {
        rotation.setAngle(angle);
    }
}