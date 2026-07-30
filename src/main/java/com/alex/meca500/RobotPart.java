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
	private float pivotX;
	private float pivotY;
	private float pivotZ;

	public RobotPart(MeshView mesh) {
		this.mesh = mesh;
		this.node = new Group(mesh);
		rotation = new Rotate(0, Rotate.Y_AXIS);
		node.getTransforms().add(rotation);
	}

	/** Called by Meca500 to configure each joint's axis and pivot. */
	public void setJointParams(Point3D axis, float px, float py, float pz) {
		this.pivotX = px;
		this.pivotY = py;
		this.pivotZ = pz;
		node.getTransforms().remove(rotation);
		rotation = new Rotate(0, px, py, pz, axis);
		node.getTransforms().add(rotation);
	}

	public float[] getPivot() {
		return new float[]{pivotX, pivotY, pivotZ};
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
