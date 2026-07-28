package com.alex.meca500;

import com.alex.meca500.kinematics.DHKinematics;
import com.alex.meca500.kinematics.IKSolver;
import com.alex.meca500.kinematics.TcpPose;

import javafx.application.Application;
import javafx.geometry.Pos;
import javafx.scene.*;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.stage.Stage;

/**
 * Use Launcher to run, or run this class with the JavaFX maven plugin:  mvn javafx:run
 *
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class MainApp {

	public static void main(String[] args) {
		Application.launch(MainFxApp.class, args);
	}

	public static class MainFxApp extends Application {

		private Meca500 robot;

		// Sliders and text fields for joint angles
		private final Slider[]    jointSliders = new Slider[6];
		private final TextField[] jointFields  = new TextField[6];

		// Sliders and text fields for TCP pose: X, Y, Z, Alpha, Beta, Gamma
		private final Slider[]    tcpSliders = new Slider[6];
		private final TextField[] tcpFields  = new TextField[6];

		// Guards against FK→TCP→IK→joint feedback loops
		private boolean updatingFromCode = false;

		@Override
		public void start(Stage stage) {

			Group world = new Group();
			// STLs are in STEP assembly frame (Y-up). Flip to JavaFX Y-down.
			world.getTransforms().add(new Rotate(180, Rotate.Y_AXIS));

			RobotPart[] parts = new RobotPart[7];
			parts[0] = new RobotPart(STLLoader.load("/Link0_base.stl"));
			parts[1] = new RobotPart(STLLoader.load("/Link1.stl"));
			parts[2] = new RobotPart(STLLoader.load("/Link2.stl"));
			parts[3] = new RobotPart(STLLoader.load("/Link3.stl"));
			parts[4] = new RobotPart(STLLoader.load("/Link4.stl"));
			parts[5] = new RobotPart(STLLoader.load("/Link5.stl"));
			parts[6] = new RobotPart(STLLoader.load("/Link6_flange.stl"));

			String[] names = {"link0_base","link1","link2","link3","link4","link5","link6_flange"};
			for (int i = 0; i < parts.length; i++) printBounds(names[i], parts[i].getMesh());

			robot = new Meca500(parts);
			world.getChildren().add(robot.getNode());

			PointLight light = new PointLight();
			light.setTranslateX(500);
			light.setTranslateY(-500);
			light.setTranslateZ(500);
			world.getChildren().add(light);
			world.getChildren().add(new AmbientLight());

			PerspectiveCamera cam = new PerspectiveCamera(true);
			cam.setNearClip(0.1);
			cam.setFarClip(10000);
			cam.setTranslateZ(-1000);

			SubScene subScene = new SubScene(world, 800, 600, true, SceneAntialiasing.BALANCED);
			subScene.setCamera(cam);

			Pane subSceneWrapper = new Pane(subScene);
			subScene.widthProperty().bind(subSceneWrapper.widthProperty());
			subScene.heightProperty().bind(subSceneWrapper.heightProperty());
			VBox.setVgrow(subSceneWrapper, Priority.ALWAYS);

			VBox jointCol = buildJointColumn();
			VBox tcpCol    = buildTcpColumn();

			HBox.setHgrow(jointCol, Priority.ALWAYS);
			HBox.setHgrow(tcpCol,    Priority.ALWAYS);

			HBox bottomPanel = new HBox(8, jointCol, tcpCol);

			// Initialise TCP fields from FK at zero angles (guard prevents spurious IK calls)
			updatingFromCode = true;
			TcpPose initPose = DHKinematics.computeTcpPose(robot.getDHParameters(), new double[6]);
			applyTcpPoseToSliders(initPose);
			updatingFromCode = false;

			VBox root = new VBox(subSceneWrapper, bottomPanel);
			Scene scene = new Scene(root, 900, 800);

			stage.setTitle("Meca500 Simulator");
			stage.getIcons().add(new Image(getClass().getResourceAsStream("/meca500.ico")));
			stage.setScene(scene);
			stage.show();
		}

		// ------------------------------------------------------------------ //
		//  Joint column                                                      //
		// ------------------------------------------------------------------ //
		private VBox buildJointColumn() {
			VBox col = new VBox(4);
			col.getChildren().add(new Label("Joints"));
			for (int i = 0; i < 6; i++) {
				int idx = i;

				Slider s = makeSlider(-180, 180, 0, 30, 5);
				TextField tf = makeTextField();
				jointSliders[idx] = s;
				jointFields[idx]  = tf;

				s.valueProperty().addListener((obs, oldV, newV) -> {
					double deg = newV.doubleValue();
					robot.setJointAngle(idx, Math.toRadians(deg));
					tf.setText(String.format("%.0f", deg));

					if (!updatingFromCode) {
						updatingFromCode = true;
						TcpPose pose = DHKinematics.computeTcpPose(
								robot.getDHParameters(), robot.getJointAngles());
						applyTcpPoseToSliders(pose);
						updatingFromCode = false;
					}
				});

				tf.setOnAction(e -> setSliderFromField(tf, s));

				HBox row = new HBox(8, new Label("Joint " + (idx + 1)), s, tf);
				HBox.setHgrow(s, Priority.ALWAYS);
				col.getChildren().add(row);
			}
			return col;
		}

		// ------------------------------------------------------------------ //
		//  TCP column (X, Y, Z, Alpha, Beta, Gamma)                           //
		// ------------------------------------------------------------------ //

		private static final String[] TCP_LABELS = {"X (mm)", "Y (mm)", "Z (mm)", "Alpha °", "Beta °", "Gamma °"};
		private static final double[] TCP_MIN    = {-300, -300, -600, -180,  -90, -180};
		private static final double[] TCP_MAX    = { 300,  300,  600,  180,   90,  180};
		private static final double[] TCP_TICK   = {  50,   50,   50,   30,   30,   30};

		private VBox buildTcpColumn() {
			VBox col = new VBox(4);
			//TCP Pose refers to the exact position (translation) and orientation (rotation) of a robot's Tool Center Point in 3D space
			col.getChildren().add(new Label("TCP Pose"));
			for (int i = 0; i < 6; i++) {
				int idx = i;

				Slider s = makeSlider(TCP_MIN[i], TCP_MAX[i], 0, TCP_TICK[i], 4);
				TextField tf = makeTextField();
				tcpSliders[idx] = s;
				tcpFields[idx]  = tf;

				s.valueProperty().addListener((obs, oldV, newV) -> {
					tf.setText(String.format("%.1f", newV.doubleValue()));

					if (!updatingFromCode) {
						TcpPose target = readTcpFromSliders();
						double[] newAngles = IKSolver.solve(
								robot.getDHParameters(), target,
								robot.getJointAngles(),
								robot.getJointMin(), robot.getJointMax());

						updatingFromCode = true;
						for (int j = 0; j < 6; j++) {
							double deg = Math.toDegrees(newAngles[j]);
							robot.setJointAngle(j, newAngles[j]);
							jointSliders[j].setValue(deg);
							jointFields[j].setText(String.format("%.0f", deg));
						}
						// Snap TCP sliders to the achievable pose after IK
						TcpPose achieved = DHKinematics.computeTcpPose(
								robot.getDHParameters(), robot.getJointAngles());
						applyTcpPoseToSliders(achieved);
						updatingFromCode = false;
					}
				});

				tf.setOnAction(e -> setSliderFromField(tf, s));

				HBox row = new HBox(8, new Label(TCP_LABELS[idx]), s, tf);
				HBox.setHgrow(s, Priority.ALWAYS);
				col.getChildren().add(row);
			}
			return col;
		}

		// ------------------------------------------------------------------ //
		//  Helpers                                                             //
		// ------------------------------------------------------------------ //

		private void applyTcpPoseToSliders(TcpPose p) {
			double[] vals = { p.x(), p.y(), p.z(), p.alpha(), p.beta(), p.gamma() };
			String[] fmts = { "%.1f","%.1f","%.1f","%.1f","%.1f","%.1f" };
			for (int i = 0; i < 6; i++) {
				tcpSliders[i].setValue(vals[i]);
				tcpFields[i].setText(String.format(fmts[i], vals[i]));
			}
		}

		private TcpPose readTcpFromSliders() {
			return new TcpPose(
				tcpSliders[0].getValue(), tcpSliders[1].getValue(), tcpSliders[2].getValue(),
				tcpSliders[3].getValue(), tcpSliders[4].getValue(), tcpSliders[5].getValue());
		}

		private static Slider makeSlider(double min, double max, double val,
		                                  double majorTick, int minorCount) {
			Slider s = new Slider(min, max, val);
			s.setShowTickMarks(true);
			s.setShowTickLabels(true);
			s.setMajorTickUnit(majorTick);
			s.setMinorTickCount(minorCount);
			return s;
		}

		private static TextField makeTextField() {
			TextField tf = new TextField("0");
			tf.setPrefWidth(70);
			tf.setAlignment(Pos.CENTER);
			return tf;
		}

		private static void setSliderFromField(TextField tf, Slider s) {
			try {
				double val = Double.parseDouble(tf.getText());
				if (val >= s.getMin() && val <= s.getMax()) {
					s.setValue(val);
				} else {
					tf.setText(String.format("%.1f", s.getValue()));
				}
			} catch (NumberFormatException ex) {
				tf.setText(String.format("%.1f", s.getValue()));
			}
		}

		private static void printBounds(String name, MeshView meshView) {
			TriangleMesh mesh = (TriangleMesh) meshView.getMesh();
			float[] pts = mesh.getPoints().toArray(null);
			float minX = Float.MAX_VALUE, minY = Float.MAX_VALUE, minZ = Float.MAX_VALUE;
			float maxX = -Float.MAX_VALUE, maxY = -Float.MAX_VALUE, maxZ = -Float.MAX_VALUE;
			for (int i = 0; i < pts.length; i += 3) {
				minX = Math.min(minX, pts[i]);   maxX = Math.max(maxX, pts[i]);
				minY = Math.min(minY, pts[i+1]); maxY = Math.max(maxY, pts[i+1]);
				minZ = Math.min(minZ, pts[i+2]); maxZ = Math.max(maxZ, pts[i+2]);
			}
			System.out.printf("%-20s  X[%7.1f, %7.1f]  Y[%7.1f, %7.1f]  Z[%7.1f, %7.1f]%n",
					name, minX, maxX, minY, maxY, minZ, maxZ);
		}
	}
}
