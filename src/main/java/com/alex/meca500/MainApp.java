package com.alex.meca500;

import javafx.scene.control.TextField;

import javafx.application.Application;
import javafx.geometry.Pos;
import javafx.scene.*;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.layout.HBox;
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

		@Override
		public void start(Stage stage) {

			Group world = new Group();
			// STLs are in STEP assembly frame (Y-up). Flip to JavaFX Y-down.
			world.getTransforms().add(new Rotate(180, Rotate.Y_AXIS));

			//load STL meshes here
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

			Meca500 robot = new Meca500(parts);
			world.getChildren().add(robot.getNode());

			//Add light
			PointLight light = new PointLight();
			light.setTranslateX(500);
			light.setTranslateY(-500);
			light.setTranslateZ(500);

			world.getChildren().add(light);

			AmbientLight ambient = new AmbientLight();
			world.getChildren().add(ambient);


			PerspectiveCamera cam = new PerspectiveCamera(true);
			cam.setNearClip(0.1);
			cam.setFarClip(10000);
			cam.setTranslateZ(-1000);

			SubScene subScene = new SubScene(world, 800, 600, true, SceneAntialiasing.BALANCED);
			subScene.setCamera(cam);

			VBox sliders = new VBox(4);

			for (int i = 0; i < 6; i++) {
				int idx = i;
				Slider s = new Slider(-180, 180, 0);
				s.setShowTickMarks(true);
				s.setShowTickLabels(true);
				s.setMajorTickUnit(30);
				s.setMinorTickCount(5);
				s.setSnapToTicks(true);
				TextField textField = new TextField();
				textField.setPrefWidth(60);
				textField.setAlignment(Pos.CENTER);

				s.valueProperty().addListener((observable, oldValue, newValue) -> {
					robot.setJoint(idx, newValue.doubleValue());
					//Synchronize Slider -> TextField
					// Format the value to keep it readable without too many decimals
					textField.setText(String.format("%.0f", newValue));
				});

				//Synchronize TextField -> Slider
				textField.setOnAction(event -> {
					try {
						double val = Double.parseDouble(textField.getText());
						if (val >= s.getMin() && val <= s.getMax()) {
							s.setValue(val);
						} else {
							// Reset to the current slider value if out of bounds
							textField.setText(String.format("%.0f", s.getValue()));
						}
					} catch (NumberFormatException e) {
						// Revert to valid text if user types letters
						textField.setText(String.format("%.0f", s.getValue()));
					}
				});

				Label label = new Label("Joint " + (i + 1));
				sliders.getChildren().add(new HBox(8, label, s, textField));
			}

			VBox root = new VBox(subScene, sliders);

			Scene scene = new Scene(root, 800, 800);

			stage.setTitle("Meca500 Simulator");
			stage.setScene(scene);
			stage.show();
		}

		private static void printBounds(String name, MeshView meshView) {
			TriangleMesh mesh = (TriangleMesh) meshView.getMesh();
			float[] pts = mesh.getPoints().toArray(null);
			float minX = Float.MAX_VALUE;
			float minY = Float.MAX_VALUE;
			float minZ = Float.MAX_VALUE;
			float maxX = -Float.MAX_VALUE;
			float maxY = -Float.MAX_VALUE;
			float maxZ = -Float.MAX_VALUE;
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

