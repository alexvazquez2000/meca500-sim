package com.alex.meca500;

import javafx.application.Application;
import javafx.scene.*;
import javafx.scene.control.Slider;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

/**
 * Use Launcher to run, or run this class with the JavaFX maven plugin:  mvn javafx:run
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public class MainApp extends Application {

    @Override
    public void start(Stage stage) {

        Group world = new Group();

        // TODO: load STL meshes here
        RobotPart[] parts = new RobotPart[6];

        for (int i = 0; i < 6; i++) {
          //  parts[i] = new RobotPart(new javafx.scene.shape.Box(50,50,50)); // placeholder
        }

        Meca500 robot = new Meca500(parts);
        world.getChildren().add(robot.getNode());

        PerspectiveCamera cam = new PerspectiveCamera(true);
        cam.setTranslateZ(-500);

        SubScene subScene = new SubScene(world, 800, 600, true, SceneAntialiasing.BALANCED);
        subScene.setCamera(cam);

        VBox sliders = new VBox();

        for (int i = 0; i < 6; i++) {
            int idx = i;
            Slider s = new Slider(-180, 180, 0);
            s.valueProperty().addListener((obs, oldVal, newVal) -> {
                robot.setJoint(idx, newVal.doubleValue());
            });
            sliders.getChildren().add(s);
        }

        VBox root = new VBox(subScene, sliders);

        Scene scene = new Scene(root, 800, 800);

        stage.setTitle("Meca500 Simulator");
        stage.setScene(scene);
        stage.show();
    }

    public static void main(String[] args) {
        launch();
    }
}
