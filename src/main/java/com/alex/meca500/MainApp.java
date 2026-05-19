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

        //load STL meshes here
        RobotPart[] parts = new RobotPart[7];

        parts[0] = new RobotPart(STLLoader.load("/link0_base.stl"));
        parts[1] = new RobotPart(STLLoader.load("/link1.stl"));
        parts[2] = new RobotPart(STLLoader.load("/link2.stl"));
        parts[3] = new RobotPart(STLLoader.load("/link3.stl"));
        parts[4] = new RobotPart(STLLoader.load("/link4.stl"));
        parts[5] = new RobotPart(STLLoader.load("/link5.stl"));
        parts[6] = new RobotPart(STLLoader.load("/link6_flange.stl"));

        Meca500 robot = new Meca500(parts);
        world.getChildren().add(robot.getNode());

        //Add light
        PointLight light = new PointLight();
        light.setTranslateX(500);
        light.setTranslateY(-500);
        light.setTranslateZ(-500);

        world.getChildren().add(light);

        AmbientLight ambient = new AmbientLight();
        world.getChildren().add(ambient);
        
        
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
