package com.alex.meca500;

import javafx.scene.shape.MeshView;
import com.interactivemesh.jfx.importer.stl.StlMeshImporter;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import java.net.URL;

public class STLLoader {

    public static MeshView load(String resourcePath) {
        StlMeshImporter importer = new StlMeshImporter();

        URL url = STLLoader.class.getResource(resourcePath);

        if (url == null) {
            throw new RuntimeException("File not found: " + resourcePath);
        }

        importer.read(url);

        MeshView mesh = new MeshView(importer.getImport());

        importer.close();

        //Add material (otherwise it looks black)
        PhongMaterial mat = new PhongMaterial();
        mat.setDiffuseColor(Color.LIGHTGRAY);
        
        mesh.setMaterial(mat);
        
        return mesh;
    }
}
