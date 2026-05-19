package com.alex.meca500;

import javafx.scene.shape.MeshView;
import org.fxyz3d.importers.stl.StlMeshImporter;

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

        return mesh;
    }
}
