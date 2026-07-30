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

		// Create PhongMaterial for aluminum (otherwise it looks black)
		PhongMaterial aluminumMaterial = new PhongMaterial();

		// Diffuse color: base silver/gray color of the aluminum
		//aluminumMaterial.setDiffuseColor(Color.web("#d1d5db"));
		aluminumMaterial.setDiffuseColor(Color.web("#71757b"));

		// Specular color: bright highlight (aluminum is a conductor and reflects light as its own color)
		aluminumMaterial.setSpecularColor(Color.web("#e5e7eb")); 

		// Specular power: high value yields a sharp, polished look (try lower values for a matte/brushed finish)
		aluminumMaterial.setSpecularPower(64);

		mesh.setMaterial(aluminumMaterial);

		return mesh;
	}

}
