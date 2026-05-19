import FreeCAD
import Mesh
import Part

# Run with:     & "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" convert_step_stl.py

import os

input_dir = "C:/Users/alexv/workspace/meca500-sim/src/main/resources/Meca500-CAD-model"
output_dir = "C:/Users/alexv/workspace/meca500-sim/src/main/resources/"

for file in os.listdir(input_dir):
    if file.lower().endswith(".stp") or file.lower().endswith(".step"):

        path = os.path.join(input_dir, file)

        shape = Part.Shape()
        shape.read(path)

        mesh = Mesh.Mesh(shape.tessellate(0.05))

        out_file = os.path.join(output_dir, file.replace(".stp", ".stl"))
        mesh.write(out_file)

        print(f"Converted: {file}")
