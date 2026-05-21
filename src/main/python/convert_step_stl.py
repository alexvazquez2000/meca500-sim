import FreeCAD
import Part
import Mesh
import os

# Run with:     & "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" convert_step_stl.py
#
# Loads the ASSEMBLY STEP file so every link is exported in assembly-world coordinates.
# All STLs share the same world frame, so at zero-angle all meshes naturally sit in
# the correct assembled position with no positional offsets needed in the simulator.

assembly_path = "C:/Users/alexv/workspace/meca500-sim/src/main/resources/Meca500-CAD-model/Meca500 assembled in zero robot position.stp"
output_dir    = "C:/Users/alexv/workspace/meca500-sim/src/main/resources/"

# Maps part index (after flattening the compound tree) to output filename.
# part_07 is the mounting fixture — omitted intentionally.
index_to_name = {
    0: "Link0_base.stl",
    1: "Link1.stl",
    2: "Link2.stl",
    3: "Link3.stl",
    4: "Link4.stl",
    5: "Link5.stl",
    6: "Link6_flange.stl",
}

def collect_parts(shape, results=None):
    if results is None:
        results = []
    for s in shape.SubShapes:
        if s.ShapeType == "Solid":
            results.append(s)
        elif s.ShapeType == "Compound":
            collect_parts(s, results)
    return results

compound = Part.Shape()
compound.read(assembly_path)
parts = collect_parts(compound)
print(f"Found {len(parts)} parts\n")

for i, part in enumerate(parts):
    bbox = part.BoundBox
    name = index_to_name.get(i, f"part_{i:02d}_UNMAPPED.stl")
    out_path = os.path.join(output_dir, name)
    print(f"  [{i:02d}]  X[{bbox.XMin:8.2f}, {bbox.XMax:8.2f}]"
          f"  Y[{bbox.YMin:8.2f}, {bbox.YMax:8.2f}]"
          f"  Z[{bbox.ZMin:8.2f}, {bbox.ZMax:8.2f}]"
          f"  -> {name}")
    mesh = Mesh.Mesh(part.tessellate(0.05))
    mesh.write(out_path)

print(f"\nDone. Files written to {output_dir}")
