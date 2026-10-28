"""
Tessellate a STEP file into triangle meshes, one per solid -> .npz.

Runs inside FreeCAD's Python (no pxr there); make_asset.py then reads the .npz
into USD. Units stay as in the CAD file (usually mm); the asset builder scales.

Usage:
    freecadcmd scripts/cad_to_mesh.py -- <in.step> <out.npz> [linear_deflection_mm]
"""
import sys

import FreeCAD  # noqa: F401  (provided by freecadcmd)
import Import
import MeshPart
import numpy as np

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[-2:]
src, out = args[0], args[1]
deflection = float(args[2]) if len(args) > 2 else 0.5

doc = FreeCAD.newDocument("cad")
Import.insert(src, doc.Name)

arrays = {}
names = []
for obj in doc.Objects:
    # leaf objects with exactly one solid; assemblies/compounds repeat their children
    if not hasattr(obj, "Shape") or obj.Shape.isNull() or len(obj.Shape.Solids) != 1:
        continue
    mesh = MeshPart.meshFromShape(Shape=obj.Shape, LinearDeflection=deflection,
                                  AngularDeflection=0.35, Relative=False)
    pts = np.array([[p.x, p.y, p.z] for p in mesh.Points], dtype=np.float64)
    tris = np.array([f.PointIndices for f in mesh.Facets], dtype=np.int32)
    key = "s%02d" % len(names)
    arrays[key + "_points"], arrays[key + "_tris"] = pts, tris
    names.append(obj.Label)
    print("%-20s %6d points %6d triangles" % (obj.Label, len(pts), len(tris)))

np.savez_compressed(out, names=np.array(names), **arrays)
print("->", out)
