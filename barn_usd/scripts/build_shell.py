"""
BARN-02: the barn shell from survey/barn.yaml -> barns/<barn_id>/layers/shell.usda

/World/Barn/Shell (group): Floor, Wall_YNeg, Wall_YPos, Wall_XNeg, Wall_XPos, Roof_YNeg, Roof_YPos.
Inner faces sit on barn.yaml's extent (the map sees inside faces); walls and roof are
built outward from there. The roof underside runs from wall_height at the long walls
to ridge_height on the centre line, which matches the measured ceiling.
The floor slab is the one prim allowed below Z=0 (barn:allowBelowFloor).

Usage (venv active, from barn_usd/):
    python scripts/build_shell.py barns/farm1_barn1
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdShade, Vt

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402

WALL_T = 0.15      # wall thickness, m (not surveyed yet)
SLAB_T = 0.15      # floor slab thickness below Z=0
ROOF_T = 0.05      # roof panel thickness
OVERHANG = 0.3     # roof overhang past the walls
MATERIALS = "../../../library/materials/library.usda"  # relative to barns/<id>/layers/


def prism_x(yz, x0, x1):
    """Convex polygon in the y-z plane (counter-clockwise seen from +x), extruded x0..x1."""
    yz = np.asarray(yz, float)
    n = len(yz)
    pts = np.vstack([np.c_[np.full(n, x0), yz], np.c_[np.full(n, x1), yz]])
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]  # -x cap faces -x, +x cap faces +x
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return pts, faces


def add_mesh(stage, path, pts, faces, look):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(mesh.GetPointsAttr().Get()))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)
    return mesh.GetPrim()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    args = ap.parse_args()
    by = yaml.safe_load((args.barn_dir / "survey" / "barn.yaml").read_text())
    e, d = by["extent"], by["dimensions"]
    x0, x1, y0, y1 = e["x_min_m"], e["x_max_m"], e["y_min_m"], e["y_max_m"]
    eave, ridge = d["wall_height_m"], d["ridge_height_m"]
    ry = (y0 + y1) / 2                       # ridge on the centre line
    slope = (ridge - eave) / (ry - y0)       # rise per metre toward the ridge

    def underside(y):
        """Roof underside height at y (inner faces at y0/y1 are at eave height)."""
        return ridge - slope * abs(y - ry)

    out = args.barn_dir / "layers" / "shell.usda"
    if out.exists():
        out.unlink()
    stage = usd_utils.new_layer(out)
    Usd.ModelAPI(stage.GetPrimAtPath("/World")).SetKind(Kind.Tokens.assembly)
    Usd.ModelAPI(UsdGeom.Xform.Define(stage, "/World/Barn").GetPrim()).SetKind(Kind.Tokens.assembly)
    shell = UsdGeom.Xform.Define(stage, "/World/Barn/Shell")
    Usd.ModelAPI(shell.GetPrim()).SetKind(Kind.Tokens.group)
    shell.GetPrim().SetCustomDataByKey("barn:source", "survey/barn.yaml")

    # looks: specialize library bases (same pattern as asset mtl.usda files)
    looks_path = Sdf.Path("/World/Barn/Shell/Looks")
    UsdGeom.Scope.Define(stage, looks_path)
    looks = {}
    # walls and roof are painted / coated sheet: keep the Metal base, but not a bare-metal mirror
    for look, base, inputs in (("Floor", "Bedding", {}),
                               ("Wall", "Metal", {"diffuseColor": (0.80, 0.80, 0.78), "metallic": 0.2}),
                               ("Roof", "Metal", {"diffuseColor": (0.70, 0.71, 0.72), "metallic": 0.3})):
        cls = looks_path.AppendChild(f"_mat_{base}")
        if not stage.GetPrimAtPath(cls):
            stage.CreateClassPrim(cls).GetReferences().AddReference(MATERIALS, Sdf.Path(f"/_mat_{base}"))
        mat = stage.DefinePrim(looks_path.AppendChild(look), "Material")
        mat.GetSpecializes().AddSpecialize(cls)
        shader = stage.OverridePrim(mat.GetPath().AppendChild("Shader"))
        for key, value in inputs.items():
            if isinstance(value, tuple):
                shader.CreateAttribute(f"inputs:{key}", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*value))
            else:
                shader.CreateAttribute(f"inputs:{key}", Sdf.ValueTypeNames.Float).Set(value)
        looks[look] = UsdShade.Material(mat)

    sp = shell.GetPath()
    xa, xb = x0 - WALL_T, x1 + WALL_T  # outside faces of the end walls
    ya, yb = y0 - WALL_T, y1 + WALL_T  # outside faces of the long walls

    floor = add_mesh(stage, sp.AppendChild("Floor"),
                     *prism_x([(ya, -SLAB_T), (yb, -SLAB_T), (yb, 0), (ya, 0)], xa, xb), looks["Floor"])
    floor.CreateAttribute("barn:allowBelowFloor", Sdf.ValueTypeNames.Bool, custom=True).Set(True)

    add_mesh(stage, sp.AppendChild("Wall_YNeg"), *prism_x([(ya, 0), (y0, 0), (y0, eave), (ya, eave)], xa, xb), looks["Wall"])
    add_mesh(stage, sp.AppendChild("Wall_YPos"), *prism_x([(y1, 0), (yb, 0), (yb, eave), (y1, eave)], xa, xb), looks["Wall"])
    # gable end walls: up to the roof underside, so they close the space under the roof
    gable = [(ya, 0), (yb, 0), (yb, underside(yb)), (ry, ridge), (ya, underside(ya))]
    add_mesh(stage, sp.AppendChild("Wall_XNeg"), *prism_x(gable, xa, x0), looks["Wall"])
    add_mesh(stage, sp.AppendChild("Wall_XPos"), *prism_x(gable, x1, xb), looks["Wall"])

    # roof panels: underside on the roof line, ROOF_T thick, overhanging the walls
    ra, rb = ya - OVERHANG, yb + OVERHANG
    rx0, rx1 = xa - OVERHANG, xb + OVERHANG
    add_mesh(stage, sp.AppendChild("Roof_YNeg"), *prism_x(
        [(ra, underside(ra)), (ry, ridge), (ry, ridge + ROOF_T), (ra, underside(ra) + ROOF_T)], rx0, rx1), looks["Roof"])
    add_mesh(stage, sp.AppendChild("Roof_YPos"), *prism_x(
        [(ry, ridge), (rb, underside(rb)), (rb, underside(rb) + ROOF_T), (ry, ridge + ROOF_T)], rx0, rx1), looks["Roof"])

    stage.GetRootLayer().Save()
    print(f"-> {out}  ({x1 - x0:.2f} x {y1 - y0:.2f} m, eave {eave} m, ridge {ridge} m at y = {ry:.2f})")


if __name__ == "__main__":
    main()
