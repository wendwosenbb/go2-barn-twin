"""
LIB-05 / LIB-06 (+ the line part of BARN-03): feeder and drinker lines with their
pans and nipples, from the barn survey -> barns/<barn_id>/layers/lines.usda

  survey/equipment.csv   one row per line: start (x, y), yaw, length, height
  survey/line_items.csv  per line: item asset, pitch, first offset, count, pattern

Each line is an Xform at its start, rotated by its yaw, so items sit at (s, 0, z)
along local +x. Every pan/nipple is an instanceable reference (design section 9):
a real prim the robot can target (FEEDLINE_01/Pan_037), sharing one prototype.

Usage (venv active, from barn_usd/):
    python scripts/build_lines.py barns/farm1_barn1
"""
import argparse
import csv
import sys
from pathlib import Path

from pxr import Gf, Kind, Sdf, Usd, UsdGeom

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402

# per line type: group prim, item prim prefix, item asset, pipe radius, pipe centre height
LINE_TYPES = {
    # feed tube centre measured at 0.25 m in the scan; pans sit on the litter (z = 0)
    "feeder_line": ("FeederLines", "Pan", "feeder_pan", 0.0225, lambda z: 0.25),
    # drinker pipe runs through the nipple clamp, 0.1225 m above the cup bottom (CAD)
    "drinker_line": ("DrinkerLines", "Nipple", "drinker_nipple", 0.011, lambda z: z + 0.1225),
}
# CAD axis that each item's pipe clamp runs along -> yaw that turns it onto local +x
ITEM_YAW = {"feeder_pan": 90.0, "drinker_nipple": 0.0}


def set_xform(prim, translate, yaw_deg=0.0):
    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(*translate))
    if yaw_deg:
        api.SetRotate(Gf.Vec3f(0, 0, yaw_deg))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    args = ap.parse_args()
    survey = args.barn_dir / "survey"
    lines = {r["id"]: r for r in csv.DictReader(open(survey / "equipment.csv")) if r["asset_type"] in LINE_TYPES}
    items = {r["line_id"]: r for r in csv.DictReader(open(survey / "line_items.csv"))}

    out = args.barn_dir / "layers" / "lines.usda"
    if out.exists():
        out.unlink()
    stage = usd_utils.new_layer(out)
    Usd.ModelAPI(stage.GetPrimAtPath("/World")).SetKind(Kind.Tokens.assembly)
    for path, kind in (("/World/Barn", Kind.Tokens.assembly), ("/World/Barn/Equipment", Kind.Tokens.group)):
        Usd.ModelAPI(UsdGeom.Xform.Define(stage, path).GetPrim()).SetKind(kind)
    library = Path("../../../library/assets")  # relative to barns/<id>/layers/

    for line_id, row in sorted(lines.items()):
        group, prefix, asset, pipe_r, pipe_z = LINE_TYPES[row["asset_type"]]
        group_path = Sdf.Path(f"/World/Barn/Equipment/{group}")
        if not stage.GetPrimAtPath(group_path):
            Usd.ModelAPI(UsdGeom.Xform.Define(stage, group_path).GetPrim()).SetKind(Kind.Tokens.group)
        length, z = float(row["length_m"]), float(row["z_m"])
        line = UsdGeom.Xform.Define(stage, group_path.AppendChild(line_id))
        Usd.ModelAPI(line.GetPrim()).SetKind(Kind.Tokens.group)
        set_xform(line, (float(row["x_m"]), float(row["y_m"]), 0.0), float(row["yaw_deg"]))
        line.GetPrim().CreateAttribute("barn:assetType", Sdf.ValueTypeNames.String, custom=True).Set(row["asset_type"])

        # the pipe: a cylinder along local x from s = 0 to s = length
        pipe = UsdGeom.Cylinder.Define(stage, line.GetPath().AppendChild("Pipe"))
        pipe.CreateAxisAttr(UsdGeom.Tokens.x)
        pipe.CreateRadiusAttr(pipe_r)
        pipe.CreateHeightAttr(length)
        pipe.CreateExtentAttr([(-length / 2, -pipe_r, -pipe_r), (length / 2, pipe_r, pipe_r)])
        pipe.CreateDisplayColorAttr([(0.6, 0.61, 0.62)])
        set_xform(pipe, (length / 2, 0.0, pipe_z(z)))

        spec = items.get(line_id)
        if spec is None:
            continue
        pitch, first, count = float(spec["pitch_m"]), float(spec["first_offset_m"]), int(spec["count"])
        for prim, value in (("barn:itemPitchM", pitch), ("barn:itemFirstOffsetM", first)):
            line.GetPrim().CreateAttribute(prim, Sdf.ValueTypeNames.Double, custom=True).Set(value)
        ref = str(library / spec["item_type"] / f"{spec['item_type']}.usda")
        for i in range(count):
            item = stage.DefinePrim(line.GetPath().AppendChild(f"{prefix}_{i + 1:03d}"))
            item.GetReferences().AddReference(ref)
            item.SetInstanceable(True)
            # "alternate": every second nipple turned around, so its cup arm points the other way
            flip = 180.0 if spec["pattern"] == "alternate" and i % 2 else 0.0
            set_xform(item, (first + i * pitch, 0.0, z), ITEM_YAW[spec["item_type"]] + flip)
        print(f"  {line_id}: {count} x {spec['item_type']} every {pitch} m")

    stage.GetRootLayer().Save()
    print("->", out)


if __name__ == "__main__":
    main()
