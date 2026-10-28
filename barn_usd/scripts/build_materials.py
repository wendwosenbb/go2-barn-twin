"""
LIB-01: base material library -> library/materials/library.usda

Root prims /_mat_<Name>, each a UsdPreviewSurface Material. Asset materials
specialize these (design section 7), so a change here reaches every asset
material that did not override that input.

Usage (venv active, from barn_usd/):
    python scripts/build_materials.py
"""
import sys
from pathlib import Path

from pxr import Gf, Sdf, UsdShade

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402

LIBRARY = Path(__file__).resolve().parents[1] / "library"

# name: (diffuse colour, metallic, roughness)
BASES = {
    "Metal": ((0.62, 0.63, 0.64), 1.0, 0.45),     # galvanized steel
    "Plastic": ((0.80, 0.80, 0.78), 0.0, 0.55),
    "Concrete": ((0.55, 0.54, 0.51), 0.0, 0.90),
    "Wood": ((0.55, 0.40, 0.26), 0.0, 0.75),
    "Bedding": ((0.42, 0.30, 0.20), 0.0, 0.95),   # wood-chip litter
}


def define_preview_material(stage, path, color, metallic, roughness):
    """Material + one UsdPreviewSurface shader named 'Shader' (asset overrides target it)."""
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def main():
    path = LIBRARY / "materials" / "library.usda"
    if path.exists():
        path.unlink()
    stage = usd_utils.new_layer(path, default_prim="_mat_Plastic", prim_type="Material")
    for name, (color, metallic, roughness) in BASES.items():
        define_preview_material(stage, Sdf.Path(f"/_mat_{name}"), color, metallic, roughness)
    stage.GetRootLayer().Save()
    print("->", path)


if __name__ == "__main__":
    main()
