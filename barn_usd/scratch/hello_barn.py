# Import the OpenUSD Python modules we need
import sys 
from pathlib import Path

from pxr import UsdGeom, Sdf, Gf

# UsdGeom = geometry schemas: Xform, Cube, Cylinder, Mesh, transform ops
# Sdf     = low-level layer/file layer: paths, value type names
# Gf      = math types: vectors, matrices, quaternions

# scratch/ isn't next to scripts/, so add barn_usd/scripts/ to Python's import path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

# Shared helpers (new_layer) used by every barn_usd authoring script
import usd_utils

# Output file sits next to this script, wherever the script is run from
barn = Path(__file__).resolve().parent / "hello_barn.usda"

# new_layer creates the file, sets Z-up and meters, and defines /World as the default prim
stage = usd_utils.new_layer(barn)

# Define a Cube prim for the barn floor (a "def Cube" in the file)
floor = UsdGeom.Cube.Define(stage, "/World/Floor")

# Set the cube's built-in size attribute to 1 m, so scale below equals real dimensions
floor.GetSizeAttr().Set(1.0)

# Add a scale transform op: 150 m long (X), 15 m wide (Y), 0.1 m thick (Z)
UsdGeom.XformCommonAPI(floor).SetScale(Gf.Vec3f(150.0, 15.0, 0.1))

# Add a translate op: drop it 5 cm so the floor's top surface sits at Z = 0
UsdGeom.XformCommonAPI(floor).SetTranslate(Gf.Vec3d(0, 0, -0.05))

# Define a Cylinder prim as a stand-in feeder pan
feeder = UsdGeom.Cylinder.Define(stage, "/World/Feeder_00")

# Author the cylinder's radius attribute: 0.2 m (Create* authors an opinion; Get* only fetches the handle)
feeder.CreateRadiusAttr(0.2)

# Author the cylinder's height attribute: 0.3 m (cylinder axis defaults to Z)
feeder.CreateHeightAttr(0.3)

# Lift the feeder by half its height so its base rests on the floor (cylinder origin is its center)
UsdGeom.XformCommonAPI(feeder).SetTranslate(Gf.Vec3d(0, 0, 0.15))

# Create a custom attribute "barn:lineIndex" of type int — your own data, namespaced with "barn:"
line = feeder.GetPrim().CreateAttribute("barn:lineIndex", Sdf.ValueTypeNames.Int)

# Set the custom attribute's default value: this feeder belongs to feeder line 0
line.Set(0)

# Write the stage's root layer to disk (hello_barn.usda)
stage.GetRootLayer().Save()

# Print the root layer as .usda text so you can read exactly what was authored
print(stage.GetRootLayer().ExportToString())