from pathlib import Path

from pxr import Usd, UsdGeom

def new_layer(path: Path, default_prim="World", prim_type="Xform")-> Usd.Stage: 

    """
      This helper function creates Usd stage and configure stage level metadatas
      1. up axis, metric unit and default prim.
      2. input argument: str or Path for storing usda, default prim name and type
      3. Returns a stage.  
    """

    # accept str or Path; create the target folder (e.g. a new asset folder) if missing
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    stage: Usd.Stage = Usd.Stage.CreateNew(str(path))

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # create default/root prim and set it to the specified prim type.
    root: Usd.Prim = stage.DefinePrim(f"/{default_prim}", prim_type)

    stage.SetDefaultPrim(root)

    return stage
    