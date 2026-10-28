"""
PIPE-05: write one component asset folder the way design section 4 lays it out.

    library/assets/<name>/
        <name>.usda    interface: kind=component, assetInfo, lod variant set, payload
        payload.usda   sublayers mtl.usda over geo.usda (one payload arc loads both)
        geo.usda       /<Root>/Geom/Full/... and /<Root>/Geom/Proxy/..., metres, pivot at origin
        mtl.usda       /<Root>/Looks: materials specializing library/materials bases + bindings
        phys.usda      colliders on the Full parts; NOT referenced by the interface

Parts are plain dicts so asset scripts stay short:
    {"name": "Bowl", "mesh": (points_m (N,3), triangles (M,3)), "look": "Yellow"}
    {"name": "Body", "cylinder": (radius, height), "at": (x, y, z_bottom), "look": "Yellow"}
Looks: {"Yellow": ("Plastic", {"diffuseColor": (0.95, 0.75, 0.05)})}
"""
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

import usd_utils

LIBRARY = Path(__file__).resolve().parents[1] / "library"


def _fresh(path: Path, root: str, prim_type="Xform"):
    if path.exists():
        path.unlink()
    return usd_utils.new_layer(path, default_prim=root, prim_type=prim_type)


def _write_part(stage, parent: Sdf.Path, part):
    path = parent.AppendChild(part["name"])
    if "mesh" in part:
        pts, tris = part["mesh"]
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(tris, np.int32).ravel()))
        # CAD tessellation is already fine: never subdivide it
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        # CAD exports often mix triangle winding; render both sides
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(mesh.GetPointsAttr().Get()))
        return mesh.GetPrim()
    radius, height = part["cylinder"]
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(UsdGeom.Tokens.z)
    cyl.CreateExtentAttr([(-radius, -radius, -height / 2), (radius, radius, height / 2)])
    x, y, z0 = part.get("at", (0, 0, 0))
    # a Cylinder is centred on its origin: lift it by half its height so z0 is its bottom
    UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(x, y, z0 + height / 2))
    return cyl.GetPrim()


def make_asset(name, root, asset_type, full, proxy, looks, info=None):
    """Write library/assets/<name>/ and return the interface path."""
    folder = LIBRARY / "assets" / name
    root_path = Sdf.Path(f"/{root}")
    geom = root_path.AppendChild("Geom")

    # geo.usda: both LODs side by side; the interface's lod variant deactivates one
    geo = _fresh(folder / "geo.usda", root)
    UsdGeom.Scope.Define(geo, geom)
    for lod, parts in (("Full", full), ("Proxy", proxy)):
        UsdGeom.Xform.Define(geo, geom.AppendChild(lod))
        for part in parts:
            _write_part(geo, geom.AppendChild(lod), part)
    geo.GetRootLayer().Save()

    # mtl.usda: bring each needed base in as an abstract class, specialize it, bind
    mtl = _fresh(folder / "mtl.usda", root)
    looks_path = root_path.AppendChild("Looks")
    UsdGeom.Scope.Define(mtl, looks_path)
    lib = "../../materials/library.usda"
    for base in sorted({b for b, _ in looks.values()}):
        cls = mtl.CreateClassPrim(looks_path.AppendChild(f"_mat_{base}"))
        cls.GetReferences().AddReference(lib, Sdf.Path(f"/_mat_{base}"))
    for look, (base, inputs) in looks.items():
        mat = mtl.DefinePrim(looks_path.AppendChild(look), "Material")
        mat.GetSpecializes().AddSpecialize(looks_path.AppendChild(f"_mat_{base}"))
        shader = mtl.OverridePrim(mat.GetPath().AppendChild("Shader"))
        for key, value in inputs.items():
            vtype = Sdf.ValueTypeNames.Color3f if isinstance(value, tuple) else Sdf.ValueTypeNames.Float
            shader.CreateAttribute(f"inputs:{key}", vtype).Set(Gf.Vec3f(*value) if isinstance(value, tuple) else value)
    for lod, parts in (("Full", full), ("Proxy", proxy)):
        for part in parts:
            prim = mtl.OverridePrim(geom.AppendChild(lod).AppendChild(part["name"]))
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material(mtl.GetPrimAtPath(looks_path.AppendChild(part["look"]))))
    # only geo.usda defines the root; set last because DefinePrim re-defines ancestors
    mtl.GetPrimAtPath(root_path).SetSpecifier(Sdf.SpecifierOver)
    mtl.GetRootLayer().Save()

    # payload.usda: just the two sublayers, strongest (mtl) first
    pay_path = folder / "payload.usda"
    if pay_path.exists():
        pay_path.unlink()
    pay = Sdf.Layer.CreateNew(str(pay_path))
    pay.subLayerPaths.append("./mtl.usda")
    pay.subLayerPaths.append("./geo.usda")
    pay.defaultPrim = root  # the payload arc targets this prim
    pay.Save()

    # phys.usda: convex-hull colliders on the Full meshes (pulled in only by layers/physics.usda)
    phys = _fresh(folder / "phys.usda", root)
    phys.GetPrimAtPath(root_path).SetSpecifier(Sdf.SpecifierOver)
    for part in full:
        prim = phys.OverridePrim(geom.AppendChild("Full").AppendChild(part["name"]))
        UsdPhysics.CollisionAPI.Apply(prim)
        if "mesh" in part:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    phys.GetRootLayer().Save()

    # interface: the only file anyone references
    iface = _fresh(folder / f"{name}.usda", root)
    prim = iface.GetPrimAtPath(root_path)
    Usd.ModelAPI(prim).SetKind("component")
    Usd.ModelAPI(prim).SetAssetName(name)
    Usd.ModelAPI(prim).SetAssetVersion("1")
    for key, value in (info or {}).items():
        prim.SetAssetInfoByKey(key, value)
    prim.CreateAttribute("barn:assetType", Sdf.ValueTypeNames.String, custom=True).Set(asset_type)
    prim.GetPayloads().AddPayload("./payload.usda")
    # lod lives in the interface so it can be picked while the payload is unloaded
    vset = prim.GetVariantSets().AddVariantSet("lod")
    for option, off in (("proxy", "Full"), ("full", "Proxy")):
        vset.AddVariant(option)
        vset.SetVariantSelection(option)
        with vset.GetVariantEditContext():
            iface.OverridePrim(geom.AppendChild(off)).SetActive(False)
    vset.SetVariantSelection("full")
    iface.GetRootLayer().Save()
    return folder / f"{name}.usda"
