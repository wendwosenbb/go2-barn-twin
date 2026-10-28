"""
LIB-03: feeder_pan asset from the CAD model (Roxell-style broiler pan).

Input: library/assets/feeder_pan/source/feeder_pan_mesh.npz, made from the STEP with
    freecadcmd scripts/cad_to_mesh.py -- "<.../TF-Broiler Pan1_AllCATPart.stp>" <npz> 0.5
CAD is in mm, Z-up. Pivot = bottom centre of the pan (design section 4), so a pan
placed at z = 0 sits on the litter.

Usage (venv active, from barn_usd/):
    python scripts/build_feeder_pan.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from make_asset import LIBRARY, make_asset  # noqa: E402

# CAD solid label -> (part name, look)
PARTS = {
    "Solid": ("Bowl", "Yellow"),
    "Solid001": ("Rim", "Yellow"),
    "Solid002": ("Grill", "Yellow"),
    "Solid003": ("Clip_1", "Grey"),
    "Solid004": ("Clip_2", "Grey"),
    "Solid005": ("Clip_3", "Grey"),
    "Solid006": ("Clip_4", "Grey"),
    "Solid007": ("Cone", "Yellow"),
}


def main():
    data = np.load(LIBRARY / "assets" / "feeder_pan" / "source" / "feeder_pan_mesh.npz")
    meshes = {str(n): (data[f"s{i:02d}_points"], data[f"s{i:02d}_tris"]) for i, n in enumerate(data["names"])}

    # pivot: centre of the bowl's footprint, at its lowest point
    bowl = meshes["Solid"][0]
    pivot = np.array([(bowl[:, 0].min() + bowl[:, 0].max()) / 2,
                      (bowl[:, 1].min() + bowl[:, 1].max()) / 2,
                      min(p.min(axis=0)[2] for p, _ in meshes.values())])
    full = [{"name": PARTS[label][0], "mesh": ((pts - pivot) * 0.001, tris), "look": PARTS[label][1]}
            for label, (pts, tris) in meshes.items()]
    top = max(p["mesh"][0][:, 2].max() for p in full)

    # proxy: bowl, grill and drop tube as three cylinders (a few hundred triangles)
    proxy = [
        {"name": "Bowl", "cylinder": (0.184, 0.09), "at": (0, 0, 0), "look": "Yellow"},
        {"name": "Grill", "cylinder": (0.15, 0.12), "at": (0, 0, 0.09), "look": "Yellow"},
        {"name": "Tube", "cylinder": (0.035, top - 0.21), "at": (0, 0, 0.21), "look": "Yellow"},
    ]
    looks = {
        "Yellow": ("Plastic", {"diffuseColor": (0.95, 0.72, 0.05), "roughness": 0.45}),
        "Grey": ("Plastic", {"diffuseColor": (0.55, 0.56, 0.58)}),
    }
    out = make_asset("feeder_pan", "FeederPan", "feeder_pan", full, proxy, looks,
                     info={"barn:source": "TF-Broiler Pan1_AllCATPart.stp",
                           "barn:pivot": "bottom centre of the pan"})
    print(f"-> {out}  ({sum(len(p['mesh'][1]) for p in full):,} triangles, {top:.3f} m tall)")


if __name__ == "__main__":
    main()
