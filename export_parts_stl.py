"""Export one STL per manufactured part of gripper_mx64_ph42.py.

Each part is taken from the assembly built by gripper_mx64_ph42.gen_step(),
re-oriented for printing / machining (thinnest dimension along +Z, the MX-64
housing keeps Z up but is flipped so its closed top sits on the bed), moved
so its bounding box starts at the origin, and written to stl/<label>.stl.
Identical copies (same label prefix and volume) are written once; the
quantity goes into stl/parts_list.csv.

Purchased / reference parts are skipped: MX-64, PH42 (J6), RealSense D435,
screws, nuts, M4 bolts, dowel pins and the adapter-board envelope.
"""

import csv
from pathlib import Path

from build123d import Axis, Pos, export_stl

import gripper_mx64_ph42 as gripper

OUT = Path(__file__).resolve().parent / "stl"
SKIP_PREFIXES = ("mx64at_ar", "ph42_", "m3_screw", "m2_5_screw", "m4_bolt", "m4_nut", "adapter_", "dowel_pin", "realsense_d435")
LINEAR_TOL = 0.02               # same mesh density as the CAD skill default
ANGULAR_TOL = 0.05


def leaves(shape):
    kids = list(getattr(shape, "children", []) or [])
    if not kids:
        yield shape
        return
    for k in kids:
        yield from leaves(k)


def lay_flat(part, label):
    bb = part.bounding_box()
    size = {"X": bb.size.X, "Y": bb.size.Y, "Z": bb.size.Z}
    if label.startswith(("mx64_housing", "j6_adapter_flange")):
        part = part.rotate(Axis.X, 180)                     # closed top / flat face on the bed (flange pilot boss up)
    elif label.startswith("pivot_bushing"):
        part = part.rotate(Axis.X, 90)                      # bore axis (Y) -> Z, stands on its end
    else:
        thin = min(size, key=size.get)
        if thin == "X":
            part = part.rotate(Axis.Y, 90)
        elif thin == "Y":
            part = part.rotate(Axis.X, 90)
    bb = part.bounding_box()
    return Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * part


def main():
    OUT.mkdir(exist_ok=True)
    groups = {}                                              # (base name, rounded volume) -> [labels]
    shapes = {}
    for leaf in leaves(gripper.gen_step()):
        label = leaf.label or "unnamed"
        if label.startswith(SKIP_PREFIXES):
            continue
        base = label.split(":")[0]
        key = (base, round(leaf.volume, 1))
        if base in ("finger_coupler", "finger_pad"):         # left/right are mirror images: keep both
            key = (label, round(leaf.volume, 1))
        groups.setdefault(key, []).append(label)
        shapes.setdefault(key, leaf)

    rows = []
    for key, labels in groups.items():
        name = labels[0].replace(":", "_") if len(labels) == 1 else key[0] + ("_" + labels[0].split(":")[1] if key[0] == "pivot_bushing" else "")
        if key[0] == "pivot_bushing":
            length = round(shapes[key].bounding_box().size.Y, 1)
            name = f"pivot_bushing_od6_id4.2_len{length}"
        part = lay_flat(shapes[key], labels[0])
        path = OUT / f"{name}.stl"
        export_stl(part, str(path), tolerance=LINEAR_TOL, angular_tolerance=ANGULAR_TOL)
        bb = part.bounding_box()
        rows.append({
            "file": path.name,
            "qty": len(labels),
            "instances": " ".join(labels),
            "size_x_mm": round(bb.size.X, 2),
            "size_y_mm": round(bb.size.Y, 2),
            "size_z_mm": round(bb.size.Z, 2),
            "volume_mm3": round(shapes[key].volume, 1),
        })
    with open(OUT / "parts_list.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['qty']}x {r['file']:<48} {r['size_x_mm']} x {r['size_y_mm']} x {r['size_z_mm']} mm")


if __name__ == "__main__":
    main()
