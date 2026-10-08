"""Export link-frame meshes and mass properties for gripper_mx64_ph42.urdf.

Writes urdf_meshes/<part>.stl (millimetres, link frame) and
urdf_meshes/mass_properties.json (SI units) from gripper_mx64_ph42.py.

Link frames (all axes parallel to the gripper frame at crank angle 0):
  gripper_base_link  origin = J6 output flange centre, +Z toward the fingertips
  <side>_crank_link  origin = gear / crank pivot A on the pivot-bolt axis (y = 0)
  <side>_idle_link   origin = idle-link base pivot B
  <side>_finger_link origin = coupler pivot A' (crank tip at crank angle 0)

Densities (assumptions, g/mm^3): aluminium 6061 for plates, flange, links,
fingers and gears; PETG for the printed housing; rubber for the pads; the
MX-64 uses its 135 g catalogue mass and the adapter board an assumed 20 g.
"""

import json
from pathlib import Path

from build123d import Pos, export_stl, mirror, Plane
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

import gripper_mx64_ph42 as g

OUT = Path(__file__).resolve().parent / "urdf_meshes"
ALUMINIUM = 0.0027
PETG = 0.00127
RUBBER = 0.0012
MX64_MASS_G = 135.0             # ROBOTIS MX-64AT/AR catalogue mass
BOARD_MASS_G = 20.0             # assumed adapter / junction board mass
D435_MASS_G = 75.0              # RealSense D435 / D435i datasheet mass
MESH_TOL = 0.05
MESH_ANG_TOL = 0.1


def gprops(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, p)
    return p


def link_mass_props(parts):
    """parts: list of (shape, density g/mm^3). Returns SI mass, CoM and inertia about CoM."""
    total = GProp_GProps()
    for shape, density in parts:
        total.Add(gprops(shape), density)
    c = total.CentreOfMass()
    m = total.MatrixOfInertia()                      # g*mm^2 about the centre of mass
    gmm2_to_kgm2 = 1e-9
    return {
        "mass": total.Mass() / 1000.0,
        "com": [c.X() / 1000.0, c.Y() / 1000.0, c.Z() / 1000.0],
        "ixx": m.Value(1, 1) * gmm2_to_kgm2, "iyy": m.Value(2, 2) * gmm2_to_kgm2, "izz": m.Value(3, 3) * gmm2_to_kgm2,
        "ixy": m.Value(1, 2) * gmm2_to_kgm2, "ixz": m.Value(1, 3) * gmm2_to_kgm2, "iyz": m.Value(2, 3) * gmm2_to_kgm2,
    }


def to_frame(shape, origin_xz):
    return Pos(-origin_xz[0], 0, -origin_xz[1]) * shape


def main():
    OUT.mkdir(exist_ok=True)
    theta0 = 0.0
    a_tip_l = (g.A_L[0], g.A_L[1] + g.crank_len)
    a_tip_r = (g.A_R[0], g.A_R[1] + g.crank_len)

    # ---- static parts, already in the gripper_base_link frame
    servo = g.make_mx64()
    pcb, comp, _ = g.make_adapter_board()
    cam = g.make_d435()
    base_parts = {
        "j6_adapter_flange": (g.make_j6_flange(), ALUMINIUM),
        "base_plate": (g.make_base_plate(), ALUMINIUM),
        "back_plate": (g.make_back_plate(), ALUMINIUM),
        "front_plate": (g.make_front_plate(), ALUMINIUM),
        "mx64_housing": (g.make_mx64_housing(), PETG),
        "mx64at_ar": (servo, MX64_MASS_G / servo.volume),
        "adapter_board": (pcb + comp, BOARD_MASS_G / (pcb.volume + comp.volume)),
        "camera_bracket": (g.make_camera_bracket(), ALUMINIUM),
        "realsense_d435": (cam, D435_MASS_G / cam.volume),
    }

    # ---- moving parts at crank angle 0, moved into their link frames
    phase_r = g.pick_gear_phase(theta0)[1]
    finger_l, pad_l, _, _ = g.make_finger_left(theta0)
    links = {
        "left_crank_link": {"drive_gear_crank_left": (to_frame(g.make_gear_crank(-1, theta0, 0.0), g.A_L), ALUMINIUM)},
        "right_crank_link": {"driven_gear_crank_right": (to_frame(g.make_gear_crank(1, theta0, phase_r), g.A_R), ALUMINIUM)},
        "left_idle_link": {"idle_link_left": (to_frame(g.make_idle_link(-1, theta0), g.B_L), ALUMINIUM)},
        "right_idle_link": {"idle_link_right": (to_frame(g.make_idle_link(1, theta0), g.B_R), ALUMINIUM)},
        "left_finger_link": {
            "finger_coupler_left": (to_frame(finger_l, a_tip_l), ALUMINIUM),
            "finger_pad_left": (to_frame(pad_l, a_tip_l), RUBBER),
        },
        "right_finger_link": {
            "finger_coupler_right": (to_frame(mirror(finger_l, Plane.YZ), a_tip_r), ALUMINIUM),
            "finger_pad_right": (to_frame(mirror(pad_l, Plane.YZ), a_tip_r), RUBBER),
        },
        "gripper_base_link": base_parts,
    }

    report = {}
    for link, parts in links.items():
        for name, (shape, _) in parts.items():
            export_stl(shape, str(OUT / f"{name}.stl"), tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL)
        report[link] = {"meshes": list(parts), **link_mass_props(list(parts.values()))}
        print(f"{link:<20} mass {report[link]['mass'] * 1000:7.1f} g  meshes {list(parts)}")
    (OUT / "mass_properties.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
