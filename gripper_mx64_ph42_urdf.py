"""URDF generator for the MX-64 parallel-linkage gear gripper (gripper_mx64_ph42).

Design ledger
-------------
Units: metres, kilograms, radians. Meshes are millimetre STLs -> scale 0.001.
Meshes and mass properties come from export_urdf_meshes.py (rerun it after any
CAD change, then regenerate this URDF).

Frames (all link frames are parallel to gripper_base_link at crank angle 0):
  gripper_base_link  origin = centre of the J6 (PH42) output flange face.
                     +Z toward the fingertips, X = finger opening direction,
                     Y = pivot-bolt direction. Attach it to the robot with a
                     fixed joint from the J6 output link.
  left/right_crank_link   origin on the gear / crank pivot A (MX-64 horn axis on the left)
  left/right_idle_link    origin on the idle-link base pivot B
  left/right_finger_link  origin on the coupler pivot A' (crank tip)
  grip_center_link        fixed, between the pads at crank angle 0 (z = 172 mm);
                          the true grip centre drops along an arc as the jaws open.

Joints:
  left_crank_joint   actuated (MX-64). Positive = jaws open. Axis -Y, because the
                     left crank swings outward (-X) when the MX-64 turns it.
  right_crank_joint  mimic left 1:1, axis +Y (meshing gears turn opposite ways)
  *_idle_joint       mimic left 1:1, same axis as the crank on that side
  *_finger_joint     mimic left 1:1, axis opposite to its crank so the finger keeps
                     its orientation (parallelogram). URDF is a tree, so the idle
                     links are not tied to the fingers; the mimic joints reproduce
                     the closed-loop motion exactly.
  Range -13.7 deg (pads touch) .. 35 deg (89 mm opening).
  Effort / velocity: MX-64 at 12 V, 6.0 N*m stall, 63 rev/min.
"""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
MESH_DIR = "urdf_meshes"                     # relative to the generated .urdf
MESH_SCALE_FROM_MM = "0.001 0.001 0.001"
MM = 0.001

# geometry (mm) - mirrors gripper_mx64_ph42.py
GEAR_PITCH_R_MM = 21.0
IDLE_OFFSET_MM = 37.0
GEAR_AXIS_Z_MM = 61.0
CRANK_LEN_MM = 55.0
GRIP_CENTER_Z_MM = GEAR_AXIS_Z_MM + CRANK_LEN_MM + 84.0 - 3.0 - 50.0 / 2   # pad centre at crank angle 0

A_L = (-GEAR_PITCH_R_MM, GEAR_AXIS_Z_MM)
A_R = (GEAR_PITCH_R_MM, GEAR_AXIS_Z_MM)
B_L = (-GEAR_PITCH_R_MM - IDLE_OFFSET_MM, GEAR_AXIS_Z_MM)
B_R = (GEAR_PITCH_R_MM + IDLE_OFFSET_MM, GEAR_AXIS_Z_MM)

CRANK_LOWER_RAD = math.radians(-13.7)
CRANK_UPPER_RAD = math.radians(35.0)
MX64_STALL_TORQUE_NM = 6.0
MX64_NO_LOAD_SPEED_RAD_S = 63.0 * 2 * math.pi / 60.0

AXIS_POS_Y = (0.0, 1.0, 0.0)
AXIS_NEG_Y = (0.0, -1.0, 0.0)

COLORS = {
    "aluminium": (0.72, 0.74, 0.78, 1.0),
    "brass": (0.85, 0.65, 0.25, 1.0),
    "blue": (0.2, 0.35, 0.6, 1.0),
    "rubber": (0.1, 0.1, 0.1, 1.0),
    "dark": (0.18, 0.18, 0.2, 1.0),
    "pcb": (0.1, 0.45, 0.2, 1.0),
}
MESH_MATERIAL = {
    "j6_adapter_flange": "aluminium", "base_plate": "aluminium", "back_plate": "aluminium",
    "front_plate": "aluminium", "mx64_housing": "aluminium", "mx64at_ar": "dark", "adapter_board": "pcb",
    "drive_gear_crank_left": "brass", "driven_gear_crank_right": "brass",
    "idle_link_left": "aluminium", "idle_link_right": "aluminium",
    "finger_coupler_left": "blue", "finger_coupler_right": "blue",
    "finger_pad_left": "rubber", "finger_pad_right": "rubber",
}
NO_COLLISION = {"mx64at_ar", "adapter_board"}   # fully inside the housing, which carries the collision


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def add_link(robot, name, props):
    link = ET.SubElement(robot, "link", {"name": name})
    if props is None:
        return link
    for mesh in props["meshes"]:
        for tag in ("visual", "collision"):
            if tag == "collision" and mesh in NO_COLLISION:
                continue
            el = ET.SubElement(link, tag, {"name": f"{mesh}_{tag}"})
            ET.SubElement(el, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geom = ET.SubElement(el, "geometry")
            ET.SubElement(geom, "mesh", {"filename": f"{MESH_DIR}/{mesh}.stl", "scale": MESH_SCALE_FROM_MM})
            if tag == "visual":
                ET.SubElement(el, "material", {"name": MESH_MATERIAL[mesh]})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": fmt(props["com"]), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{props['mass']:.6g}"})
    ET.SubElement(inertial, "inertia", {k: f"{props[k]:.6g}" for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")})
    return link


def add_joint(robot, name, jtype, parent, child, xyz_mm, axis=None, mimic=None):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": jtype})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": fmt(v * MM for v in xyz_mm), "rpy": "0 0 0"})
    if jtype == "revolute":
        ET.SubElement(joint, "axis", {"xyz": fmt(axis)})
        ET.SubElement(joint, "limit", {
            "lower": f"{CRANK_LOWER_RAD:.6f}", "upper": f"{CRANK_UPPER_RAD:.6f}",
            "effort": f"{MX64_STALL_TORQUE_NM}", "velocity": f"{MX64_NO_LOAD_SPEED_RAD_S:.4f}",
        })
        if mimic:
            ET.SubElement(joint, "mimic", {"joint": mimic, "multiplier": "1", "offset": "0"})
    return joint


def gen_urdf():
    props = json.loads((HERE / MESH_DIR / "mass_properties.json").read_text())
    robot = ET.Element("robot", {"name": "gripper_mx64_ph42"})
    for name, rgba in COLORS.items():
        mat = ET.SubElement(robot, "material", {"name": name})
        ET.SubElement(mat, "color", {"rgba": fmt(rgba)})

    for link in ("gripper_base_link", "left_crank_link", "right_crank_link", "left_idle_link",
                 "right_idle_link", "left_finger_link", "right_finger_link"):
        add_link(robot, link, props[link])
    add_link(robot, "grip_center_link", None)

    add_joint(robot, "left_crank_joint", "revolute", "gripper_base_link", "left_crank_link",
              (A_L[0], 0.0, A_L[1]), AXIS_NEG_Y)
    add_joint(robot, "right_crank_joint", "revolute", "gripper_base_link", "right_crank_link",
              (A_R[0], 0.0, A_R[1]), AXIS_POS_Y, mimic="left_crank_joint")
    add_joint(robot, "left_idle_joint", "revolute", "gripper_base_link", "left_idle_link",
              (B_L[0], 0.0, B_L[1]), AXIS_NEG_Y, mimic="left_crank_joint")
    add_joint(robot, "right_idle_joint", "revolute", "gripper_base_link", "right_idle_link",
              (B_R[0], 0.0, B_R[1]), AXIS_POS_Y, mimic="left_crank_joint")
    add_joint(robot, "left_finger_joint", "revolute", "left_crank_link", "left_finger_link",
              (0.0, 0.0, CRANK_LEN_MM), AXIS_POS_Y, mimic="left_crank_joint")
    add_joint(robot, "right_finger_joint", "revolute", "right_crank_link", "right_finger_link",
              (0.0, 0.0, CRANK_LEN_MM), AXIS_NEG_Y, mimic="left_crank_joint")
    add_joint(robot, "grip_center_joint", "fixed", "gripper_base_link", "grip_center_link",
              (0.0, 0.0, GRIP_CENTER_Z_MM))
    return robot
