"""Parallel-linkage gear gripper driven by one Dynamixel MX-64, mounted on a
Dynamixel-P PH42-020-S300-R (robot joint J6).

Coordinate convention (mm):
  origin = centre of the J6 (PH42) output horn face
  +Z     = away from J6 toward the fingertips (gripper approach axis)
  X      = finger opening direction
  Y      = pivot bolt direction (mechanism layers stacked along Y)

Kinematics:
  The MX-64 horn drives the left spur gear (screwed to the horn). It meshes 1:1
  with the right spur gear, so both cranks rotate equal-and-opposite. Each side
  is a parallelogram four-bar (crank A-A', idle link B-B', |AB| == |A'B'|), so
  the finger coupler translates without rotating: true parallel jaws.

  crank angle theta (deg, measured outward from +Z):
    theta_closed ~ -13.7 deg -> pads touch at x = 0
    theta_open   =  35 deg   -> ~89 mm jaw opening
  fingertip reach from the J6 flange = gear_axis_z + crank_len*cos(theta) + finger_len
    = 200 mm at theta = 0 (maximum overall length)

Pivot construction (three-layer stack):
  back plate | crank / idle link (on a steel bushing) | front plate
  one M4 socket-head bolt through all three, head on the front plate, nut on the
  back-plate rear face. The bolt clamps both plates onto the bushing, so the
  link turns freely on the bushing. Coupler (finger) pivots: M4 bolt through a
  bushing in the finger coupler and the crank / idle link, nut behind the link.

Nut / tool access (every bolt hole was checked):
  - fixed-pivot nuts sit on the back-plate rear face OUTSIDE the MX-64 housing
    footprint with >= 4.5 mm socket clearance;
  - coupler-pivot nuts are behind the links above the back plate / housing;
  - tab screws thread into tapped base-plate holes (no nut) and are reachable
    from above; the J6 screws go in through a separate adapter flange first,
    then the base plate is screwed to the flange from below, outside the PH42 body.

MX-64: the official ROBOTIS MX-64AT/AR STEP (parts/MX-64AT_AR.stp, from the
ROBOTIS e-Manual drawings section) is imported as-is. Every MX-64 interface
dimension below was measured from that STEP: horn dia 28 with 8x M2.5 on
PCD 22, horn boss, front mounting ears (8x M2.5 tapped thru) and the raised
centre body. The servo bolts to the back-plate rear face by its ears.
PH42 (J6): the PH42-020-S300-R STEP supplied with the project
(parts/PH42-020-S300-R.stp) is imported as-is; the adapter flange holes,
dowel holes and centring boss were measured from it.
"""

import math
from pathlib import Path

from build123d import (
    Align,
    Axis,
    Box,
    Circle,
    Color,
    Cylinder,
    Location,
    Plane,
    Polygon,
    Pos,
    Rectangle,
    RegularPolygon,
    Rot,
    Sketch,
    Vector,
    extrude,
    fillet,
    import_step,
    mirror,
)
from cadpy.assembly import AssemblyHelper

# ---------------------------------------------------------------- pose
crank_angle_deg = 15.0          # pose exported to STEP (closed -13.7 .. open 35)
crank_angle_min_deg = -13.7     # pads touch
crank_angle_max_deg = 35.0      # fully open

# ---------------------------------------------------------------- fasteners
m3_clear = 3.4
m3_tap = 2.5                    # tap drill, also the modelled thread diameter
m3_head_dia = 5.5
m3_head_h = 3.0
m3_cbore_dia = 6.5
m3_cbore_depth = 3.2
m4_clear = 4.3
m4_dia = 4.0
m4_head_dia = 7.0
m4_head_h = 4.0
m4_nut_af = 7.0                 # across flats
m4_nut_h = 3.2
m2_5_clear = 2.7
m2_5_tap = 2.0
m2_5_head_dia = 4.5
m2_5_head_h = 2.0
m2_5_low_head_h = 1.6
bushing_od = 6.0                # steel pivot bushing, 4.2 bore for the M4 bolt
bushing_id = 4.2
bushing_hole = 6.2
socket_clear_r = 5.0            # radius needed around a nut for a socket / nut driver

# ---------------------------------------------------------------- J6: PH42-020-S300-R
# Measured from parts/PH42-020-S300-R.stp. STEP frame: output axis = +Y through (0, 0),
# output horn face at y = 6. STEP (x, y, z) -> gripper (x, -z, y - 6).
j6_step_path = "parts/PH42-020-S300-R.stp"
j6_step_horn_face_y = 6.0
j6_horn_dia = 33.0              # rotating output horn (face), 2.5 proud of the case ring
j6_horn_tap_depth = 5.6         # 8x M3 tapped (dia 2.5 drill), ~5.65 deep
j6_horn_holes = [               # (x, y) in the gripper frame: NOT an even 45 deg pattern
    (12.0, 6.0), (6.0, 12.0), (-6.0, 12.0), (-12.0, 6.0),
    (-12.0, -6.0), (-6.0, -12.0), (6.0, -12.0), (12.0, -6.0),
]
j6_dowel_holes = [(12.0, 0.0), (0.0, 12.0), (-12.0, 0.0), (0.0, -12.0)]   # 4x dia 2, 3 deep
j6_dowel_dia = 2.0
j6_recess_dia = 20.0            # centre recess in the horn face, 2 deep
j6_recess_depth = 2.0

# ---------------------------------------------------------------- J6 adapter flange
flange_dia = 66.0
flange_thk = 6.0
flange_screw_r = 28.0           # 4x M3 from below into the base plate, outside the PH42 body
flange_pilot_dia = j6_recess_dia - 0.2     # centring boss into the PH42 horn recess
flange_pilot_h = j6_recess_depth - 0.2
dowel_pin_len = 6.0             # dia 2 x 6 dowel: 3 in the horn, 3 in the flange

# ---------------------------------------------------------------- MX-64AT/AR (official STEP)
# STEP frame: horn axis = +Z through (0, 0); +Y toward the horn end of the case.
mx_step_path = "parts/MX-64AT_AR.stp"
mx_step_x = (-20.1, 20.1)       # measured bounding box
mx_step_y = (-48.1, 14.0)
mx_step_z = (-22.0, 28.0)
mx_ear_face_z = 17.0            # front mounting-ear face (mates the back plate)
mx_case_face_z = 20.5           # raised centre body front face
mx_raised_x = (-14.5, 14.5)     # raised centre body outline (3.5 mm proud of the ears)
mx_raised_y = (-42.5, 13.0)
mx_horn_face_z = 23.5
mx_horn_dia = 28.0
mx_horn_screw_pcd = 22.0        # 8x M2.5 tapped, 2.5 deep
mx_horn_screw_count = 8
mx_horn_screw_depth = 2.5
mx_horn_boss_dia = 10.0         # centre boss proud of the horn face (to z = 28)
mx_ear_holes = [                # 8x M2.5 tapped thru in the front ears (STEP x, y)
    (17.3, 4.0), (-17.3, 4.0), (17.3, -18.0), (-17.3, -18.0),
    (17.3, -40.0), (-17.3, -40.0), (11.0, -45.3), (-11.0, -45.3),
]
mx_ear_thk = 3.5                # ear thickness along Z (13.5 .. 17)

# ---------------------------------------------------------------- gears (spur, involute)
gear_module = 1.5
gear_teeth = 28
gear_pressure_deg = 20.0
gear_backlash = 0.06
gear_pitch_r = gear_module * gear_teeth / 2          # 21
gear_tip_r = gear_pitch_r + gear_module              # 22.5
gear_root_r = gear_pitch_r - 1.25 * gear_module      # 19.125 (horn screw heads end at r 13.6)
gear_bore_dia = mx_horn_boss_dia + 1.0               # clears the MX-64 horn boss
horn_screw_cbore_dia = 5.2
horn_screw_cbore_depth = 2.5
ear_screw_cbore_depth = 2.5     # MX-64 ear screws, counterbored into the back-plate front face

# ---------------------------------------------------------------- linkage
flange_top = flange_thk
base_thk = 6.0
base_top = flange_top + base_thk                     # 12
crank_len = 55.0                # |AA'| == |BB'|
idle_offset = 37.0              # |AB|: idle link clears the gear tip by 2.3 mm at 35 deg
link_w = 10.0
boss_r = 6.0
finger_w = 10.0
pad_thk = 3.0
pad_len = 50.0
finger_depth = 18.0
finger_neck = 10.0              # room for the coupler bolt head before the finger widens
overall_reach = 200.0

# ---------------------------------------------------------------- Y layer stack
front_y = (-3.0, 3.0)           # front plate AND finger coupler layer (coupler sweeps above the plate)
link_y = (-11.5, -3.5)          # gears, cranks, idle links (8 mm), 0.5 mm running gap each side
back_y = (-18.0, -12.0)         # back plate
# STEP -> global: x -> -X, y -> +Z, z -> +Y; ears flush on the back-plate rear face
mx_origin_y = back_y[0] - mx_ear_face_z
servo_y = (mx_origin_y + mx_step_z[0], back_y[0])

# ---------------------------------------------------------------- MX-64 upright, horn end up
gear_axis_z = math.ceil(base_top + 0.8 - mx_step_y[0])               # 61: servo clears the base plate
finger_len = overall_reach - gear_axis_z - crank_len                   # 84

A_L = (-gear_pitch_r, gear_axis_z)
A_R = (gear_pitch_r, gear_axis_z)
B_L = (-gear_pitch_r - idle_offset, gear_axis_z)
B_R = (gear_pitch_r + idle_offset, gear_axis_z)

mx_x = (A_L[0] - mx_step_x[1], A_L[0] - mx_step_x[0])
mx_z = (gear_axis_z + mx_step_y[0], gear_axis_z + mx_step_y[1])

# ---------------------------------------------------------------- housing + adapter bay
housing_wall = 2.5
housing_clear = 0.5
board_len = 60.0                # adapter board envelope: 60 (Z) x 30 (X) x 20 (Y), standing upright
board_w = 30.0
board_depth = 20.0
board_gap_to_servo = 2.0
board_standoff = 3.0
board_hole_inset = 3.5
standoff_dia = 5.0
port_w = 14.0                   # Dynamixel junction ports through both side walls
port_h = 10.0
board_y = (servo_y[0] - board_gap_to_servo - board_depth, servo_y[0] - board_gap_to_servo)
board_x = ((mx_x[0] + mx_x[1]) / 2 - board_w / 2, (mx_x[0] + mx_x[1]) / 2 + board_w / 2)
board_z = (base_top + 2.0, base_top + 2.0 + board_len)

housing_x = (mx_x[0] - housing_clear - housing_wall, mx_x[1] + housing_clear + housing_wall)
housing_y = (board_y[0] - board_standoff - housing_wall, back_y[0])
housing_z = (base_top, max(mx_z[1], board_z[1]) + housing_clear + housing_wall)

# ---------------------------------------------------------------- RealSense D435 / D435i depth camera
# Intel RealSense D400 datasheet (337029-017): 90 x 25 x 25.05 mm, 75 g; back face 2x M3
# 45 mm apart (max insertion 3 mm, 0.4 N*m); 1/4-20 tripod hole underneath; depth origin =
# left imager centre, 17.5 mm from the tripod centreline, 4.2 mm behind the front glass;
# imager baseline 50 mm; min-Z 105 mm @ 424x240, 195 mm @ 848x480, 280 mm @ 1280x720.
# Mounted eye-in-hand looking along +Z (approach axis), bottom (tripod) toward -Y.
cam_w = 90.0                    # along X
cam_h = 25.0                    # along Y
cam_d = 25.05                   # along Z (optical axis)
cam_m3_spacing = 45.0
cam_m3_insert_max = 3.0
cam_left_imager_x = 17.5        # camera's left = +X when it looks along +Z with up = +Y
cam_right_imager_x = cam_left_imager_x - 50.0
cam_rgb_x = cam_left_imager_x + 15.0
cam_projector_x = cam_left_imager_x - 29.0
cam_depth_origin_behind_glass = 4.2
cbr_thk = 4.0                   # flat camera bracket plate on the base plate
cam_gap_to_front_plate = 0.5
cam_centre_y = front_y[1] + cam_gap_to_front_plate + cam_h / 2     # 16: camera right against the front plate
cam_back_z = base_top + cbr_thk                                     # 16: as low as it goes (on the bracket plate)
cam_x = (-cam_w / 2, cam_w / 2)
cam_y = (cam_centre_y - cam_h / 2, cam_centre_y + cam_h / 2)
cam_z = (cam_back_z, cam_back_z + cam_d)

# camera bracket: flat plate under the D435. The camera is screwed to it first (2x M3 from the
# plate underside, counterbored, low heads); the plate then screws to the base plate with 2x M3
# from above, in front of the camera where a driver reaches them.
cbr_half_w = 28.0               # clear of the front-plate tabs (moved to x = +/- 50, +/- 61)
cbr_y = (cam_y[0], cam_y[1] + 11.5)
cbr_cam_cbore_dia = 6.5
cbr_cam_cbore_depth = 2.2       # low-head M3 (2 mm head) sits below the plate underside
cbr_foot_screw_xy = [(x, cam_y[1] + 6.0) for x in (-15.0, 15.0)]       # M3 into the base plate, from above
cam_screw_xy = [(x, cam_centre_y) for x in (-cam_m3_spacing / 2, cam_m3_spacing / 2)]  # M3 x 4 into the D435 back

# ---------------------------------------------------------------- plates / tabs
tab_thk = 3.0
tab_w = 10.0
plate_half_w = 66.0
back_plate_top = housing_z[1]                                    # covers the MX-64 face
front_plate_top = gear_axis_z + gear_tip_r + 5.5                 # covers the gears, below the coupler sweep
base_y = (housing_y[0], max(flange_screw_r + 6.0, cbr_y[1] + 2.0))

# tab screw positions (x, y). Rear tabs on the back plate avoid the housing; their screw
# axis stays behind the pivot nuts so a driver reaches them from above.
back_tab_xy = [(x, back_y[0] - 6.0) for x in (-61.0, 12.0, 36.0, 61.0)]
front_tab_xy = [(x, front_y[1] + 6.0) for x in (-61.0, -50.0, 50.0, 61.0)]   # outside the D435 footprint (x +/- 45)
housing_tab_xy = [
    (x, y)
    for x in (housing_x[0] - 3.0, housing_x[1] + 3.0)
    for y in (housing_y[0] + 10.0, housing_y[1] - 32.0)          # clear of the back-plate tabs
]

ALU = Color(0.72, 0.74, 0.78)
DARK = Color(0.18, 0.18, 0.2)
BRASS = Color(0.85, 0.65, 0.25)
STEEL = Color(0.45, 0.47, 0.5)
RUBBER = Color(0.1, 0.1, 0.1)
BLUE = Color(0.2, 0.35, 0.6)
PCB = Color(0.1, 0.45, 0.2)


# ================================================================= helpers
def xz_prism(sketch: Sketch, y0: float, y1: float):
    """Extrude a sketch drawn in local XY (local Y == global Z) to global Y in [y0, y1]."""
    t = y1 - y0
    solid = extrude(sketch, amount=t)
    return Pos(0, y0 + t, 0) * (Rot(90, 0, 0) * solid)        # (x,y,z) -> (x,-z,y)


def axis_cyl(origin, direction, length, dia):
    pl = Plane(origin=Vector(*origin), z_dir=Vector(*direction))
    return pl.location * Cylinder(dia / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def y_span(x, z, y0, y1, dia):
    return axis_cyl((x, y0, z), (0, 1, 0), y1 - y0, dia)


def cap_screw(seat, into, shank_len, head_dia, head_h, shank_dia):
    """Socket-head screw: head sits on `seat` facing away from `into`; shank goes along `into`."""
    sx, sy, sz = seat
    dx, dy, dz = into
    head = axis_cyl((sx - dx * head_h, sy - dy * head_h, sz - dz * head_h), into, head_h, head_dia)
    return head + axis_cyl(seat, into, shank_len, shank_dia)


def m4_nut(x, z, y_face, toward_minus_y=True):
    """Hex nut seated on a face at y_face, extending away from it."""
    sk = Pos(x, z) * RegularPolygon(m4_nut_af / math.sqrt(3), 6) - Pos(x, z) * Circle(m4_dia / 2)
    y0, y1 = (y_face - m4_nut_h, y_face) if toward_minus_y else (y_face, y_face + m4_nut_h)
    return xz_prism(sk, y0, y1)


def bushing(x, z, y0, y1):
    return y_span(x, z, y0, y1, bushing_od) - y_span(x, z, y0 - 0.1, y1 + 0.1, bushing_id)


def involute_gear_points(module, teeth, pressure_deg, backlash, phase_deg=0.0, flank_pts=8):
    r = module * teeth / 2
    rb = r * math.cos(math.radians(pressure_deg))
    ra = r + module
    rf = r - 1.25 * module
    inv = lambda a: math.tan(a) - a
    alpha_p = math.radians(pressure_deg)
    psi_p = math.pi / (2 * teeth) - backlash / r

    def half_angle(rho):
        a = math.acos(min(1.0, rb / rho))
        return psi_p + inv(alpha_p) - inv(a)

    r_start = max(rb, rf)
    radii = [r_start + (ra - r_start) * i / (flank_pts - 1) for i in range(flank_pts)]
    pitch = 2 * math.pi / teeth
    pol = lambda rho, ang: (rho * math.cos(ang), rho * math.sin(ang))
    pts = []
    for k in range(teeth):
        c = math.radians(phase_deg) + k * pitch
        psi_b = half_angle(r_start)
        pts.append(pol(rf, c - psi_b))
        for rho in radii:
            pts.append(pol(rho, c - half_angle(rho)))
        pts.append(pol(ra, c))
        for rho in reversed(radii):
            pts.append(pol(rho, c + half_angle(rho)))
        pts.append(pol(rf, c + psi_b))
        pts.append(pol(rf, c + pitch / 2))
    return pts


def horn_screw_angles():
    """Global angles (XZ plane) of the 8 MX-64 horn holes: 0, 45, ... deg (measured from the STEP)."""
    return [2 * math.pi * i / mx_horn_screw_count for i in range(mx_horn_screw_count)]


def gear_crank_sketch(phase_deg: float, on_horn: bool, theta: float = 0.0) -> Sketch:
    """Spur gear + integral crank arm pointing local +Y, centred at origin.

    The drive gear horn-screw holes are laid out so that, after the gear is rotated by
    +theta, they sit on the MX-64 horn holes (the horn turns with the gear).
    """
    gear = Polygon(
        *involute_gear_points(gear_module, gear_teeth, gear_pressure_deg, gear_backlash, phase_deg),
        align=None,
    )
    sk = gear + Pos(0, crank_len / 2) * Rectangle(link_w + 2, crank_len) + Pos(0, crank_len) * Circle(boss_r)
    sk -= Pos(0, crank_len) * Circle(m4_clear / 2)               # coupler pivot bolt
    if on_horn:
        sk -= Circle(gear_bore_dia / 2)                          # MX-64 horn boss
        for a in horn_screw_angles():
            a_local = a - math.radians(theta)
            sk -= Pos(mx_horn_screw_pcd / 2 * math.cos(a_local), mx_horn_screw_pcd / 2 * math.sin(a_local)) * Circle(m2_5_clear / 2)
    else:
        sk -= Circle(bushing_hole / 2)                           # turns on the fixed-pivot bushing
    return sk


def link_sketch() -> Sketch:
    sk = Pos(0, crank_len / 2) * Rectangle(link_w, crank_len)
    sk += Circle(boss_r) + Pos(0, crank_len) * Circle(boss_r)
    sk -= Circle(bushing_hole / 2) + Pos(0, crank_len) * Circle(m4_clear / 2)
    return sk


def crank_dir(theta_deg: float, side: int):
    t = math.radians(theta_deg)
    return (side * math.sin(t), math.cos(t))


def coupler_points(theta: float, side: int):
    ux, uz = crank_dir(theta, side)
    a, b = (A_L, B_L) if side < 0 else (A_R, B_R)
    return (a[0] + crank_len * ux, a[1] + crank_len * uz), (b[0] + crank_len * ux, b[1] + crank_len * uz)


def tab_screw(x, y):
    return cap_screw((x, y, base_top + tab_thk), (0, 0, -1), tab_thk + base_thk, m3_head_dia, m3_head_h, m3_tap)


def plate_tab(x, y_from, y_to):
    y0, y1 = min(y_from, y_to), max(y_from, y_to)
    return Pos(x, (y0 + y1) / 2, base_top + tab_thk / 2) * Box(tab_w, y1 - y0, tab_thk)


# ================================================================= static parts
def j6_bolt_xy():
    return list(j6_horn_holes)


def flange_screw_xy():
    return [(flange_screw_r, 0.0), (-flange_screw_r, 0.0), (0.0, flange_screw_r), (0.0, -flange_screw_r)]


def make_j6():
    """PH42-020-S300-R STEP, output horn face on z = 0, output axis = +Z."""
    src = Path(globals().get("__file__", "gripper_mx64_ph42.py")).resolve().parent / j6_step_path
    ph42 = import_step(str(src))
    # STEP x -> X, STEP y -> Z, STEP z -> -Y
    place = Plane(origin=Vector(0, 0, -j6_step_horn_face_y), x_dir=Vector(1, 0, 0), z_dir=Vector(0, -1, 0))
    return place.location * ph42


def make_j6_flange():
    """Round adapter: counterbored M3 into the J6 horn (fitted first), clearance M3 up into the base plate."""
    fl = Pos(0, 0, flange_thk / 2) * Cylinder(flange_dia / 2, flange_thk)
    fl += Pos(0, 0, -flange_pilot_h / 2) * Cylinder(flange_pilot_dia / 2, flange_pilot_h + 0.01)
    for x, y in j6_dowel_holes:
        fl -= Pos(x, y, (flange_thk - flange_pilot_h) / 2) * Cylinder(j6_dowel_dia / 2, flange_thk + flange_pilot_h)
    for x, y in j6_bolt_xy():
        fl -= Pos(x, y, flange_thk / 2) * Cylinder(m3_clear / 2, flange_thk)
        fl -= Pos(x, y, flange_thk - m3_cbore_depth / 2) * Cylinder(m3_cbore_dia / 2, m3_cbore_depth)
    for x, y in flange_screw_xy():
        fl -= Pos(x, y, flange_thk / 2) * Cylinder(m3_clear / 2, flange_thk)
    return fl


def make_base_plate():
    yc = (base_y[0] + base_y[1]) / 2
    plate = Pos(0, yc, flange_top + base_thk / 2) * Box(2 * plate_half_w, base_y[1] - base_y[0], base_thk)
    plate = fillet(plate.edges().filter_by(Axis.Z), 4.0)
    for x, y in flange_screw_xy() + back_tab_xy + front_tab_xy + housing_tab_xy + cbr_foot_screw_xy:
        plate -= Pos(x, y, flange_top + base_thk / 2) * Cylinder(m3_tap / 2, base_thk)
    return plate


def plate_sketch(top_z: float, horn_hole: bool) -> Sketch:
    sk = Pos(0, (base_top + top_z) / 2) * Rectangle(2 * plate_half_w, top_z - base_top)
    sk = fillet(sk.vertices().group_by(Axis.Y)[-1], 6.0)
    if horn_hole:
        sk -= Pos(*A_L) * Circle(mx_horn_dia / 2 + 0.5)        # dia 29 for the dia 28 MX-64 horn
    for p in (A_R, B_L, B_R):
        sk -= Pos(*p) * Circle(m4_clear / 2)                    # one bolt through plate-link-plate
    return sk


def mx_to_global(sx, sy, sz):
    """MX-64 STEP coordinates -> assembly coordinates."""
    return (A_L[0] - sx, mx_origin_y + sz, A_L[1] + sy)


def ear_hole_xz():
    return [(mx_to_global(sx, sy, 0)[0], mx_to_global(sx, sy, 0)[2]) for sx, sy in mx_ear_holes]


def make_back_plate():
    """Pivot plate. The MX-64 ears bolt to its rear face; the raised case body sits in a pocket."""
    plate = xz_prism(plate_sketch(back_plate_top, horn_hole=True), *back_y)
    # pocket for the raised centre body of the MX-64 (0.3 mm clearance all round)
    gx = sorted(mx_to_global(sx, 0, 0)[0] for sx in mx_raised_x)
    gz = sorted(mx_to_global(0, sy, 0)[2] for sy in mx_raised_y)
    depth = (mx_case_face_z - mx_ear_face_z) + 0.3
    plate -= Pos((gx[0] + gx[1]) / 2, back_y[0] + (depth - 0.01) / 2, (gz[0] + gz[1]) / 2) * Box(
        gx[1] - gx[0] + 0.6, depth + 0.01, gz[1] - gz[0] + 0.6
    )
    # 8x M2.5 into the MX-64 front ears, counterbored from the gear side
    for ex, ez in ear_hole_xz():
        plate -= y_span(ex, ez, back_y[0] - 0.01, back_y[1] + 0.01, m2_5_clear)
        plate -= y_span(ex, ez, back_y[1] - ear_screw_cbore_depth, back_y[1] + 0.01, horn_screw_cbore_dia)
    for x, y in back_tab_xy:
        plate += plate_tab(x, back_y[1], y - 5.0)
        plate -= Pos(x, y, base_top + tab_thk / 2) * Cylinder(m3_clear / 2, tab_thk)
    return plate


def make_front_plate():
    plate = xz_prism(plate_sketch(front_plate_top, horn_hole=False), *front_y)
    for x, y in front_tab_xy:
        plate += plate_tab(x, front_y[0], y + 5.0)
        plate -= Pos(x, y, base_top + tab_thk / 2) * Cylinder(m3_clear / 2, tab_thk)
    return plate


def board_hole_positions():
    return [
        (x, z)
        for x in (board_x[0] + board_hole_inset, board_x[1] - board_hole_inset)
        for z in (board_z[0] + board_hole_inset, board_z[1] - board_hole_inset)
    ]


def port_centres():
    yc = (board_y[0] + board_y[1]) / 2
    zc = (board_z[0] + board_z[1]) / 2
    return [(housing_x[0] + housing_wall / 2, yc, zc), (housing_x[1] - housing_wall / 2, yc, zc)]


def make_mx64_housing():
    """Cover around the MX-64 + adapter bay. Open only onto the back plate and the base plate."""
    sx, sy, sz = housing_x[1] - housing_x[0], housing_y[1] - housing_y[0], housing_z[1] - housing_z[0]
    shell = Pos((housing_x[0] + housing_x[1]) / 2, (housing_y[0] + housing_y[1]) / 2, housing_z[0] + sz / 2) * Box(sx, sy, sz)
    shell = fillet(shell.edges().filter_by(Axis.Y).group_by(Axis.Z)[-1], 4.0)
    ix = (housing_x[0] + housing_wall, housing_x[1] - housing_wall)
    iy = (housing_y[0] + housing_wall, housing_y[1] + 1.0)
    iz = (housing_z[0] - 1.0, housing_z[1] - housing_wall)
    shell -= Pos((ix[0] + ix[1]) / 2, (iy[0] + iy[1]) / 2, (iz[0] + iz[1]) / 2) * Box(
        ix[1] - ix[0], iy[1] - iy[0], iz[1] - iz[0]
    )
    for c in port_centres():
        port = Pos(*c) * Box(housing_wall * 2, port_w, port_h)
        shell -= fillet(port.edges().filter_by(Axis.X), 2.0)
    rear_inner_y = housing_y[0] + housing_wall
    for bx, bz in board_hole_positions():
        shell += y_span(bx, bz, rear_inner_y - 0.01, board_y[0], standoff_dia)
        shell -= y_span(bx, bz, rear_inner_y, board_y[0] + 0.01, m2_5_tap)
    for x, y in housing_tab_xy:
        left = x < A_L[0]
        wall_x = housing_x[0] if left else housing_x[1]
        # start inside the wall so the tab fuses, end 4 mm past the screw axis
        x0, x1 = sorted((wall_x + (housing_wall if left else -housing_wall), x + (-4.0 if left else 4.0)))
        shell += Pos((x0 + x1) / 2, y, base_top + tab_thk / 2) * Box(x1 - x0, tab_w, tab_thk)
        shell -= Pos(x, y, base_top + tab_thk / 2) * Cylinder(m3_clear / 2, tab_thk)
    return shell


def horn_screw_xz():
    return [
        (A_L[0] + mx_horn_screw_pcd / 2 * math.cos(a), A_L[1] + mx_horn_screw_pcd / 2 * math.sin(a))
        for a in horn_screw_angles()
    ]


def make_mx64():
    """Official ROBOTIS MX-64AT/AR STEP: ears on the back-plate rear face, horn on the drive gear axis."""
    src = Path(globals().get("__file__", "gripper_mx64_ph42.py")).resolve().parent / mx_step_path
    servo = import_step(str(src))
    # STEP x -> -X, STEP y -> +Z, STEP z -> +Y
    place = Plane(origin=Vector(A_L[0], mx_origin_y, A_L[1]), x_dir=Vector(-1, 0, 0), z_dir=Vector(0, 1, 0))
    return place.location * servo


def stadium_xy(w, h):
    return Rectangle(w - h, h) + Pos(-(w - h) / 2, 0) * Circle(h / 2) + Pos((w - h) / 2, 0) * Circle(h / 2)


def make_d435():
    """RealSense D435/D435i envelope from the datasheet drawing (lens features cosmetic)."""
    body = Pos(0, cam_centre_y, cam_z[0]) * extrude(stadium_xy(cam_w, cam_h), amount=cam_d)
    lenses = [(cam_left_imager_x, 4.5), (cam_right_imager_x, 4.5), (cam_rgb_x, 3.5), (cam_projector_x, 5.0)]
    for lx, r in lenses:
        body -= Pos(lx, cam_centre_y, cam_z[1] - 0.5) * Cylinder(r, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x, y in cam_screw_xy:                                     # 2x M3 tapped, 3 deep, in the back face
        body -= Pos(x, y, cam_z[0] - 0.01) * Cylinder(m3_tap / 2, cam_m3_insert_max, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= axis_cyl((0.0, cam_y[0] - 0.01, (cam_z[0] + cam_z[1]) / 2), (0, 1, 0), 6.0, 5.1)   # 1/4-20 tripod
    return body


def make_camera_bracket():
    """Flat plate under the D435: counterbored camera screws from below, 2 screws into the base plate."""
    z0 = base_top
    br = Pos(0, sum(cbr_y) / 2, z0 + cbr_thk / 2) * Box(2 * cbr_half_w, cbr_y[1] - cbr_y[0], cbr_thk)
    br = fillet(br.edges().filter_by(Axis.Z), 3.0)
    for x, y in cbr_foot_screw_xy:
        br -= Pos(x, y, z0 + cbr_thk / 2) * Cylinder(m3_clear / 2, cbr_thk + 0.02)
    for x, y in cam_screw_xy:
        br -= Pos(x, y, z0 + cbr_thk / 2) * Cylinder(m3_clear / 2, cbr_thk + 0.02)
        br -= Pos(x, y, z0 + cbr_cam_cbore_depth / 2 - 0.01) * Cylinder(cbr_cam_cbore_dia / 2, cbr_cam_cbore_depth + 0.02)
    return br


def make_adapter_board():
    pcb_thk = 1.6
    sk = Pos((board_x[0] + board_x[1]) / 2, (board_z[0] + board_z[1]) / 2) * Rectangle(board_w, board_len)
    for bx, bz in board_hole_positions():
        sk -= Pos(bx, bz) * Circle(m2_5_clear / 2)
    pcb = xz_prism(sk, board_y[0], board_y[0] + pcb_thk)
    comp_z = (board_z[0] + 8.0, board_z[1] - 8.0)
    comp = Pos((board_x[0] + board_x[1]) / 2, (board_y[0] + pcb_thk + board_y[1]) / 2, (comp_z[0] + comp_z[1]) / 2) * Box(
        board_w - 6.0, board_y[1] - board_y[0] - pcb_thk, comp_z[1] - comp_z[0]
    )
    return pcb, comp, pcb_thk


# ================================================================= moving parts
def make_gear_crank(side: int, theta: float, phase_deg: float):
    sk = gear_crank_sketch(phase_deg, on_horn=(side < 0), theta=theta).rotate(Axis.Z, -side * theta)
    pivot = A_L if side < 0 else A_R
    part = xz_prism(Pos(*pivot) * sk, *link_y)
    if side < 0:
        for hx, hz in horn_screw_xz():
            part -= y_span(hx, hz, link_y[1] - horn_screw_cbore_depth, link_y[1] + 0.01, horn_screw_cbore_dia)
    return part


def make_idle_link(side: int, theta: float):
    sk = link_sketch().rotate(Axis.Z, -side * theta)
    pivot = B_L if side < 0 else B_R
    return xz_prism(Pos(*pivot) * sk, *link_y)


def pad_screw_z(tip_z):
    z0 = tip_z - 3 - pad_len
    return (z0 + 6.0, z0 + 42.0)


def make_finger_left(theta: float):
    (ax, az), (bx, bz) = coupler_points(theta, -1)
    tip_z = az + finger_len

    cpl = Pos((ax + bx) / 2, az) * Rectangle(abs(ax - bx), 2 * boss_r)
    cpl += Pos(ax, az) * Circle(boss_r) + Pos(bx, bz) * Circle(boss_r)
    cpl += Pos(ax, az + finger_neck / 2) * Rectangle(finger_w, finger_neck)
    cpl -= Pos(ax, az) * Circle(bushing_hole / 2) + Pos(bx, bz) * Circle(bushing_hole / 2)
    coupler = xz_prism(cpl, *front_y)

    upper_len = tip_z - (az + finger_neck)
    stalk = Pos(ax, 0, az + finger_neck + upper_len / 2) * Box(finger_w, finger_depth, upper_len)
    stalk = fillet(stalk.edges().filter_by(Axis.X).group_by(Axis.Z)[-1], 3.0)
    finger = coupler + stalk
    inner_x = ax + finger_w / 2
    for z in pad_screw_z(tip_z):
        finger -= axis_cyl((inner_x + 0.01, 0, z), (-1, 0, 0), 6.0, m2_5_tap)

    pad = Pos(inner_x + pad_thk / 2, 0, tip_z - 3 - pad_len / 2) * Box(pad_thk, finger_depth - 2, pad_len)
    for i in range(8):
        pad -= Pos(inner_x + pad_thk, 0, tip_z - 3 - pad_len + 3 + i * 6) * Box(1.2, finger_depth, 1.5)
    screws = []
    for z in pad_screw_z(tip_z):
        face_x = inner_x + pad_thk
        pad -= axis_cyl((face_x + 0.01, 0, z), (-1, 0, 0), 1.5 + 0.01, m2_5_head_dia + 0.2)
        pad -= axis_cyl((face_x, 0, z), (-1, 0, 0), pad_thk + 0.01, m2_5_clear)
        screws.append(cap_screw((face_x - 1.5, 0, z), (-1, 0, 0), pad_thk - 1.5 + 5.5, m2_5_head_dia, 1.5, m2_5_tap))
    return finger, pad, screws, tip_z


def pick_gear_phase(theta: float):
    left = make_gear_crank(-1, theta, 0.0)
    pitch = 360.0 / gear_teeth
    best = None
    for frac in (0.0, 0.25, 0.5, 0.75):
        right = make_gear_crank(+1, theta, frac * pitch)
        inter = left & right
        ov = inter.volume if inter is not None else 0.0
        if best is None or ov < best[0]:
            best = (ov, frac * pitch, left, right)
    return best


# ================================================================= assembly
theta = crank_angle_deg
asm = AssemblyHelper("gripper_mx64_on_ph42_j6")

j6 = asm.add(make_j6(), "ph42_020_s300_r", "j6_step", color=DARK)
j6_face = asm.rigid_frame(j6, "j6_output_flange_face", Location((0, 0, 0)))
flange = asm.add(make_j6_flange(), "j6_adapter_flange", color=ALU)
flange_under = asm.rigid_frame(flange, "flange_underside", Location((0, 0, 0)))
asm.face_to_face(j6_face, flange_under, label="j6_horn_to_adapter_flange")

asm.add(make_base_plate(), "base_plate", color=ALU)
asm.add(make_back_plate(), "back_plate", "pivot_plate", color=ALU)
asm.add(make_front_plate(), "front_plate", "pivot_plate", color=ALU)
asm.add(make_mx64_housing(), "mx64_housing", "cover_with_adapter_bay", color=ALU)

asm.add(make_mx64(), "mx64at_ar", "robotis_official_step", color=DARK)
asm.add(make_camera_bracket(), "camera_bracket", "d435", color=ALU)
asm.add(make_d435(), "realsense_d435", "depth_camera", color=DARK)

board_pcb, board_comp, pcb_thk = make_adapter_board()
asm.add_module(
    "dynamixel_adapter_board_envelope",
    [
        asm.feature(board_pcb, "adapter_pcb", "30x60", color=PCB),
        asm.feature(board_comp, "adapter_component_envelope", "20mm_total_depth", color=DARK),
    ],
)

overlap, phase_r, gear_l, gear_r = pick_gear_phase(theta)
asm.add(gear_l, "drive_gear_crank", "left", "m1_5_z28_on_mx64_horn", color=BRASS)
asm.add(gear_r, "driven_gear_crank", "right", "m1_5_z28", color=BRASS)
asm.add(make_idle_link(-1, theta), "idle_link", "left", color=ALU)
asm.add(make_idle_link(+1, theta), "idle_link", "right", color=ALU)

finger_l, pad_l, pad_screws_l, tip_z = make_finger_left(theta)
asm.add(finger_l, "finger_coupler", "left", color=BLUE)
asm.add(mirror(finger_l, Plane.YZ), "finger_coupler", "right", color=BLUE)
asm.add(pad_l, "finger_pad", "left", color=RUBBER)
asm.add(mirror(pad_l, Plane.YZ), "finger_pad", "right", color=RUBBER)

# ---------------------------------------------------------------- pivot hardware
pivot_parts = []
fixed_bolt_len = (front_y[1] - back_y[0]) + m4_nut_h + 1.0                 # 25.2 -> M4x25
for name, p in (("driven_gear_right", A_R), ("idle_left", B_L), ("idle_right", B_R)):
    pivot_parts.append(asm.feature(
        cap_screw((p[0], front_y[1], p[1]), (0, -1, 0), fixed_bolt_len, m4_head_dia, m4_head_h, m4_dia),
        "m4_bolt", "three_layer_pivot", name, color=STEEL))
    pivot_parts.append(asm.feature(bushing(p[0], p[1], back_y[1], front_y[0]), "pivot_bushing", name, color=BRASS))
    pivot_parts.append(asm.feature(m4_nut(p[0], p[1], back_y[0]), "m4_nut", "three_layer_pivot", name, color=STEEL))

coupler_bushing_y = (link_y[1], front_y[1] + 0.3)                           # 0.3 proud: head clamps the bushing only
coupler_bolt_len = (coupler_bushing_y[1] - link_y[0]) + m4_nut_h + 1.0
for side, sname in ((-1, "left"), (1, "right")):
    (ax, az), (bx, bz) = coupler_points(theta, side)
    for jname, (px, pz) in (("crank", (ax, az)), ("idle", (bx, bz))):
        tag = f"{jname}_{sname}"
        pivot_parts.append(asm.feature(
            cap_screw((px, coupler_bushing_y[1], pz), (0, -1, 0), coupler_bolt_len, m4_head_dia, m4_head_h, m4_dia),
            "m4_bolt", "coupler_pivot", tag, color=STEEL))
        pivot_parts.append(asm.feature(bushing(px, pz, *coupler_bushing_y), "pivot_bushing", tag, color=BRASS))
        pivot_parts.append(asm.feature(m4_nut(px, pz, link_y[0]), "m4_nut", "coupler_pivot", tag, color=STEEL))
asm.add_module("pivot_bolts_bushings_nuts", pivot_parts)

# ---------------------------------------------------------------- screws
screws = []
for i, (x, y) in enumerate(j6_bolt_xy()):
    screws.append(asm.feature(
        cap_screw((x, y, flange_thk - m3_cbore_depth), (0, 0, -1), flange_thk - m3_cbore_depth + 5.0, m3_head_dia, m3_head_h, m3_tap),
        "m3_screw", "j6_horn_to_flange", i, color=STEEL))
for i, (x, y) in enumerate(j6_dowel_holes):
    screws.append(asm.feature(
        Pos(x, y, 0) * Cylinder(j6_dowel_dia / 2, dowel_pin_len),
        "dowel_pin", "j6_horn_to_flange", i, color=STEEL))
for i, (x, y) in enumerate(flange_screw_xy()):
    screws.append(asm.feature(
        cap_screw((x, y, 0.0), (0, 0, 1), flange_thk + base_thk, m3_head_dia, m3_head_h, m3_tap),
        "m3_screw", "flange_to_base_from_below", i, color=STEEL))
for group, pts in (("back_plate_tab", back_tab_xy), ("front_plate_tab", front_tab_xy), ("housing_tab", housing_tab_xy)):
    for i, (x, y) in enumerate(pts):
        screws.append(asm.feature(tab_screw(x, y), "m3_screw", group, i, color=STEEL))
for i, (hx, hz) in enumerate(horn_screw_xz()):
    seat_y = link_y[1] - horn_screw_cbore_depth
    screws.append(asm.feature(
        cap_screw((hx, seat_y, hz), (0, -1, 0), (seat_y - link_y[0]) + mx_horn_screw_depth, m2_5_head_dia, m2_5_low_head_h, m2_5_tap),
        "m2_5_screw", "gear_to_mx64_horn", i, color=STEEL))
for i, (ex, ez) in enumerate(ear_hole_xz()):
    seat_y = back_y[1] - ear_screw_cbore_depth
    screws.append(asm.feature(
        cap_screw((ex, seat_y, ez), (0, -1, 0), (seat_y - back_y[0]) + mx_ear_thk, m2_5_head_dia, m2_5_low_head_h, m2_5_tap),
        "m2_5_screw", "mx64_ears_to_back_plate", i, color=STEEL))
for i, (x, y) in enumerate(cbr_foot_screw_xy):
    screws.append(asm.feature(
        cap_screw((x, y, base_top + cbr_thk), (0, 0, -1), cbr_thk + base_thk, m3_head_dia, m3_head_h, m3_tap),
        "m3_screw", "camera_bracket_to_base", i, color=STEEL))
for i, (x, y) in enumerate(cam_screw_xy):
    screws.append(asm.feature(
        cap_screw((x, y, base_top + cbr_cam_cbore_depth), (0, 0, 1), (cbr_thk - cbr_cam_cbore_depth) + cam_m3_insert_max - 0.5,
                  m3_head_dia, 2.0, m3_tap),
        "m3_screw", "d435_to_bracket_low_head", i, color=STEEL))
for i, (bx, bz) in enumerate(board_hole_positions()):
    seat_y = board_y[0] + pcb_thk
    screws.append(asm.feature(
        cap_screw((bx, seat_y, bz), (0, -1, 0), pcb_thk + board_standoff, m2_5_head_dia, m2_5_head_h, m2_5_tap),
        "m2_5_screw", "adapter_board", i, color=STEEL))
for i, s in enumerate(pad_screws_l):
    screws.append(asm.feature(s, "m2_5_screw", "pad_left", i, color=STEEL))
    screws.append(asm.feature(mirror(s, Plane.YZ), "m2_5_screw", "pad_right", i, color=STEEL))
asm.add_module("screws", screws)


def gen_step():
    return asm.build()


if __name__ == "__main__":
    (ax, _), _ = coupler_points(theta, -1)
    print("gear_axis_z", gear_axis_z, "finger_len", finger_len, "right gear phase", phase_r, "gear overlap", overlap)
    print("fingertip z", round(tip_z, 2), "jaw gap", round(-2 * (ax + finger_w / 2 + pad_thk), 2))
    print("horn screw cbore edge r", mx_horn_screw_pcd / 2 + horn_screw_cbore_dia / 2, "gear root r", gear_root_r,
          "-> clearance", round(gear_root_r - mx_horn_screw_pcd / 2 - horn_screw_cbore_dia / 2, 2))
    print("housing x", housing_x, "y", housing_y, "z", housing_z, "plates top", back_plate_top, front_plate_top)
