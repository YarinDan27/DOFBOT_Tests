#!/usr/bin/env python3
"""
Our arm model as a URDF -- the robot description ROS (RViz, MoveIt) and
MuJoCo read.

Microduck keeps one description of its robot (an MJCF made from the CAD) and
the simulator, the trainer and the real robot all use it. We have one too:
config.py's lengths and thicknesses, used by robot/body.py for safety and by
the 3D view. This writes that same model out as a URDF, so RViz shows exactly
the arm the safety checks use -- not Yahboom's mesh, whose numbers we can't
see -- and tests/test_urdf.py checks the two agree, pose by pose.

    python3 -m viz.urdf          # writes dofbot_chess.urdf next to run.py

Joint values (radians) from servo angles (degrees), as joint_values() does:
    base      = radians(servo1 - 90)      turns about +z (left = positive)
    shoulder  = radians(90 - servo2)      the three pitch joints turn about -x:
    elbow     = radians(90 - servo3)      positive = leaning toward the board
    wrist     = radians(90 - servo4)
    roll      = radians(servo5 - 90)      about the hand
Limits are config.JOINT_RANGE (what each servo can physically do). Lengths
are in metres, the same frame as everywhere else: x right, y toward the
board, z up, origin at the centre of the turning base on the base plate.

Every link is a capsule, as in body.py: URDF has no capsule, so each is a
cylinder with a sphere at each end. The electronics box, base plate, turret
and table are fixed links too, so RViz shows what the arm is kept clear of.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                       # noqa: E402
from robot import world             # noqa: E402

JOINTS = ("base", "shoulder", "elbow", "wrist", "roll")
OUT = os.path.join(config.ROOT, "dofbot_chess.urdf")
COLOURS = {"arm": (0.92, 0.75, 0.15, 1.0), "dark": (0.15, 0.15, 0.17, 1.0),
           "box": (0.25, 0.25, 0.28, 0.8), "table": (0.75, 0.68, 0.55, 0.5)}


def joint_values(angles):
    """{joint name: radians} for servo angles [s1..s5] (degrees) -- for /joint_states."""
    s = [float(a) for a in angles[:5]] + [90.0] * (5 - len(angles[:5]))
    return {"base": math.radians(s[0] - 90.0), "shoulder": math.radians(90.0 - s[1]),
            "elbow": math.radians(90.0 - s[2]), "wrist": math.radians(90.0 - s[3]),
            "roll": math.radians(s[4] - 90.0)}


def limits(name):
    """(lower, upper) radians for a joint, from config.JOINT_RANGE."""
    sid = JOINTS.index(name) + 1
    lo, hi = config.JOINT_RANGE[sid]
    if name in ("base", "roll"):
        return math.radians(lo - 90.0), math.radians(hi - 90.0)
    return math.radians(90.0 - hi), math.radians(90.0 - lo)


def _m(mm):
    return "%.5f" % (mm / 1000.0)


def _xyz(x, y, z):
    return "%s %s %s" % (_m(x), _m(y), _m(z))


def _material(parent, name):
    mat = ET.SubElement(parent, "material", name=name)
    ET.SubElement(mat, "color", rgba="%.2f %.2f %.2f %.2f" % COLOURS[name])


def _capsule(link, a, b, radius, colour):
    """Cylinder from a to b (link frame, mm, along a single axis) + spheres at the ends."""
    d = [b[i] - a[i] for i in range(3)]
    length = math.sqrt(sum(c * c for c in d))
    mid = [(a[i] + b[i]) / 2.0 for i in range(3)]
    # a cylinder lies along z: turn it onto the x axis if that's where it goes
    rpy = "0 1.5708 0" if abs(d[0]) > abs(d[2]) else "0 0 0"
    for tag in ("visual", "collision"):
        el = ET.SubElement(link, tag)
        ET.SubElement(el, "origin", xyz=_xyz(*mid), rpy=rpy)
        geo = ET.SubElement(el, "geometry")
        ET.SubElement(geo, "cylinder", radius=_m(radius), length=_m(length))
        if tag == "visual":
            ET.SubElement(el, "material", name=colour)
        for end in (a, b):
            el = ET.SubElement(link, tag)
            ET.SubElement(el, "origin", xyz=_xyz(*end), rpy="0 0 0")
            geo = ET.SubElement(el, "geometry")
            ET.SubElement(geo, "sphere", radius=_m(radius))
            if tag == "visual":
                ET.SubElement(el, "material", name=colour)


def _inertial(link, mass_kg):
    """A small token inertia: RViz doesn't need it, MuJoCo and Gazebo refuse links without."""
    el = ET.SubElement(link, "inertial")
    ET.SubElement(el, "mass", value="%.3f" % mass_kg)
    ET.SubElement(el, "inertia", ixx="1e-4", iyy="1e-4", izz="1e-4", ixy="0", ixz="0", iyz="0")


def _joint(robot, name, parent, child, xyz, axis, kind="revolute"):
    j = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    ET.SubElement(j, "origin", xyz=_xyz(*xyz), rpy="0 0 0")
    if kind != "fixed":
        ET.SubElement(j, "axis", xyz=axis)
        lo, hi = limits(name)
        ET.SubElement(j, "limit", lower="%.4f" % lo, upper="%.4f" % hi, effort="1.5",
                      velocity="%.3f" % math.radians(config.JOINT_SPEED_DEG_S))
    return j


def _box(robot, name, ob_center, half, colour):
    link = ET.SubElement(robot, "link", name=name)
    for tag in ("visual", "collision"):
        el = ET.SubElement(link, tag)
        ET.SubElement(el, "origin", xyz=_xyz(*ob_center), rpy="0 0 0")
        geo = ET.SubElement(el, "geometry")
        ET.SubElement(geo, "box", size=" ".join(_m(2.0 * h) for h in half))
        if tag == "visual":
            ET.SubElement(el, "material", name=colour)
    _joint(robot, name + "_fixed", "base_link", name, (0.0, 0.0, 0.0), None, "fixed")


def build(zone=None):
    """The URDF as an ElementTree root element."""
    zone = zone or world.World.default()
    robot = ET.Element("robot", name="dofbot_chess")
    for name in COLOURS:
        _material(robot, name)
    ET.SubElement(robot, "link", name="base_link")

    # the turning base: everything above the base plate turns with servo 1
    turn = ET.SubElement(robot, "link", name="turret_link")
    _inertial(turn, 0.3)
    for part in config.TURRET:
        z0, z1 = part["z"]
        for tag in ("visual", "collision"):
            el = ET.SubElement(turn, tag)
            ET.SubElement(el, "origin", xyz=_xyz(0, 0, (z0 + z1) / 2.0), rpy="0 0 0")
            geo = ET.SubElement(el, "geometry")
            ET.SubElement(geo, "cylinder", radius=_m(part["radius"]), length=_m(z1 - z0))
            if tag == "visual":
                ET.SubElement(el, "material", name="dark")
    _joint(robot, "base", "base_link", "turret_link", (0, 0, 0), "0 0 1")

    # each pitch link's frame: origin at its joint, z along the link
    R = config.LINK_RADIUS
    upper = ET.SubElement(robot, "link", name="upper_arm_link")
    _inertial(upper, 0.1)
    _capsule(upper, (0, 0, 0), (0, 0, config.L1), R["upper arm"], "arm")
    _joint(robot, "shoulder", "turret_link", "upper_arm_link", (0, 0, config.H), "-1 0 0")

    fore = ET.SubElement(robot, "link", name="forearm_link")
    _inertial(fore, 0.1)
    _capsule(fore, (0, 0, 0), (0, 0, config.L2), R["forearm"], "arm")
    _joint(robot, "elbow", "upper_arm_link", "forearm_link", (0, 0, config.L1), "-1 0 0")

    hand = ET.SubElement(robot, "link", name="wrist_link")
    _inertial(hand, 0.1)
    _capsule(hand, (0, 0, 0), (0, 0, config.L3 * config.WRIST_PART), R["wrist"], "arm")
    # the camera bar rides on the wrist (it doesn't roll with the claw)
    w = config.CAMERA_HALF_WIDTH_MM
    cam = (0.0, -config.CAMERA_UP_MM, config.CAMERA_ALONG_MM)
    _capsule(hand, (cam[0] - w, cam[1], cam[2]), (cam[0] + w, cam[1], cam[2]), R["camera"], "dark")
    _joint(robot, "wrist", "forearm_link", "wrist_link", (0, 0, config.L2), "-1 0 0")

    claw = ET.SubElement(robot, "link", name="gripper_link")
    _inertial(claw, 0.05)
    _capsule(claw, (0, 0, config.L3 * config.WRIST_PART), (0, 0, config.L3), R["gripper"], "arm")
    _joint(robot, "roll", "wrist_link", "gripper_link", (0, 0, 0), "0 0 1")

    ET.SubElement(robot, "link", name="tip")
    _joint(robot, "tip_fixed", "gripper_link", "tip", (0, 0, config.L3), None, "fixed")

    # what the arm is kept clear of
    for ob in zone.obstacles:
        if getattr(ob, "half", None) is None:
            continue                                   # the turret is drawn on turret_link
        half = list(ob.half)
        center = list(ob.center)
        if ob.name == "table":                         # not 4 m wide in RViz, please
            half[0] = half[1] = 300.0
            center[1] = 100.0
        _box(robot, ob.name.replace(" ", "_"), center, half,
             "table" if ob.name == "table" else "box")
    return robot


def write(path=OUT, zone=None):
    root = build(zone)
    _indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def _indent(el, level=0):
    pad = "\n" + "  " * level
    if len(el):
        if not (el.text or "").strip():
            el.text = pad + "  "
        for child in el:
            _indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (el.tail or "").strip():
        el.tail = pad


if __name__ == "__main__":
    print("Wrote %s" % os.path.relpath(write(), os.getcwd()))
