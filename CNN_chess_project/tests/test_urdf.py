#!/usr/bin/env python3
"""
v2.6: the URDF (viz/urdf.py) is the same arm as robot/body.py -- the model
the safety checks use. Walks the URDF's joint chain for many poses and
compares every joint, the hand, the tip and the camera bar with body.py.

Nothing real moves:   python3 tests/test_urdf.py
"""
import math
import os
import random
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import config                                     # noqa: E402

TMP = tempfile.mkdtemp()
config.CALIBRATION_DIR = os.path.join(TMP, "calibration")
config.ELECTRONICS_FILE = os.path.join(config.CALIBRATION_DIR, "electronics.json")

from robot import body                            # noqa: E402
from viz import urdf                              # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


# ---------- a tiny forward-kinematics walker for URDF ----------
def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def translate(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rotate(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
            [0, 0, 0, 1]]


def apply(T, p):
    return tuple(sum(T[i][k] * v for k, v in enumerate(list(p) + [1.0])) for i in range(3))


def frames(root, values):
    """{link name: 4x4 transform from base_link} for joint values in radians."""
    joints = {}
    for j in root.findall("joint"):
        joints[j.find("child").get("link")] = j
    out = {"base_link": translate(0, 0, 0)}

    def frame(link):
        if link in out:
            return out[link]
        j = joints[link]
        T = frame(j.find("parent").get("link"))
        o = j.find("origin")
        xyz = [float(v) * 1000.0 for v in o.get("xyz").split()]
        assert o.get("rpy") == "0 0 0"
        T = matmul(T, translate(*xyz))
        if j.get("type") != "fixed":
            axis = [float(v) for v in j.find("axis").get("xyz").split()]
            T = matmul(T, rotate(axis, values[j.get("name")]))
        out[link] = T
        return T

    for link in joints:
        frame(link)
    return out


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# =====================================================================
print("--- the URDF ---")
path = urdf.write(os.path.join(TMP, "arm.urdf"))
root = ET.parse(path).getroot()
check("it's written and parses", root.tag == "robot" and root.get("name") == "dofbot_chess")
names = [j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"]
check("five moving joints: base, shoulder, elbow, wrist, roll", names == list(urdf.JOINTS),
      ", ".join(names))
ok = True
for j in root.findall("joint"):
    if j.get("type") != "revolute":
        continue
    lim = j.find("limit")
    lo, hi = float(lim.get("lower")), float(lim.get("upper"))
    sid = urdf.JOINTS.index(j.get("name")) + 1
    a, b = config.JOINT_RANGE[sid]
    # both ends of the servo range land on the joint limits (in some order)
    ends = sorted([urdf.joint_values([a] * 5)[j.get("name")], urdf.joint_values([b] * 5)[j.get("name")]])
    ok = ok and abs(ends[0] - lo) < 1e-3 and abs(ends[1] - hi) < 1e-3
check("joint limits are config.JOINT_RANGE", ok)
links = {link.get("name") for link in root.findall("link")}
check("the electronics, base plate and table are in it",
      {"electronics", "base_plate", "table"} <= links)

# =====================================================================
print("\n--- the same arm as robot/body.py, pose by pose ---")
rng = random.Random(3)
poses = [list(config.POSE_REST), list(config.POSE_READY), [86, 182, 18, -2, 91],
         [90, 190, 0, 0, 90], [90, 190, 180, 180, 90], [45, 60, 120, 30, 10]]
poses += [[rng.uniform(*config.JOINT_RANGE[s]) for s in range(1, 6)] for _ in range(60)]
worst, where = 0.0, ""
for pose in poses:
    F = frames(root, urdf.joint_values(pose))
    P = body.joint_points(pose)
    wl = F["wrist_link"]
    w = config.CAMERA_HALF_WIDTH_MM
    cam = (0.0, -config.CAMERA_UP_MM, config.CAMERA_ALONG_MM)
    pairs = {
        "shoulder": apply(F["upper_arm_link"], (0, 0, 0)),
        "elbow": apply(F["forearm_link"], (0, 0, 0)),
        "wrist": apply(wl, (0, 0, 0)),
        "hand": apply(wl, (0, 0, config.L3 * config.WRIST_PART)),
        "tip": apply(F["tip"], (0, 0, 0)),
        "camera_a": apply(wl, (cam[0] - w, cam[1], cam[2])),
        "camera_b": apply(wl, (cam[0] + w, cam[1], cam[2])),
    }
    for name, p in pairs.items():
        d = dist(p, P[name])
        if d > worst:
            worst, where = d, "%s at %s" % (name, [round(a) for a in pose])
check("every joint, the hand, the tip and the camera bar agree (66 poses)", worst < 0.01,
      "worst %.4f mm, %s" % (worst, where))

# the claw turning (servo 5) doesn't move the tip or the camera, as in body.py
a = frames(root, urdf.joint_values([90, 90, 90, 90, 0]))
b = frames(root, urdf.joint_values([90, 90, 90, 90, 180]))
check("turning the claw moves nothing else", dist(apply(a["tip"], (0, 0, 0)),
                                                apply(b["tip"], (0, 0, 0))) < 1e-6)

# capsule thicknesses
radii = {}
for link in root.findall("link"):
    for col in link.findall("collision"):
        cyl = col.find("geometry/cylinder")
        if cyl is not None:
            radii.setdefault(link.get("name"), []).append(round(float(cyl.get("radius")) * 1000, 1))
R = config.LINK_RADIUS
check("each part is as thick as body.py says",
      radii.get("upper_arm_link") == [R["upper arm"]] and radii.get("forearm_link") == [R["forearm"]]
      and sorted(radii.get("wrist_link", [])) == sorted([R["wrist"], R["camera"]])
      and radii.get("gripper_link") == [R["gripper"]], str(radii))

print("\n%d failure(s)" % len(failures))
if failures:
    for f in failures:
        print("  - " + f)
    sys.exit(1)
