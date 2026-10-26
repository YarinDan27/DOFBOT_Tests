#!/usr/bin/env python3
"""
What's around the arm, and how close the arm's body (robot/body.py) gets to it.

Two kinds of danger:

  * the board (safety.Zone, unchanged since v0.2): pieces stand on it, so
    joint swings must stay CLEAR_MM above it; only planned straight lines
    may go down to a piece.
  * solid things the arm must never touch, in any pose or move:
        the table, the base plate, the turning base (turret), and the
        electronics (Jetson + driver board) behind the arm.
    Every link of the arm, with its thickness, must stay HARD_CLEAR_MM away
    from them.

This replaces the per-servo limits of v2.4 and before (shoulder 10-140 etc.),
which refused good poses -- like resting with the shoulder leaned back over
the electronics and the elbow and wrist folded -- while not knowing where the
electronics actually are. Now any combination of joints is allowed if the
arm's shape is clear, and refused if it isn't.

The electronics box is a guess (config.ELECTRONICS) until you measure it:
menu "Teach the electronics" touches its top corners with the claw and saves
calibration/electronics.json, which then replaces the guess.

Pure math, runs anywhere.
"""
import json
import math
import os

import config
from robot import body, safety


class Box:
    """A box. `axes` are three unit vectors (a turned box, like the board)."""

    def __init__(self, name, center, axes, half, clear):
        self.name = name
        self.center = [float(c) for c in center]
        self.axes = [[float(c) for c in a] for a in axes]
        self.half = [float(h) for h in half]
        self.clear = float(clear)

    @classmethod
    def between(cls, name, xr, yr, zr, clear):
        """Axis-aligned box from (low, high) ranges."""
        center = [(xr[0] + xr[1]) / 2.0, (yr[0] + yr[1]) / 2.0, (zr[0] + zr[1]) / 2.0]
        half = [(xr[1] - xr[0]) / 2.0, (yr[1] - yr[0]) / 2.0, (zr[1] - zr[0]) / 2.0]
        return cls(name, center, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], half, clear)

    def distance(self, p):
        """Signed distance from point p to the box surface (negative = inside)."""
        d = [p[i] - self.center[i] for i in range(3)]
        q = [abs(sum(d[i] * axis[i] for i in range(3))) - h
             for axis, h in zip(self.axes, self.half)]
        outside = math.sqrt(sum(max(c, 0.0) ** 2 for c in q))
        return outside + min(max(q), 0.0)

    def to_dict(self):
        return {"kind": "box", "name": self.name, "center": self.center,
                "axes": self.axes, "half": self.half, "clear": self.clear}


class Cylinder:
    """A standing cylinder (the turret)."""

    def __init__(self, name, center_xy, radius, z0, z1, clear):
        self.name = name
        self.cx, self.cy = float(center_xy[0]), float(center_xy[1])
        self.radius = float(radius)
        self.z0, self.z1 = float(z0), float(z1)
        self.clear = float(clear)

    def distance(self, p):
        dr = math.hypot(p[0] - self.cx, p[1] - self.cy) - self.radius
        dz = max(self.z0 - p[2], p[2] - self.z1)
        if dr <= 0.0 and dz <= 0.0:
            return max(dr, dz)
        return math.sqrt(max(dr, 0.0) ** 2 + max(dz, 0.0) ** 2)

    def to_dict(self):
        return {"kind": "cylinder", "name": self.name, "center": [self.cx, self.cy],
                "radius": self.radius, "z": [self.z0, self.z1], "clear": self.clear}


# Links that may touch an obstacle by design: the upper arm sits on the turret,
# and the claw fingers go down among the pieces near the table (the board
# zone looks after those).
EXEMPT = {("upper arm", "turret"), ("gripper", "table")}

SPACING = 6.0          # mm between checked points along each link


def electronics_box():
    """The measured electronics (calibration/electronics.json), or the config guess."""
    ranges = config.ELECTRONICS
    if os.path.exists(config.ELECTRONICS_FILE):
        try:
            with open(config.ELECTRONICS_FILE) as fh:
                saved = json.load(fh)
            ranges = {k: (float(saved[k][0]), float(saved[k][1])) for k in ("x", "y", "z")}
        except (OSError, ValueError, KeyError, TypeError, IndexError):
            pass
    return Box.between("electronics", ranges["x"], ranges["y"], ranges["z"],
                       config.HARD_CLEAR_MM)


def save_electronics(ranges):
    if not os.path.isdir(config.CALIBRATION_DIR):
        os.makedirs(config.CALIBRATION_DIR)
    with open(config.ELECTRONICS_FILE, "w") as fh:
        json.dump({k: [round(float(v), 1) for v in ranges[k]] for k in ("x", "y", "z")},
                  fh, indent=2)


def make_room_for(zone, angles, lowest_top=25.0):
    """
    You held the arm in this pose by hand, so the space it fills is free --
    the electronics can't be there. If the electronics box (a guess until
    measured) is what makes this pose "too close", lower its top until the
    pose is clear again, and save that. Returns the new top (mm) if it
    changed, None if nothing needed changing or lowering the top can't help.
    """
    m, link, ob, _ = zone.margin(angles)
    if m >= 0.0 or ob != "electronics":
        return None
    box = [o for o in zone.obstacles if o.name == "electronics"][0]
    ranges = {"x": (box.center[0] - box.half[0], box.center[0] + box.half[0]),
              "y": (box.center[1] - box.half[1], box.center[1] + box.half[1]),
              "z": (box.center[2] - box.half[2], box.center[2] + box.half[2])}
    top = ranges["z"][1]
    while top > lowest_top:
        top -= 1.0
        trial = Box.between("electronics", ranges["x"], ranges["y"], (ranges["z"][0], top),
                            box.clear)
        others = [o for o in zone.obstacles if o.name != "electronics"]
        if World(zone.polygon, zone.board_z, others + [trial]).margin(angles)[0] >= 0.0:
            zone.obstacles = others + [trial]
            ranges["z"] = (ranges["z"][0], top)
            save_electronics(ranges)
            return top
    return None


def solid_obstacles():
    big = 2000.0
    plate = config.BASE_PLATE
    return [
        Box.between("table", (-big, big), (-big, big), (config.TABLE_Z - 50.0, config.TABLE_Z),
                    5.0),
        Box.between("base plate", plate["x"], plate["y"], plate["z"], 5.0),
    ] + [Cylinder("turret", (0.0, 0.0), part["radius"], part["z"][0], part["z"][1], 5.0)
         for part in config.TURRET] + [electronics_box()]


class World(safety.Zone):
    """
    The board zone (all of safety.Zone) plus the solid obstacles. Everything
    in safety.py accepts a World where it accepted a Zone.
    """

    def __init__(self, polygon, board_z, obstacles=None):
        safety.Zone.__init__(self, polygon, board_z)
        self.obstacles = solid_obstacles() if obstacles is None else obstacles

    @classmethod
    def default(cls):
        zone = safety.Zone.default()
        return cls(zone.polygon, zone.board_z)

    @classmethod
    def from_board(cls, frame):
        zone = safety.Zone.from_board(frame)
        return cls(zone.polygon, zone.board_z)

    @classmethod
    def for_board(cls, frame):
        return cls.from_board(frame) if frame is not None else cls.default()

    def margin(self, angles):
        """
        How much room the arm has: (mm, link, obstacle, point). mm = the
        smallest (gap - required clearance) over every link and obstacle:
        negative = too close (or inside). Points every SPACING mm along each
        link; half a spacing is taken off to be safe between points.
        """
        best = (float("inf"), None, None, None)
        for link, point, radius in body.sample_points(angles, SPACING):
            for ob in self.obstacles:
                if (link, ob.name) in EXEMPT:
                    continue
                m = ob.distance(point) - radius - SPACING / 2.0 - ob.clear
                if m < best[0]:
                    best = (m, link, ob.name, point)
        return best

    def body_problem(self, angles):
        """Text if the arm's shape is too close to something solid, else None."""
        m, link, ob, _ = self.margin(angles)
        if m >= 0.0:
            return None
        gap = m + _clear_of(self, ob)
        if gap < 0:
            return "the %s would be inside the %s" % (link, ob)
        return "the %s would be only %.0f mm from the %s" % (link, gap, ob)

    def to_dict(self):
        return {"board_polygon": [list(p) for p in self.polygon], "board_z": self.board_z,
                "obstacles": [ob.to_dict() for ob in self.obstacles]}


def _clear_of(world, name):
    for ob in world.obstacles:
        if ob.name == name:
            return ob.clear
    return 0.0
