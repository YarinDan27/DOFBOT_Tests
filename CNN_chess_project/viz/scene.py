#!/usr/bin/env python3
"""
Everything the 3D view draws, as one JSON-ready dict -- built from the same
model the program moves by (robot/body.py, robot/world.py), so what you see
is what it checks against.

The page (viz/viewer.html) recomputes the arm's shape from the servo angles
with the same formulas, so it can also show poses you dial in yourself.
"""
import config
from robot import body, kinematics
from robot.board_frame import all_squares


def params():
    return {"H": config.H, "L1": config.L1, "L2": config.L2, "L3": config.L3,
            "wrist_part": config.WRIST_PART, "camera_along": config.CAMERA_ALONG_MM,
            "camera_up": config.CAMERA_UP_MM, "camera_half_width": config.CAMERA_HALF_WIDTH_MM,
            "radius": dict(config.LINK_RADIUS), "table_z": config.TABLE_Z,
            "clear_mm": config.CLEAR_MM, "hard_clear_mm": config.HARD_CLEAR_MM,
            "joint_range": {str(k): list(v) for k, v in config.JOINT_RANGE.items()},
            "exempt": [["upper arm", "turret"], ["gripper", "table"]]}


_REACH = {}


def _reach(frame, zone):
    """The reach map is slow-ish (it solves 128 poses): worked out once per board."""
    key = (id(frame), repr(zone.to_dict()) if zone is not None else None)
    if key not in _REACH:
        _REACH.clear()
        _REACH[key] = frame.reach_map(zone)
    return _REACH[key]


def board(frame, results=None, zone=None):
    """The 64 squares (centres in arm mm) with their reach and self-test status."""
    if frame is None:
        return None
    results = results or {}
    reach = _reach(frame, zone)
    squares = []
    for sq in all_squares():
        r = results.get(sq) or {}
        squares.append({"name": sq, "xyz": [round(c, 1) for c in frame.square_xyz(sq)],
                        "reach": reach[sq], "taught": sq in frame.taught,
                        "error_mm": (round(r["servo_error_mm"], 2)
                                     if r.get("status") == "measured" else None),
                        "status": r.get("status")})
    return {"origin": frame.origin, "file_step": frame.file_step, "rank_step": frame.rank_step,
            "square_mm": round(frame.square_size_mm()[0], 2), "squares": squares,
            "taught": {sq: [round(c, 1) for c in xyz] for sq, xyz in frame.taught.items()}}


def arm_state(arm):
    angles = list(arm.last_angles)
    tip = None
    if all(a is not None for a in angles[:4]):
        tip = [round(c, 1) for c in kinematics.forward(*angles[:4])[:3]]
    margin = None
    if all(a is not None for a in angles[:4]) and hasattr(arm.zone, "margin"):
        m, link, ob, _ = arm.zone.margin(angles)
        margin = {"mm": round(m + config.HARD_CLEAR_MM, 1), "link": link, "obstacle": ob}
    return {"angles": angles, "tip": tip, "margin": margin, "powered": arm.powered,
            "limp": arm.is_limp, "blocked": arm.blocked, "seen": arm.last_seen_t,
            "plan": arm.last_plan, "home": list(arm.home_pose)}


def build(zone, frame=None, arm=None, results=None, extra=None):
    out = {"version": config.VERSION, "params": params(), "world": zone.to_dict(),
           "board": board(frame, results, zone), "rest": list(config.POSE_REST),
           "ready": list(config.POSE_READY)}
    if arm is not None:
        out["arm"] = arm_state(arm)
    if extra:
        out.update(extra)
    return out


def shape(angles):
    """The arm's parts for one pose (for checking the page against Python)."""
    return {name: [list(a), list(b), r] for name, a, b, r in body.capsules(angles)}
