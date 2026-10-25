#!/usr/bin/env python3
"""
Teach the look-at-board pose: where the arm waits, with the camera seeing the
whole board. The arm goes here every time run.py starts, and comes back here
after every job.

Servos off, you move the arm while watching the camera stream, Enter saves.
Saved to calibration/look_pose.json, plus a photo of the view in data/look/.
"""
import json
import os
import time

import config
from robot import kinematics, safety
from robot.arm import OUT_OF_RANGE_HINT, pose_problem, silent_servos


def load():
    """The saved pose (6 servo angles), or None if there isn't a usable one."""
    if not os.path.exists(config.LOOK_POSE_FILE):
        return None
    try:
        with open(config.LOOK_POSE_FILE) as fh:
            pose = [float(a) for a in json.load(fh)["servos"]]
    except (ValueError, KeyError, TypeError):
        print("  look_pose.json is unreadable -- ignoring it.")
        return None
    if len(pose) != 6 or _outside_limits(pose):
        print("  look_pose.json is outside the servo limits -- ignoring it.")
        return None
    return pose


def _outside_limits(angles):
    for servo_id, angle in enumerate(angles, start=1):
        low, high = config.SERVO_LIMITS[servo_id]
        if not (low <= angle <= high):
            return "servo %d is at %.0f, outside its safe range %.0f-%.0f" % (
                servo_id, angle, low, high)
    return None


def _problem(angles, zone):
    """Why this pose can't be the look pose, or None if it's fine."""
    outside = _outside_limits(angles)
    if outside:
        return outside + ". Move it back a little."
    if not safety.is_clear(angles, zone):
        return ("Too low: every part of the arm must stay %.0f mm above the board. "
                "Raise it." % config.CLEAR_MM)
    return None


def run(arm, camera=None):
    print("\n--- Teach the look-at-board pose ---")
    print("The servos switch off and you hold the arm where the camera sees the whole board.")
    if camera is None:
        print("  (camera off -- you won't be able to see what it sees)")
    else:
        print("  Watch what the camera sees: " + camera.url())
    input("HOLD THE ARM with one hand and press Enter. It will go limp. ")
    arm.limp()

    pose = None
    try:
        while True:
            ans = arm.live_prompt("\nMove it until the camera sees the WHOLE board, with a "
                                  "little margin.\n      Enter = save this pose   q = cancel   "
                                  "(the line below is live)").strip().lower()
            if ans == "q":
                break
            angles = arm.read_angles(samples=config.TEACH_READS)
            silent = silent_servos(angles)
            if silent:
                print("  %s didn't answer. %s" % (silent, OUT_OF_RANGE_HINT))
                continue
            if angles[4] is None:
                angles[4] = config.CENTER       # wrist roll didn't answer: face forward
            angles[5] = config.CLAW_OPEN
            problem = pose_problem(angles) or _problem(angles, arm.zone)
            if problem:
                print("  " + problem)
                continue
            pose = [round(a, 1) for a in angles]
            break
    except KeyboardInterrupt:
        raise                       # Ctrl-C: power stays off, nothing moves
    except Exception:
        arm.power_on_with_help()
        raise
    arm.power_on_with_help()

    if pose is None:
        print("Nothing saved.")
        return None

    arm.move_to_pose(pose)          # settle exactly on the saved angles
    photo = None
    if camera is not None:
        session = time.strftime("%Y%m%d-%H%M%S")
        path = camera.save_photo(os.path.join(config.DATA_DIR, "look", session + ".jpg"))
        if path:
            photo = os.path.relpath(path, config.ROOT)

    if not os.path.isdir(config.CALIBRATION_DIR):
        os.makedirs(config.CALIBRATION_DIR)
    x, y, z, _ = kinematics.forward(*pose[:4])
    with open(config.LOOK_POSE_FILE, "w") as fh:
        json.dump({"servos": pose,
                   "claw_tip_mm": [round(x, 1), round(y, 1), round(z, 1)],
                   "photo": photo,
                   "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, fh, indent=2)
    arm.home_pose = pose
    print("\nSaved: " + os.path.relpath(config.LOOK_POSE_FILE, config.ROOT)
          + "  (claw tip at x=%.0f y=%.0f z=%.0f mm)" % (x, y, z))
    if photo:
        print("View saved: " + photo)
    print("The arm will start here from now on.")
    return pose
