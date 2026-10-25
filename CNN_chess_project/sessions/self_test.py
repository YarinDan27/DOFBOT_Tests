#!/usr/bin/env python3
"""
Self-test: the robot checks, on its own, how well it can reach each square.

No questions: it hovers above every square it can reach (no touching, so
pieces are safe), row by row, and for each one measures:

  * servo error  -- reads its own joint angles back, works out where the claw
                    really is, and compares with where it was aiming. Catches
                    sagging, sticky or weak servos. Servos move in whole
                    degrees, so even a perfect arm shows ~1-2 mm here.
  * a photo      -- for the camera check (vision/claw_check.py, not built
                    yet), scored later against the teach photos.

Ctrl-C stops it; whatever was measured is kept, and the next run adds to it.
The results feed the coloured board map in run.py (option 3).
"""
import json
import math
import os
import time

import config
from robot import kinematics
from robot.arm import ArmBlocked, OutOfReach, UnsafeMove, silent_servos
from robot.board_frame import FILES


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def test_order(frame):
    """Every square the arm can hover over, row by row in a snake (short moves)."""
    reach = frame.reach_map()
    order = []
    for rank in range(1, 9):
        row = [f + str(rank) for f in FILES]
        if rank % 2 == 0:
            row.reverse()
        order += [sq for sq in row if reach[sq] == "ok"]
    return order


def measure_square(arm, frame, sq, camera=None, photo_dir=None):
    """
    Hovers over one square, measures how far off the claw is (servo
    read-back), photographs it. Returns a result dict. ArmBlocked passes
    through: the caller decides what to do.
    """
    x, y, z = frame.square_xyz(sq)
    target = (x, y, z + config.HOVER_MM)
    try:
        arm.goto(*target)
    except (OutOfReach, UnsafeMove) as e:
        return {"status": "no_route", "why": str(e)}
    time.sleep(config.SETTLE_S)
    angles = arm.read_angles(samples=3)
    silent = silent_servos(angles)
    if silent:
        return {"status": "servo_read_failed", "why": silent}
    actual = kinematics.forward(*angles[:4])[:3]
    photo = None
    if camera is not None and photo_dir:
        path = camera.save_photo(os.path.join(photo_dir, sq + ".jpg"))
        if path:
            photo = os.path.relpath(path, config.ROOT)
    return {"status": "measured", "target": list(target), "actual": list(actual),
            "servo_error_mm": _distance(actual, target), "servos_read": angles,
            "photo": photo}


def describe(result):
    status = result.get("status")
    if status == "measured":
        return "off by %.1f mm" % result["servo_error_mm"]
    if status == "no_route":
        return "no safe route"
    if status == "servo_read_failed":
        return "%s didn't answer" % result.get("why")
    if status == "blocked":
        return "BLOCKED"
    return status or "?"


def save_report(session, mode, results, frame):
    report_dir = os.path.join(config.LOGS_DIR, "self_test", session)
    if not os.path.isdir(report_dir):
        os.makedirs(report_dir)
    path = os.path.join(report_dir, "report.json")
    with open(path, "w") as fh:
        json.dump({"session": session, "mode": mode, "results": results,
                   "board": frame.to_dict()}, fh, indent=2)
    return path


def _same_board(saved, frame):
    """A report only counts for the board it was measured on (re-teaching = start over)."""
    try:
        return _distance(saved["origin"], frame.origin) < 2.0 and \
            _distance(saved["file_step"], frame.file_step) < 1.0
    except (KeyError, TypeError, IndexError):
        return False


def latest_results(frame):
    """{square: result} from the newest self-test of THIS board, or {}."""
    folder = os.path.join(config.LOGS_DIR, "self_test")
    if frame is None or not os.path.isdir(folder):
        return {}
    for session in sorted(os.listdir(folder), reverse=True):
        path = os.path.join(folder, session, "report.json")
        try:
            with open(path) as fh:
                report = json.load(fh)
        except (OSError, ValueError):
            continue
        if _same_board(report.get("board", {}), frame):
            return report.get("results", {})
        return {}
    return {}


def run(arm, frame, camera=None, squares=None):
    """
    Hovers over `squares` (default: every reachable square) and measures each.
    Returns all results for this board (old + new). ArmBlocked is re-raised
    after the results are saved.
    """
    squares = squares or test_order(frame)
    session = time.strftime("%Y%m%d-%H%M%S")
    photo_dir = os.path.join(config.DATA_DIR, "self_test", session)
    print("\n--- Self-test: hovering %.0f mm above %d squares, no touching. "
          "Ctrl-C stops it. ---" % (config.HOVER_MM, len(squares)))
    if camera is None:
        print("  (camera off: measuring servos only, no photos)")

    results = {}
    blocked = None
    stopped = False
    try:
        for n, sq in enumerate(squares, start=1):
            try:
                result = measure_square(arm, frame, sq, camera, photo_dir)
            except ArmBlocked as e:
                results[sq] = {"status": "blocked", "why": str(e)}
                blocked = e
                print("  [%d/%d] %s: BLOCKED" % (n, len(squares), sq))
                break
            results[sq] = result
            print("  [%d/%d] %s: %s" % (n, len(squares), sq, describe(result)))
    except KeyboardInterrupt:
        stopped = True
        print("\n  Stopped by you. The arm is holding still where it is.")

    merged = latest_results(frame)
    merged.update(results)
    path = save_report(session, "board", merged, frame)
    print("  Saved: " + os.path.relpath(path, config.ROOT))
    if blocked is not None:
        raise blocked
    if not stopped:
        try:
            arm.lift_clear()
            arm.go_home()
        except (OutOfReach, UnsafeMove, RuntimeError) as e:
            print("  Couldn't get home automatically (%s). It's holding where it is." % e)
    return merged
