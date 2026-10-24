#!/usr/bin/env python3
"""
Self-test: the robot checks, on its own, how well it can reach each square.

For each square it hovers above it (no touching, so pieces are safe), waits
for the arm to settle, then measures:

  * servo error  -- reads its own joint angles back, works out where the claw
                    really is, and compares with where it was aiming. Catches
                    sagging, sticky or weak servos. Servos move in whole
                    degrees, so even a perfect arm shows ~1-2 mm here.
  * camera error -- NOT BUILT YET (vision/claw_check.py). Every photo is saved
                    now, so this footage can be scored the moment it exists,
                    against the teach photos of the same squares.

Result: a confidence map of the board, plus all the footage and a JSON report.
"""
import json
import math
import os
import time

import config
from robot import kinematics
from robot.arm import OutOfReach
from robot.board_frame import all_squares, format_grid
from vision import claw_check


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _reference_photo(square):
    """The newest teach photo of this square (claw placed there by hand), if any."""
    teach_dir = os.path.join(config.DATA_DIR, "teach")
    if not os.path.isdir(teach_dir):
        return None
    for session in sorted(os.listdir(teach_dir), reverse=True):
        path = os.path.join(teach_dir, session, square + ".jpg")
        if os.path.exists(path):
            return path
    return None


def _ask_squares():
    print("1. Quick: %d squares spread over the board (about a minute)"
          % len(config.QUICK_SQUARES))
    print("2. Every square (about 4 minutes)")
    choice = input("Select [1]: ").strip()
    return ("full", all_squares()) if choice == "2" else ("quick", list(config.QUICK_SQUARES))


def _summarise(results):
    cells = {}
    for sq, r in results.items():
        cells[sq] = "%.1f" % r["servo_error_mm"] if r["status"] == "measured" else "--"
    print("\nServo error per square, mm (-- = couldn't test):")
    print(format_grid(cells, width=6))

    measured = {sq: r["servo_error_mm"] for sq, r in results.items()
                if r["status"] == "measured"}
    summary = {"measured": len(measured)}
    if not measured:
        print("\nNothing could be measured.")
        return summary
    worst = max(measured, key=measured.get)
    mean = sum(measured.values()) / len(measured)
    good = sum(1 for e in measured.values() if e <= config.SELF_TEST_GOOD_MM)
    usable = sum(1 for e in measured.values()
                 if config.SELF_TEST_GOOD_MM < e <= config.SELF_TEST_OK_MM)
    bad = len(measured) - good - usable
    print("\nMeasured %d squares: mean %.1f mm, worst %.1f mm (%s)"
          % (len(measured), mean, measured[worst], worst))
    print("  confident (<= %.0f mm): %d   usable (<= %.0f mm): %d   not trusted: %d"
          % (config.SELF_TEST_GOOD_MM, good, config.SELF_TEST_OK_MM, usable, bad))
    skipped = sorted(sq for sq, r in results.items() if r["status"] != "measured")
    if skipped:
        print("  Couldn't test: " + " ".join(skipped))
    summary.update({"mean_mm": round(mean, 2), "worst_mm": round(measured[worst], 2),
                    "worst_square": worst, "confident": good, "usable": usable,
                    "not_trusted": bad, "skipped": skipped})
    return summary


def run(arm, frame, camera=None):
    if frame is None:
        print("\nTeach the board first (menu option 1).")
        return None
    print("\n--- Self-test ---")
    mode, squares = _ask_squares()
    reach = frame.reach_map()

    session = time.strftime("%Y%m%d-%H%M%S")
    photo_dir = os.path.join(config.DATA_DIR, "self_test", session)
    report_dir = os.path.join(config.LOGS_DIR, "self_test", session)
    if camera is None:
        print("  (camera off: measuring servos only, no footage)")

    print("\nThe arm hovers %.0f mm above each square -- no touching. Keep hands clear."
          % config.HOVER_MM)
    input("Press Enter to start (Ctrl-C stops it): ")

    camera_check_built = True
    results = {}
    try:
        for n, sq in enumerate(squares, start=1):
            tag = "  [%d/%d] %s:" % (n, len(squares), sq)
            if reach[sq] != "ok":
                results[sq] = {"status": "out_of_reach"}
                print(tag + " out of reach")
                continue
            x, y, z = frame.square_xyz(sq)
            target = (x, y, z + config.HOVER_MM)
            try:
                arm.goto(*target)
            except OutOfReach as e:
                results[sq] = {"status": "no_route", "why": str(e)}
                print(tag + " no safe route")
                continue

            time.sleep(config.SETTLE_S)
            angles = arm.read_angles(samples=3)
            if any(a is None for a in angles[:4]):
                results[sq] = {"status": "servo_read_failed"}
                print(tag + " a servo didn't answer")
                continue
            actual = kinematics.forward(*angles[:4])[:3]
            aimed = kinematics.solve_reachable(*target)[0]
            error = _distance(actual, target)

            photo, camera_error = None, None
            if camera is not None:
                path = camera.save_photo(os.path.join(photo_dir, sq + ".jpg"))
                if path:
                    photo = os.path.relpath(path, config.ROOT)
                    if camera_check_built:
                        try:
                            camera_error = claw_check.claw_offset_mm(
                                path, _reference_photo(sq))
                        except NotImplementedError:
                            camera_check_built = False

            results[sq] = {"status": "measured",
                           "target": list(target),
                           "actual": list(actual),
                           "servo_error_mm": error,
                           "servos_aimed": [aimed[i] for i in range(1, 5)],
                           "servos_read": angles,
                           "photo": photo,
                           "camera_error_mm": camera_error}
            print(tag + " off by %.1f mm" % error)
    except KeyboardInterrupt:
        print("\n  Stopped early.")
    finally:
        try:
            arm.lift_clear()
            arm.go_home()
        except Exception as e:
            print("  Couldn't get home automatically (%s). It's holding where it is." % e)

    summary = _summarise(results)
    photos = sum(1 for r in results.values() if r.get("photo"))
    if camera is not None:
        print("  Camera check: not built yet -- %d photos saved for it." % photos)

    if not os.path.isdir(report_dir):
        os.makedirs(report_dir)
    report_path = os.path.join(report_dir, "report.json")
    with open(report_path, "w") as fh:
        json.dump({"session": session, "mode": mode, "summary": summary,
                   "results": results, "board": frame.to_dict()}, fh, indent=2)
    print("  Report: " + os.path.relpath(report_path, config.ROOT))
    return results
