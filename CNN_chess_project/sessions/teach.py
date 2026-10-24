#!/usr/bin/env python3
"""
Teach the board by hand.

Servo power goes off, you push the claw onto a few squares, and at each one
the program:
  * reads the joint angles  -> where the claw is, in mm (via kinematics)
  * takes a photo           -> the camera can see the claw, so this is the
                               "this is exactly right" reference picture the
                               self-test compares its own footage against

Teach 4 or more squares and it checks your work: every square that disagrees
with the others by more than config.TEACH_MAX_RESIDUAL_MM is flagged, so you
redo just that one instead of starting over.
"""
import json
import os
import time

import config
from robot import kinematics
from robot.arm import OUT_OF_RANGE_HINT, pose_problem
from robot.board_frame import BoardFrame, format_grid, is_square


def _ask_squares():
    default = " ".join(config.TEACH_SQUARES)
    raw = input("Squares to teach [%s]: " % default).strip().lower()
    squares = raw.split() if raw else list(config.TEACH_SQUARES)
    bad = [s for s in squares if not is_square(s)]
    if bad:
        print("  Not squares: " + " ".join(bad))
        return None
    unique = []
    for s in squares:
        if s not in unique:
            unique.append(s)
    if len(unique) < 3:
        print("  Need at least three different squares (four or more is better).")
        return None
    return unique


def _record(arm, camera, square, photo_dir):
    """Reads where the claw is right now and photographs it."""
    angles = arm.read_angles(samples=config.TEACH_READS)
    problem = pose_problem(angles)
    if problem:
        print("  %s. %s" % (problem, OUT_OF_RANGE_HINT))
        print("  (That square may be beyond the arm's reach -- type s to skip it.)")
        return None
    x, y, z, _ = kinematics.forward(*angles[:4])
    photo = None
    if camera is not None:
        path = camera.save_photo(os.path.join(photo_dir, square + ".jpg"))
        if path:
            photo = os.path.relpath(path, config.ROOT)
    print("  %s: x=%.0f y=%.0f z=%.0f mm%s"
          % (square, x, y, z, "" if photo else "   (no photo)"))
    return {"xyz": [x, y, z], "servos": angles, "photo": photo}


def _review(taught):
    """Fits the board to everything taught so far and grades each touch."""
    frame = BoardFrame.fit({sq: d["xyz"] for sq, d in taught.items()})
    print("\n  Board found: squares %.1f mm" % frame.square_size_mm()[0])
    if len(taught) == 3:
        print("  (3 squares only give a weak check -- 4 or more is better)")

    residuals = frame.residuals()
    bad = []
    print("  How far each touch is from the board the others agree on:")
    for sq in taught:
        r = residuals[sq]
        flag = ""
        if r > config.TEACH_MAX_RESIDUAL_MM:
            flag = "   <- probably off, redo it"
            bad.append(sq)
        print("    %s  %4.1f mm%s" % (sq, r, flag))
    if not bad:
        print("  All touches agree.")
    return frame, bad


def _save_session(photo_dir, session, taught, frame):
    if not os.path.isdir(photo_dir):
        os.makedirs(photo_dir)
    with open(os.path.join(photo_dir, "teach.json"), "w") as fh:
        json.dump({"session": session,
                   "taught": taught,
                   "board": frame.to_dict()}, fh, indent=2)


def run(arm, camera=None):
    print("\n--- Teach the board by hand ---")
    print("The servos switch off and you push the claw onto each square yourself.")
    squares = None
    while squares is None:
        squares = _ask_squares()

    session = time.strftime("%Y%m%d-%H%M%S")
    photo_dir = os.path.join(config.DATA_DIR, "teach", session)
    if camera is None:
        print("  (camera off: no reference photos this time)")
    else:
        print("  Watch live: " + camera.url())

    print("\nPut a piece on each of: " + " ".join(s.upper() for s in squares))
    input("Then HOLD THE ARM with one hand and press Enter. It will go limp. ")
    arm.limp()

    taught = {}
    frame = None
    interrupted = False
    try:
        i = 0
        while i < len(squares):
            sq = squares[i]
            ans = input("\n[%d/%d] Open the claw around the piece on %s, at grabbing height.\n"
                        "      Enter = record   r = redo previous   s = skip: "
                        % (i + 1, len(squares), sq.upper())).strip().lower()
            if ans == "s":
                print("  skipped " + sq)
                i += 1
                continue
            if ans == "r":
                if i > 0:
                    i -= 1
                    taught.pop(squares[i], None)
                continue
            point = _record(arm, camera, sq, photo_dir)
            if point is not None:
                taught[sq] = point
                i += 1

        # review: fix single bad touches without starting over
        while True:
            frame = None
            if len(taught) >= 3:
                try:
                    frame, _ = _review(taught)
                except ValueError as e:
                    print("  " + str(e))
            else:
                print("\n  Only %d square(s) taught -- need at least 3." % len(taught))

            if frame is not None:
                choice = input("\nEnter = save   a square name = teach/redo it   "
                               "q = quit without saving: ").strip().lower()
            else:
                choice = input("\nType a square to teach it, or q to quit "
                               "without saving: ").strip().lower()
            if choice == "" and frame is not None:
                break
            if choice == "q":
                frame = None
                break
            if is_square(choice):
                input("  Put a piece on %s and open the claw around it at grabbing "
                      "height, then press Enter: " % choice.upper())
                point = _record(arm, camera, choice, photo_dir)
                if point is not None:
                    taught[choice] = point
            elif choice:
                print("  Didn't understand %r." % choice)
    except KeyboardInterrupt:
        interrupted = True          # Ctrl-C: power stays off, nothing moves
        raise
    finally:
        if not interrupted:
            arm.power_on_with_help()
            print("  Rising straight up, then going home...")
            try:
                arm.lift_clear()
                arm.go_home()
            except Exception as e:
                print("  Couldn't get home automatically (%s). It's holding where it is." % e)

    if frame is None:
        print("Nothing saved.")
        return None
    frame.save()
    _save_session(photo_dir, session, taught, frame)
    print("\nSaved: " + os.path.relpath(config.BOARD_FILE, config.ROOT))
    print("\nReach map (ok = can grab AND hover, low = grab only, -- = out of reach):")
    print(format_grid(frame.reach_map()))
    return frame
