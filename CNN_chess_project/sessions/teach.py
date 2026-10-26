#!/usr/bin/env python3
"""
Teach the board by hand -- hands-free at the arm.

You start it at the computer, walk over, and teach with the arm alone:

  1. The claw closes and the arm moves low over the board.
  2. Countdown: beeps every second for the last 5 s, then a LONG beep: power
     is off and the arm is floppy -- be holding it by then.
  3. For each square, in the order shown: put the CLOSED claw tip on top of
     the piece and HOLD STILL for 2 s. It records by itself:
     1 beep = first square recorded, 2 beeps = second, and so on.
  4. After the last square: a long beep -- power is back on and the arm holds
     itself. You can let go.

The arm's light while teaching:
  blue  = waiting for you      red = a joint can't report its angle: tilt the
  green = just recorded              claw a little until it turns blue

To stop early: lift the claw up high and hold still for 4 s. It switches
power back on right there (two beeps) and keeps what you've taught so far.

A joint bent more than config.WIDE_READ_DEG past the end of its servo's
0-180 scale can't report (red light) -- tilting the claw a little fixes it.
Only the claw tip's position is used.

Teach 4 or more squares and it checks your work: a touch that disagrees with
the others by more than config.TEACH_MAX_RESIDUAL_MM is flagged, and you redo
just that one.
"""
import json
import math
import os
import time

import config
from robot import kinematics
from robot.arm import ArmBlocked, FIX_BY_HAND, OutOfReach, SERVO_NAMES, UnsafeMove
from robot.board_frame import BoardFrame, is_square

BLUE = (0, 0, 80)
RED = (90, 0, 0)
GREEN = (0, 90, 0)
OFF = (0, 0, 0)


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _ask_squares():
    default = " ".join(config.TEACH_SQUARES)
    raw = input("Squares to teach, in this order [%s]: " % default).strip().lower()
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
        print("  Need at least three different squares (four or five is better).")
        return None
    return unique


def _guide(squares):
    print("\nAt the arm:")
    print("  * wait for the LONG beep -- be holding the arm by then (power goes off)")
    print("  * then, in this order: %s" % "  ".join(s.upper() for s in squares))
    print("    (a pawn on each of them -- the same kind of piece everywhere)")
    print("    put the CLOSED claw tip on top of the pawn, claw tilted a little, and")
    print("    HOLD STILL 2 s. Beeps = which square was recorded (1, 2, 3...).")
    print("  * red light = tilt the claw a little more; blue = waiting; green = got it")
    print("  * to stop early: lift the claw up high and hold still 4 s (two beeps)")
    print("  * long beep at the end = power is back on, you can let go")
    print("    (3 beeps + red light = it couldn't: lay the arm down gently)")


class _Stillness:
    """
    How long the claw tip has stayed within STILL_MM of where it settled.
    A servo reading wobbles by a degree, which moves the computed tip ~4 mm,
    so "still" is a circle wider than that, around the average of the tips
    since it settled.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.points = []
        self.centre = None
        self.since = None

    def update(self, tip, now):
        """Returns seconds held still (0.0 right after it moved)."""
        if self.centre is None or _distance(tip, self.centre) > config.STILL_MM:
            self.points = [list(tip)]
            self.centre = list(tip)
            self.since = now
            return 0.0
        self.points.append(list(tip))
        n = float(len(self.points))
        self.centre = [sum(p[i] for p in self.points) / n for i in range(3)]
        return now - self.since


class _Light:
    """The arm's RGB light; only talks to the board when the colour changes."""

    def __init__(self, arm):
        self.arm = arm
        self.colour = None

    def set(self, colour):
        if colour != self.colour:
            self.colour = colour
            self.arm.light(*colour)


def _show(line, state):
    print("\r" + line.ljust(state.get("width", 0)), end="", flush=True)
    state["width"] = max(state.get("width", 0), len(line))


def _end_line(state):
    if state.get("width"):
        print()
        state["width"] = 0


def countdown(arm, seconds):
    """Gives you time to walk to the arm, then switches its power off."""
    seconds = int(seconds)
    print()
    for left in range(seconds, 0, -1):
        print("\r  Power goes off in %2d s -- walk to the arm and hold it. (Ctrl-C to cancel)"
              % left, end="", flush=True)
        if left <= 5:
            arm.beep(1)
            time.sleep(max(0.0, 1.0 - 0.3))
        else:
            time.sleep(1.0)
    print("\r  Power OFF -- the arm is floppy now." + " " * 40)
    arm.limp()
    arm.beep(long=True)


def _move_low_over_board(arm, frame):
    """Before power goes off: close the claw and come down low, so it can't fall far."""
    try:
        arm.set_servo(6, config.CLAW_MAX, 500)
        time.sleep(0.5)
    except Exception:
        pass
    if frame is not None:
        x, y, z = frame.square_xyz(config.TEACH_START_SQUARE)
        target = (x, y, z + config.TEACH_START_ABOVE_MM)
    else:
        target = config.TEACH_START_XYZ
    try:
        arm.lift_clear()
        arm.goto(*target)
    except (OutOfReach, UnsafeMove, ArmBlocked, RuntimeError) as e:
        print("  (couldn't move low over the board first: %s -- staying here)" % e)


def _record(arm, camera, square, photo_dir):
    """Reads where the claw tip is right now (averaged) and photographs it."""
    angles = arm.read_angles(samples=config.TEACH_READS)
    if any(a is None for a in angles[:4]):
        return None
    x, y, z, approach = kinematics.forward(*angles[:4])
    photo = None
    if camera is not None:
        path = camera.save_photo(os.path.join(photo_dir, square + ".jpg"))
        if path:
            photo = os.path.relpath(path, config.ROOT)
    return {"xyz": [x, y, z], "servos": angles, "photo": photo,
            "tilt": abs(180.0 - approach)}


def collect(arm, squares, camera=None, photo_dir=None):
    """
    The hands-free part. The arm must already be limp. Records each square
    in order when the closed claw tip is held still on it (low, and moved on
    from the previous one). Returns (taught, how): how = 'done' | 'paused'
    (you lifted it high and held still) | 'idle' (nothing moved for a while).
    """
    taught = {}
    still = _Stillness()
    light = _Light(arm)
    screen = {}
    # Where the claw was when power went off: the first square only counts once
    # you've moved it from there (so a pause before you start, or the arm
    # sagging, isn't recorded as a square).
    start = arm.read_angles(samples=1, ids=(1, 2, 3, 4))
    last_tip = (None if any(a is None for a in start[:4])
                else list(kinematics.forward(*start[:4])[:3]))
    last_motion = time.time()
    i = 0
    while i < len(squares):
        square = squares[i]
        now = time.time()
        angles = arm.read_angles(samples=1, ids=(1, 2, 3, 4))
        silent = [s for s in (1, 2, 3, 4) if angles[s - 1] is None]
        if silent:
            light.set(RED)
            still.reset()
            _show("  [%d/%d] %s: %s can't report -- %s" % (
                i + 1, len(squares), square.upper(), SERVO_NAMES[silent[0]],
                FIX_BY_HAND[silent[0]]), screen)
        else:
            tip = list(kinematics.forward(*angles[:4])[:3])
            held = still.update(tip, now)
            if held == 0.0:
                last_motion = now
            if tip[2] > config.AUTO_MAX_Z_MM:
                light.set(BLUE)
                _show("  [%d/%d] claw is up high -- hold still %.0f s here to stop "
                      "teaching (%.1f s)" % (i + 1, len(squares), config.PAUSE_HOLD_S, held),
                      screen)
                if held >= config.PAUSE_HOLD_S:
                    _end_line(screen)
                    return taught, "paused"
            elif last_tip is not None and _distance(tip, last_tip) < config.AUTO_MIN_MOVE_MM:
                light.set(BLUE)
                _show("  [%d/%d] move the claw on to %s" % (i + 1, len(squares),
                                                           square.upper()), screen)
            else:
                light.set(BLUE)
                _show("  [%d/%d] %s: claw at x=%.0f y=%.0f z=%.0f -- hold still %.1f/%.0f s"
                      % (i + 1, len(squares), square.upper(), tip[0], tip[1], tip[2],
                         min(held, config.STILL_S), config.STILL_S), screen)
                if held >= config.STILL_S:
                    point = _record(arm, camera, square, photo_dir)
                    if point is not None:
                        taught[square] = point
                        last_tip = point["xyz"]
                        light.set(GREEN)
                        _end_line(screen)
                        print("  %s recorded: x=%.0f y=%.0f z=%.0f mm, claw tilt %.0f deg%s"
                              % (square.upper(), point["xyz"][0], point["xyz"][1],
                                 point["xyz"][2], point["tilt"],
                                 "" if point["photo"] or camera is None else " (no photo)"))
                        arm.beep(len(taught))
                        still.reset()
                        i += 1
                        continue
        if now - last_motion > config.TEACH_IDLE_S:
            _end_line(screen)
            return taught, "idle"
        time.sleep(config.TEACH_TICK_S)
    _end_line(screen)
    return taught, "done"


def _finish_at_arm(arm, how):
    """
    Power back on in place (the arm holds itself). Long beep = done, two
    beeps = stopped early. Three beeps + red light = it couldn't (still floppy).
    """
    if how == "idle":
        print("  Nothing moved for %.0f s -- stopping." % config.TEACH_IDLE_S)
    arm.light(*BLUE)
    try:
        ok = arm.power_on_when_readable(timeout=config.FINISH_POWER_ON_S, show=True)
    except ArmBlocked as e:
        arm.light(*OFF)
        arm.beep(2)
        print("  Power is on, but: %s." % e)
        print("  It's holding still. What you taught is kept.")
        return True
    if ok:
        arm.light(*OFF)
        if how == "paused":
            arm.beep(2)
        else:
            arm.beep(long=True)
        print("  Power is back on -- the arm holds itself where it is.")
    else:
        arm.light(*RED)
        arm.beep(3)
        print("  Couldn't switch power back on: a joint can't report its angle. The arm "
              "is still floppy -- lay it down gently.")
    return ok


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
        print("    %s  %4.1f mm%s" % (sq.upper(), r, flag))
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


def _teach_at_arm(arm, squares, camera, photo_dir, frame):
    """One trip to the arm: move low, countdown, record hands-free, power back on."""
    _guide(squares)
    if not arm.is_limp:
        _move_low_over_board(arm, frame)
    countdown(arm, config.TEACH_COUNTDOWN_S)
    taught, how = collect(arm, squares, camera, photo_dir)
    _finish_at_arm(arm, how)
    return taught


def run(arm, camera=None, frame=None):
    """Teach the board. Returns the new BoardFrame, or None if nothing was saved."""
    print("\n--- Teach the board by hand (hands-free at the arm) ---")
    squares = None
    while squares is None:
        squares = _ask_squares()
    session = time.strftime("%Y%m%d-%H%M%S")
    photo_dir = os.path.join(config.DATA_DIR, "teach", session)
    if camera is None:
        print("  (camera off: no reference photos this time)")

    taught = _teach_at_arm(arm, squares, camera, photo_dir, frame)
    new = None
    while True:
        new, bad = None, []
        if len(taught) >= 3:
            try:
                new, bad = _review(taught)
            except ValueError as e:
                print("  " + str(e))
        else:
            print("\n  Only %d square(s) recorded -- need at least 3." % len(taught))
        missing = [s for s in squares if s not in taught]
        if new is not None and not bad and not missing:
            break                      # all taught and they agree: save, no questions
        options = ["Enter = save" if new is not None else None,
                   "a square = teach/redo just that one",
                   "c = carry on with %s" % " ".join(s.upper() for s in missing) if missing else None,
                   "q = quit without saving"]
        choice = input("\n" + "   ".join(o for o in options if o) + ": ").strip().lower()
        if choice == "" and new is not None:
            break
        if choice == "q":
            new = None
            break
        again = None
        if choice == "c" and missing:
            again = missing
        elif is_square(choice):
            again = [choice]
        elif choice:
            print("  Didn't understand %r." % choice)
        if again:
            more = _teach_at_arm(arm, again, camera, photo_dir, frame)
            taught.update(more)

    if new is None:
        print("Nothing saved.")
        return None
    new.save()
    _save_session(photo_dir, session, taught, new)
    print("\nSaved the board: " + os.path.relpath(config.BOARD_FILE, config.ROOT))
    return new
