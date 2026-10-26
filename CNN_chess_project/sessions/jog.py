#!/usr/bin/env python3
"""
Keyboard control: move the arm yourself, a small step per key press.

    joints        q/a base     w/s shoulder   e/d elbow    r/f wrist
                  t/g wrist roll               [ ] claw open / close
    claw tip      arrows (or i/k/j/l): toward the board / back / left / right
                  (as you see it from behind the arm)   u/o: up/down
    step size     1 = fine   2 = medium   3 = big
    m             mark: record the claw tip as a square (teaching by keyboard)
    space         show where the arm is
    x or 0        back to the menu

Every step is checked before it happens, like every other move: the whole
arm against the electronics, base, table and board (robot/world.py), and the
servos are read back after it. A step that isn't safe is refused and the
arm doesn't move -- you're told why ("the forearm would be 4 mm from the
electronics"). Holding a key repeats the step; keys pressed while the arm
is still moving are dropped, so it never runs on after you let go.

Watch it move in the live 3D view (http://<jetson>:8081) as well as for real.
"""
import os
import sys
import time

import config
from robot import kinematics, safety
from robot.arm import ArmBlocked, OutOfReach, SERVO_NAMES, UnsafeMove
from robot.board_frame import is_square

STEPS = {"1": (1.0, 2.0), "2": (5.0, 10.0), "3": (12.0, 25.0)}     # degrees, mm
JOINT_KEYS = {"q": (1, +1), "a": (1, -1), "w": (2, -1), "s": (2, +1),
              "e": (3, -1), "d": (3, +1), "r": (4, -1), "f": (4, +1),
              "t": (5, +1), "g": (5, -1)}
TIP_KEYS = {"UP": "forward", "DOWN": "back", "LEFT": "left", "RIGHT": "right",
            "i": "forward", "k": "back", "j": "left", "l": "right", "u": "up", "o": "down"}
HELP = ("  joints: q/a base  w/s shoulder  e/d elbow  r/f wrist  t/g roll  [ ] claw\n"
        "  claw tip: arrows or i/k/j/l, u/o up/down   step: 1 fine 2 medium 3 big\n"
        "  m = mark this spot as a square   space = where am I   x = back to the menu")


class Keys:
    """
    Single key presses from the terminal (no Enter). Not a terminal (tests,
    pipes): reads a line and hands out its characters one by one.
    """

    def __init__(self):
        self.raw = False
        self.pending = []
        try:
            self.raw = sys.stdin.isatty()
        except (AttributeError, ValueError):
            self.raw = False
        self.saved = None

    def __enter__(self):
        if self.raw:
            import termios
            import tty
            fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        return self

    def __exit__(self, *exc):
        self.restore()
        return False

    def restore(self):
        if self.raw and self.saved is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.saved)
            self.saved = None

    def resume(self):
        if self.raw and self.saved is None:
            self.__enter__()

    def flush(self):
        """Drop keys typed while the arm was moving (held-key repeats)."""
        if self.raw:
            import termios
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)

    def get(self):
        if not self.raw:
            while not self.pending:
                line = input("keys> ")
                self.pending = list(line) if line else ["x"]
            return self.pending.pop(0)
        data = os.read(sys.stdin.fileno(), 3).decode("utf-8", "ignore")
        arrows = {"\x1b[A": "UP", "\x1b[B": "DOWN", "\x1b[C": "RIGHT", "\x1b[D": "LEFT"}
        if data in arrows:
            return arrows[data]
        if data == "\x1b":
            return "x"
        return data[:1]

    def line(self, prompt):
        """A whole typed line (for a square name), with normal echo."""
        self.restore()
        try:
            return input(prompt)
        finally:
            self.resume()


def describe(arm, angles):
    joints = "  ".join("%s %.0f" % (SERVO_NAMES[s], angles[s - 1]) for s in (1, 2, 3, 4))
    x, y, z, approach = kinematics.forward(*angles[:4])
    room = ""
    if hasattr(arm.zone, "margin"):
        m, link, ob, _ = arm.zone.margin(angles)
        room = "   room: %.0f mm (%s to %s)" % (m + config.HARD_CLEAR_MM, link, ob)
    return "  %s | claw x=%.0f y=%.0f z=%.0f, tilt %.0f%s" % (
        joints, x, y, z, abs(180.0 - approach), room)


def _tip_target(angles, direction, mm):
    """Where the claw tip should go, as you see it standing behind the arm:
    forward = toward the board (+y), left = -x, up = +z."""
    x, y, z, approach = kinematics.forward(*angles[:4])
    move = {"forward": (0.0, 1.0, 0.0), "back": (0.0, -1.0, 0.0),
            "left": (-1.0, 0.0, 0.0), "right": (1.0, 0.0, 0.0),
            "up": (0.0, 0.0, 1.0), "down": (0.0, 0.0, -1.0)}[direction]
    return (x + move[0] * mm, y + move[1] * mm, z + move[2] * mm), approach


def step_joint(arm, angles, sid, sign, deg):
    target = list(angles)
    target[sid - 1] = angles[sid - 1] + sign * deg
    low, high = config.JOINT_RANGE[sid]
    if not low <= target[sid - 1] <= high:
        return None, "the %s is at the end of its range (%.0f-%.0f)" % (SERVO_NAMES[sid], low, high)
    if sid >= 5:
        arm.set_servo(sid, target[sid - 1], 200)
        time.sleep(0.25)
        return target, None
    return _go(arm, angles, target)


def step_tip(arm, angles, direction, mm):
    (x, y, z), approach = _tip_target(angles, direction, mm)
    if z < arm.zone.board_z:
        return None, "the claw is at the board -- it won't push into it"
    sol, _ = arm._next_step(x, y, z, angles[:4], approach, arm.zone)
    if sol is None:
        return None, "the claw can't go %s from here without something hitting" % direction
    target = [sol[1], sol[2], sol[3], sol[4]] + list(angles[4:])
    return _go(arm, angles, target)


def _go(arm, angles, target):
    """One checked step from `angles` to `target` (servos 1-4)."""
    ok, where = safety.check_transition(angles[:4], target[:4], arm.zone)
    if not ok:
        why = arm.zone.body_problem(target) if hasattr(arm.zone, "body_problem") else None
        return None, why or ("that would sweep low over the board near (%.0f, %.0f, %.0f)"
                             % tuple(where))
    arm._run_stages(angles, [{sid: target[sid - 1] for sid in (1, 2, 3, 4)}])
    return target, None


def run(arm, camera=None, taught=None):
    """
    Keyboard control. `taught` (a dict) collects squares marked with m; it's
    returned so run.py can fit a board from them. Needs the power on.
    """
    taught = {} if taught is None else taught
    print("\n--- Keyboard control ---")
    print(HELP)
    if arm.blocked:
        print("  The arm is holding still after a stopped move -- choose c in the menu first.")
        return taught
    try:
        angles = arm.current_angles()
    except RuntimeError as e:
        print("  Can't start: %s." % e)
        return taught
    angles = [a if a is not None else config.CENTER for a in angles]
    size = "2"
    print(describe(arm, angles))
    with Keys() as keys:
        while True:
            key = keys.get()
            if key in ("x", "0", "X"):
                break
            if key in STEPS:
                size = key
                print("  step: %.0f degrees / %.0f mm" % STEPS[size])
                continue
            deg, mm = STEPS[size]
            result, why = None, None
            try:
                if key in JOINT_KEYS:
                    sid, sign = JOINT_KEYS[key]
                    result, why = step_joint(arm, angles, sid, sign, deg)
                elif key in TIP_KEYS:
                    result, why = step_tip(arm, angles, TIP_KEYS[key], mm)
                elif key in ("[", "]"):
                    claw = config.CLAW_OPEN if key == "[" else config.CLAW_MAX
                    arm.set_servo(6, claw, 300)
                    time.sleep(0.35)
                    angles[5] = claw
                    continue
                elif key == "m":
                    _mark(arm, camera, keys, taught)
                    continue
                elif key == " ":
                    print(describe(arm, arm.current_angles()))
                    continue
                elif key in ("?", "h"):
                    print(HELP)
                    continue
                else:
                    continue
            except ArmBlocked as e:
                print("\n  STOPPED: %s. The arm is holding still -- leave with x, look at it,"
                      " then c in the menu." % e)
                break
            except (OutOfReach, UnsafeMove, RuntimeError) as e:
                why = str(e)
            if why:
                print("  not moving: %s" % why)
            elif result is not None:
                angles = [float(a) for a in result]
                print(describe(arm, angles))
            keys.flush()
    return taught


def _mark(arm, camera, keys, taught):
    from sessions import teach
    name = keys.line("  Square under the claw tip (e.g. e4), Enter = cancel: ").strip().lower()
    if not name:
        return
    if not is_square(name):
        print("  %r isn't a square." % name)
        return
    point = teach._record(arm, camera, name, os.path.join(config.DATA_DIR, "teach", "keyboard"))
    if point is None:
        print("  A joint didn't answer -- try again.")
        return
    taught[name] = point
    print("  %s recorded at x=%.0f y=%.0f z=%.0f (%d marked so far%s)" % (
        name.upper(), point["xyz"][0], point["xyz"][1], point["xyz"][2], len(taught),
        "" if len(taught) >= 3 else "; 3 or more make a board"))
