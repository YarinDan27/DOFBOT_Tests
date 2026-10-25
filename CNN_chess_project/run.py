#!/usr/bin/env python3
"""
DOFBOT chess -- start here:

    cd ~/YDscripts/CNN_chess_project && python3 run.py

No questions at start-up. It switches the arm's power on right where the arm
is (no jump), says where the claw is, and -- unless you press Ctrl-C within
3 seconds -- goes home slowly: up first, then across, in small steps, each
checked against what the servos report back. If a joint falls short
(something in the way), it stops and holds still.

Teaching is hands-free at the arm: start it here, walk over, and the arm
tells you what it's doing with beeps and its light (see sessions/teach.py).

Ctrl-C never moves the arm: it stops right where it is. Quitting leaves the
arm holding where it is.

This menu (and the sessions/ it starts) is the only code that connects the
arm, the camera and the game. robot/, vision/ and game/ never import each
other, so each can be tested on its own.
"""
import os
import signal
import sys
import time

import config
from robot import kinematics
from robot.arm import Arm, ArmBlocked, OutOfReach, UnsafeMove, other_arm_programs
from robot.board_frame import FILES, BoardFrame, is_square
from sessions import look_pose, self_test, teach


# ---------------- coloured board map ----------------
def _use_colour():
    try:
        return (sys.stdout.isatty() and os.environ.get("TERM", "dumb") != "dumb"
                and not os.environ.get("NO_COLOR"))
    except (AttributeError, ValueError):
        return False


COLOURS = {"good": "30;42", "ok": "30;43", "bad": "97;41", "out": "37;100", "new": ""}
CELL = 7


def _paint(text, kind, colour):
    text = text.center(CELL)
    if not colour or not COLOURS[kind]:
        return text
    return "\033[%sm%s\033[0m" % (COLOURS[kind], text)


def square_grade(sq, reach, result):
    """(text, kind) for one square of the map."""
    if reach[sq] == "--":
        return "--", "out"
    if result and result.get("status") == "measured":
        e = result["servo_error_mm"]
        kind = ("good" if e <= config.SELF_TEST_GOOD_MM else
                "ok" if e <= config.SELF_TEST_OK_MM else "bad")
        return "%.1f" % e, kind
    if result and result.get("status") in ("no_route", "servo_read_failed", "blocked"):
        return "fail", "bad"
    if reach[sq] == "low":
        return "low", "ok"
    return "?", "new"


def board_map(frame, results, colour=None):
    """
    The board as seen from behind the arm (rank 8 at the top). Each square:
    how far off the claw was when it hovered there (mm), coloured
    green = sure, yellow = usable, red = not trusted / failed, grey = out of
    reach, plain '?' = not tested yet. * = a square you taught.
    """
    colour = _use_colour() if colour is None else colour
    reach = frame.reach_map()
    lines = []
    for rank in range(8, 0, -1):
        cells = []
        for f in FILES:
            sq = f + str(rank)
            text, kind = square_grade(sq, reach, results.get(sq))
            if sq in frame.taught:
                text += "*"
            cells.append(_paint(text, kind, colour))
        lines.append(" %d %s" % (rank, "".join(cells)))
    lines.append("   " + "".join(f.center(CELL) for f in FILES))
    counts = {}
    for sq in reach:
        kind = square_grade(sq, reach, results.get(sq))[1]
        counts[kind] = counts.get(kind, 0) + 1
    lines.append("")
    lines.append("  Numbers = how many mm off the claw was when it hovered there.")
    lines.append("  %s sure, within %.0f mm: %d    %s usable, within %.0f mm: %d    "
                 "(%s = can grab but not hover)"
                 % (_paint("1.0", "good", colour), config.SELF_TEST_GOOD_MM,
                    counts.get("good", 0), _paint("3.5", "ok", colour), config.SELF_TEST_OK_MM,
                    counts.get("ok", 0), _paint("low", "ok", colour).strip()))
    lines.append("  %s not trusted: %d    %s out of reach: %d    %s not tested yet: %d    "
                 "* = you taught it"
                 % (_paint("fail", "bad", colour), counts.get("bad", 0),
                    _paint("--", "out", colour), counts.get("out", 0),
                    _paint("?", "new", colour), counts.get("new", 0)))
    return "\n".join(lines)


# ---------------- helpers ----------------
def banner():
    print("\n=== DOFBOT Chess v%s ===" % config.VERSION)
    print("Last fix: %s" % config.VERSION_NOTE)


def stop_other_programs():
    """
    Another copy of this program is stopped automatically. Yahboom's ROS driver
    runs as root, so it can only be reported -- it sends its own commands to
    the same servos and holds the camera.
    """
    for pid, what, ours in other_arm_programs():
        if ours:
            try:
                os.kill(pid, signal.SIGTERM)
                print("  Stopped %s." % what)
                time.sleep(0.5)
            except OSError as e:
                print("  Couldn't stop %s (%s) -- quit and run: kill %d" % (what, e, pid))
        else:
            print("\n  WARNING: %s is also driving the arm." % what)
            print("  Stop it (until the next reboot) with:  sudo pkill -f arm.launch")


def start_camera(state, retry=False):
    """Starts the camera the first time it's needed; carries on without it if it fails."""
    if state["camera"] is None and (retry or not state["camera_failed"]):
        try:
            from vision.camera import Camera
            camera = Camera()
            camera.start(stream=True)
            state["camera"] = camera
            state["camera_failed"] = False
        except Exception as e:
            state["camera_failed"] = True
            print("  Camera unavailable: %s" % e)
            print("  Carrying on without photos (option 5 tries again).")
    return state["camera"]


def where_is_claw(arm):
    angles = arm.read_angles(samples=2, ids=(1, 2, 3, 4))
    if any(a is None for a in angles[:4]):
        return "unknown (a joint can't report)"
    if max(abs(angles[i] - arm.home_pose[i]) for i in range(4)) <= 4.0:
        return "home"
    x, y, z, _ = kinematics.forward(*angles[:4])
    return "claw at x=%.0f y=%.0f z=%.0f mm" % (x, y, z)


def report_block(e):
    print("\n  STOPPED: %s." % e)
    print("  The arm is holding still where it stopped. Look at it -- when the way is "
          "clear, choose c.")
    print("  (Every move is in logs/motion/ -- send me the newest file.)")


def power_on(arm):
    """Power on in place once every joint reports. Ctrl-C = give up. True if powered."""
    if arm.powered:
        return True
    print("  Waiting for every joint to report, then the power comes on right where the "
          "arm is (Ctrl-C = give up):")
    try:
        ok = arm.power_on_when_readable()
    except KeyboardInterrupt:
        print("\n  Gave up -- the power is still off.")
        return False
    except ArmBlocked as e:
        report_block(e)
        return False
    arm.beep(1)
    print("  Power is on -- the arm holds itself where it is.")
    return ok


def do(arm, action, *args, **kwargs):
    """
    Runs a menu action. A blocked move, an unsafe one or Ctrl-C stops it
    cleanly -- the arm holds still -- instead of crashing the program.
    """
    try:
        return action(*args, **kwargs)
    except ArmBlocked as e:
        report_block(e)
    except (OutOfReach, UnsafeMove) as e:
        print("  " + str(e))
    except RuntimeError as e:
        print("  Stopped: %s." % e)
    except KeyboardInterrupt:
        print("\n  Stopped. The arm is holding still where it is.")
        if arm.is_limp:
            try:
                if arm.power_on_when_readable(timeout=2, show=False):
                    print("  (Power is back on where it was resting.)")
                else:
                    print("  Servo power is still OFF -- the arm is floppy.")
            except (ArmBlocked, KeyboardInterrupt):
                pass
    return None


def back_home(arm):
    """Rise straight up first, THEN swing home: never swing low."""
    arm.lift_clear()
    arm.go_home()
    print("  Home.")


def can_move(arm):
    """True if the arm may move now; otherwise says why (and how to fix it)."""
    if arm.blocked:
        print("\n  The arm is holding still because the last move was stopped. Look at it;")
        print("  when the way is clear, choose c.")
        return False
    if not arm.powered:
        return power_on(arm)
    return True


# ---------------- start-up: no questions ----------------
def start(arm):
    if arm.start_problem is not None:
        report_block(arm.start_problem)
        return
    if not arm.powered:
        angles = arm.read_angles(samples=2, ids=(1, 2, 3, 4))
        problem = arm.reading_problem(angles) or "a joint didn't answer"
        print("\n  Power isn't on yet: %s." % problem)
        if power_on(arm):
            # you may still be holding it: don't move until you choose to
            print("  Choose 2 when you're back, to go home.")
        else:
            print("  Choose 1 to teach anyway (that switches the power off), or fix the "
                  "joint by hand and choose 2.")
        return
    angles = arm.read_angles(samples=3)
    problem = arm.reading_problem(angles)
    if problem:
        arm.blocked = True
        print("\n  NOT MOVING: %s." % problem)
        print("  Look at the arm. If the readings are right after all, choose c.")
        return
    place = where_is_claw(arm)
    if place == "home":
        print("  The arm is home.")
        return
    print("  The arm is holding: %s." % place)
    try:
        for left in (3, 2, 1):
            print("\r  Going home in %d s -- Ctrl-C = stay here. " % left, end="", flush=True)
            time.sleep(1.0)
        print()
    except KeyboardInterrupt:
        print("\n  Staying here.")
        return
    do(arm, back_home, arm)


# ---------------- menu actions ----------------
def map_screen(arm, state):
    """The coloured map; then hover over a square or move a piece from here."""
    frame = state["frame"]
    if frame is None:
        print("\n  No board yet -- teach it first (option 1).")
        return
    file_mm, rank_mm = frame.square_size_mm()
    worst = max(frame.residuals().values() or [0.0])
    print("\nBoard: %d squares taught, squares %.1f mm, worst touch %.1f mm off"
          % (len(frame.taught), file_mm, worst))
    print(board_map(frame, self_test.latest_results(frame)))
    reach = frame.reach_map()
    hovering = False
    while True:
        try:
            raw = input("\n  A square = hover over it   two squares (e2 e4) = move that piece"
                        "   Enter = back: ").strip().lower()
        except EOFError:
            raw = ""
        if not raw:
            break
        squares = raw.replace("-", " ").split()
        if len(squares) not in (1, 2) or not all(is_square(s) for s in squares):
            print("  Type one square (e4) or two (e2 e4).")
            continue
        bad = [s for s in squares if reach[s] != "ok"]
        if bad:
            print("  The arm can't hover above %s (grey or 'low' on the map)." % " ".join(bad))
            continue
        if not can_move(arm):
            return
        if len(squares) == 1:
            result = do(arm, self_test.measure_square, arm, frame, squares[0],
                        start_camera(state), os.path.join(config.DATA_DIR, "hover"))
            if result is not None:
                hovering = True
                print("  Hovering over %s: %s." % (squares[0], self_test.describe(result)))
        else:
            src, dst = squares
            grab = config.GRAB_BELOW_TOP_MM

            def lower(xyz):
                return (xyz[0], xyz[1], xyz[2] - grab)

            def move():
                gripped = arm.move_piece(lower(frame.square_xyz(src)), lower(frame.square_xyz(dst)))
                print("  Moved %s -> %s. %s" % (src, dst, "Gripped something." if gripped
                                                 else "The claw closed on nothing."))
            hovering = True
            do(arm, move)
    if hovering and not arm.blocked:
        do(arm, back_home, arm)


def self_test_screen(arm, state):
    frame = state["frame"]
    if frame is None:
        print("\n  No board yet -- teach it first (option 1).")
        return
    if not can_move(arm):
        return
    results = do(arm, self_test.run, arm, frame, start_camera(state))
    print()
    print(board_map(frame, results if results is not None else self_test.latest_results(frame)))


def camera_screen(state):
    print("\n--- Camera test ---")
    camera = start_camera(state, retry=True)
    if camera is None:
        return
    print("  " + camera.stats())
    frame = camera.fresh_frame()
    if frame is None:
        print("  No new picture arrived -- the camera opened but isn't delivering frames.")
        return
    brightness = float(frame.mean())
    if brightness < 15:
        print("  The picture is almost black (brightness %.0f/255) -- lens covered?" % brightness)
    elif brightness > 240:
        print("  The picture is almost white (brightness %.0f/255) -- too much light?"
              % brightness)
    else:
        print("  Picture looks normal (brightness %.0f/255)." % brightness)
    path = camera.save_photo(os.path.join(config.DATA_DIR, "photos",
                                          time.strftime("%Y%m%d-%H%M%S") + ".jpg"))
    if path:
        print("  Photo saved: " + os.path.relpath(path, config.ROOT))
    if camera.server:
        print("  Live view (open on your laptop): " + camera.url())


def teach_screen(arm, state):
    new = do(arm, teach.run, arm, start_camera(state), state["frame"])
    if new is not None:
        state["frame"] = new
        arm.set_board(new)
        print()
        print(board_map(new, {}))
        print("  Option 4 measures every square and fills in this map.")


def look_screen(arm, state):
    do(arm, look_pose.run, arm, start_camera(state))


def status_line(arm, state):
    if arm.blocked:
        arm_text = "STOPPED -- holding still (c = carry on)"
    elif not arm.powered:
        arm_text = "power off"
    else:
        arm_text = where_is_claw(arm)
    frame = state["frame"]
    board = "not taught" if frame is None else "%d squares taught" % len(frame.taught)
    camera = "on" if state["camera"] is not None else "off"
    return "Arm: %s | Board: %s | Camera: %s" % (arm_text, board, camera)


MENU = [("1", "Teach the board", "hands-free at the arm -- put a pawn on each square"),
        ("2", "Go home", "look at the board"),
        ("3", "Board map", "which squares it's sure about; hover or move a piece"),
        ("4", "Self-test", "hovers over every square it can reach and measures itself"),
        ("5", "Camera test", "frame rate, brightness, a photo, live view link"),
        ("6", "Teach the camera view", "hands-free at the arm: where home is"),
        ("0", "Quit", "the arm stays where it is, holding")]


def main():
    banner()
    stop_other_programs()
    state = {"frame": BoardFrame.load(), "camera": None, "camera_failed": False}
    try:
        arm = Arm()
    except Exception as e:
        print("\nCan't talk to the arm: %s" % e)
        return
    with arm:
        arm.interactive = False          # never stop mid-move to ask
        arm.skip_park = True             # quitting (or a crash) never moves the arm
        arm.set_board(state["frame"])
        home = look_pose.load()
        if home:
            arm.home_pose = home
        start(arm)

        while True:
            print("\n" + status_line(arm, state))
            for key, name, note in MENU:
                print("  %s  %-22s %s" % (key, name, note))
            if arm.blocked:
                print("  c  Carry on             you looked at the arm and the way is clear")
            try:
                choice = input("Choose: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                print()
                break
            if choice == "1":
                teach_screen(arm, state)
            elif choice == "2":
                if can_move(arm):
                    do(arm, back_home, arm)
            elif choice == "3":
                map_screen(arm, state)
            elif choice == "4":
                self_test_screen(arm, state)
            elif choice == "5":
                camera_screen(state)
            elif choice == "6":
                look_screen(arm, state)
            elif choice == "c" and arm.blocked:
                arm.clear_block()
                print("  OK -- moves are allowed again. Nothing moved.")
            elif choice in ("0", "q"):
                break

    if state["camera"] is not None:
        state["camera"].stop()
    print("Bye! The arm is holding where it is.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped. (Ctrl-C never moves the arm.)")
