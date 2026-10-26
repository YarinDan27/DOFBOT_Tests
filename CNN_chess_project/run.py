#!/usr/bin/env python3
"""
DOFBOT chess -- start here:

    cd ~/YDscripts/CNN_chess_project && python3 run.py

    python3 run.py --sim     the same program on a simulated arm: no arm needed
                             (try it on your PC; watch it in the 3D view)
    python3 run.py --check   reads the servos and says what it sees; moves nothing

No questions at start-up. It switches the arm's power on right where the arm
is (no jump), says where the claw is, and -- unless you press Ctrl-C within
3 seconds -- goes home: resting (folded back), or the home pose you taught.
Every move is planned against the arm's real shape -- the electronics box,
the turning base, the plate, the table and the board (robot/world.py) --
and checked against what the servos report back as it goes. If a joint falls
short (something in the way), it stops and holds still.

Teaching is hands-free at the arm: start it here, walk over, and the arm
tells you what it's doing with beeps and its light (see sessions/teach.py).
Or move it from the keyboard (option 5) and mark squares there.

Live 3D view of the arm while this runs: http://<jetson>:8081 (printed at
start). Everything printed is also saved in logs/session/ -- send me the
newest file if something goes wrong.

Ctrl-C never moves the arm: it stops right where it is. 0 folds the arm
back to rest, then quits; q quits leaving it where it is.

This menu (and the sessions/ it starts) is the only code that connects the
arm, the camera and the game. robot/, vision/ and game/ never import each
other, so each can be tested on its own.
"""
import os
import shutil
import signal
import socket
import sys
import time

import config

SIM = "--sim" in sys.argv
if SIM:
    # the pretend arm from the tests, and its own folders, so a simulated run
    # never touches the real calibration
    sys.path.insert(0, os.path.join(config.ROOT, "tests", "fakes"))
    _real_calibration = config.CALIBRATION_DIR
    config.CALIBRATION_DIR = os.path.join(config.ROOT, "sim", "calibration")
    config.DATA_DIR = os.path.join(config.ROOT, "sim", "data")
    config.LOGS_DIR = os.path.join(config.ROOT, "sim", "logs")
    config.BOARD_FILE = os.path.join(config.CALIBRATION_DIR, "board_arm.json")
    config.LOOK_POSE_FILE = os.path.join(config.CALIBRATION_DIR, "look_pose.json")
    config.ELECTRONICS_FILE = os.path.join(config.CALIBRATION_DIR, "electronics.json")
    if not os.path.isdir(config.CALIBRATION_DIR) and os.path.isdir(_real_calibration):
        shutil.copytree(_real_calibration, config.CALIBRATION_DIR)

from robot import kinematics                                   # noqa: E402
from robot.arm import (Arm, ArmBlocked, OutOfReach, SERVO_NAMES, UnsafeMove,  # noqa: E402
                       other_arm_programs)
from robot.board_frame import FILES, BoardFrame, is_square     # noqa: E402
from sessions import electronics, jog, look_pose, self_test, teach  # noqa: E402


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


def board_map(frame, results, colour=None, zone=None):
    """
    The board as seen from behind the arm (rank 8 at the top). Each square:
    how far off the claw was when it hovered there (mm), coloured
    green = sure, yellow = usable, red = not trusted / failed, grey = out of
    reach, plain '?' = not tested yet. * = a square you taught.
    """
    colour = _use_colour() if colour is None else colour
    reach = frame.reach_map(zone)
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
    print("\n=== DOFBOT Chess v%s%s ===" % (config.VERSION, "  (SIMULATED ARM)" if SIM else ""))
    print("Last fix: %s" % config.VERSION_NOTE)


class Tee(object):
    """Everything printed also goes to logs/session/<time>.txt."""

    def __init__(self, stream, path):
        self.stream = stream
        self.file = open(path, "a")

    def write(self, text):
        self.stream.write(text)
        try:
            self.file.write(text)
            self.file.flush()
        except Exception:
            pass

    def flush(self):
        self.stream.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


def start_session_log():
    try:
        folder = os.path.join(config.LOGS_DIR, "session")
        if not os.path.isdir(folder):
            os.makedirs(folder)
        path = os.path.join(folder, time.strftime("%Y%m%d-%H%M%S") + ".txt")
        sys.stdout = Tee(sys.stdout, path)
        return path
    except Exception:
        return None


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def start_view(arm, state):
    """The live 3D view (viz/live.py). Returns its address, or None."""
    try:
        from viz import live, scene
    except Exception:
        return None

    def get_scene():
        return scene.build(arm.zone, state["frame"], arm,
                           results=self_test.latest_results(state["frame"]),
                           extra={"presets": [{"name": "Resting (folded back)",
                                               "angles": list(config.POSE_REST)},
                                              {"name": "Home", "angles": list(arm.home_pose)}],
                                  "subtitle": "Live from the arm program v%s%s" % (
                                      config.VERSION, " (simulated arm)" if SIM else "")})
    try:
        state["view"] = live.start(get_scene)
    except OSError as e:
        print("  (3D view unavailable: port %d is busy -- %s)" % (config.VIEW_PORT, e))
        return None
    return "http://%s:%d" % ("localhost" if SIM else local_ip(), config.VIEW_PORT)


def _sudo_kill(pids):
    """Stops root processes (Yahboom's driver). sudo may ask for your password."""
    import subprocess
    try:
        return subprocess.call(["sudo", "kill"] + [str(p) for p in pids]) == 0
    except OSError:
        return False


def stop_other_programs():
    """
    Nothing else may drive the arm. Another copy of this program is stopped
    directly; Yahboom's ROS driver (root, started at every boot) holds the
    servos stiff and fights over the readings, so it's stopped with sudo --
    you may be asked for your password once.
    """
    found = other_arm_programs()
    for pid, what, ours in found:
        if ours:
            try:
                os.kill(pid, signal.SIGTERM)
                print("  Stopped %s." % what)
                time.sleep(0.5)
            except OSError as e:
                print("  Couldn't stop %s (%s) -- quit and run: kill %d" % (what, e, pid))
    ros = [pid for pid, what, ours in found if not ours]
    if not ros:
        return
    print("\n  Yahboom's ROS arm driver is running (it starts at every boot). It holds the")
    print("  arm stiff and fights this program over the servos -- stopping it now.")
    print("  (sudo may ask for your password.)")
    _sudo_kill(ros)
    for _ in range(10):
        time.sleep(0.5)
        left = [pid for pid, what, ours in other_arm_programs() if not ours]
        if not left:
            print("  Stopped it (until the next reboot).")
            return
    print("  WARNING: it's still running. Stop it with:  sudo pkill -f arm.launch")


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
    """
    Power on in place once every joint reports. A joint that can't report
    may need moving by hand -- and the arm may be stiff -- so after a few
    seconds: countdown, power OFF (go and hold it), move it until it beeps,
    power back on by itself. Ctrl-C = give up. True if powered.
    """
    if arm.powered:
        return True
    try:
        if arm.power_on_when_readable(timeout=3, show=False):
            arm.beep(1)
            print("  Power is on -- the arm holds itself where it is.")
            return True
        angles = arm.read_angles(samples=1, ids=(1, 2, 3, 4))
        problem = arm.reading_problem(angles) or "a joint didn't answer"
        print("\n  Power isn't on yet: %s." % problem)
        print("  So you can move it, its power goes OFF after the countdown: go and hold the")
        print("  arm. Move it until it beeps -- power is back on, it holds itself -- then let go.")
        teach.countdown(arm, config.TEACH_COUNTDOWN_S)
    except KeyboardInterrupt:
        print("\n  Cancelled -- nothing changed.")
        return False
    try:
        arm.power_on_when_readable(light=True)
    except KeyboardInterrupt:
        arm.light(0, 0, 0)
        print("\n  Gave up -- the power is still OFF, the arm is floppy.")
        return False
    except ArmBlocked as e:
        report_block(e)
        return False
    arm.beep(long=True)
    print("  Power is on -- the arm holds itself where it is.")
    return True


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
    """Home: rises straight up first if it's low over the board, then a planned move."""
    arm.go_home()
    print("  Home.")


def rest(arm):
    arm.move_to_pose(config.POSE_REST)
    print("  Resting.")


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
        if power_on(arm):
            # you may still be holding it: don't move until you choose to
            print("  Choose 2 when you're back, to go home.")
        else:
            print("  Choose 2 to try again, or 1 to teach the board.")
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
    print(board_map(frame, self_test.latest_results(frame), zone=arm.zone))
    reach = frame.reach_map(arm.zone)
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
    print(board_map(frame, results if results is not None else self_test.latest_results(frame),
                    zone=arm.zone))


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
    if SIM:
        print(SIM_HANDS)
        return
    new = do(arm, teach.run, arm, start_camera(state), state["frame"])
    if new is not None:
        state["frame"] = new
        arm.set_board(new)
        print()
        print(board_map(new, {}, zone=arm.zone))
        print("  Option 4 measures every square and fills in this map.")


def look_screen(arm, state):
    if SIM:
        print(SIM_HANDS)
        return
    do(arm, look_pose.run, arm, start_camera(state))


SIM_HANDS = ("\n  That one needs your hands on the real arm. On the simulated arm, use"
             "\n  5 Keyboard control instead (m marks the square under the claw).")


def keyboard_screen(arm, state):
    if not can_move(arm):
        return
    taught = do(arm, jog.run, arm, None, state["jog_taught"])
    if taught is None:
        return
    state["jog_taught"] = taught
    if len(taught) < 3:
        if taught:
            print("  %d square(s) marked -- mark 3 or more to make a board (kept until you "
                  "quit)." % len(taught))
        return
    try:
        new, bad = teach._review(taught)
    except ValueError as e:
        print("  " + str(e))
        return
    if bad:
        print("  Not saved: mark %s again (5, put the claw on it, m)." % " ".join(bad))
        for sq in bad:
            taught.pop(sq, None)
        return
    new.save()
    state["frame"] = new
    arm.set_board(new)
    print("\nSaved the board from %d marked squares." % len(taught))
    print(board_map(new, {}, zone=arm.zone))


def electronics_screen(arm, state):
    if SIM:
        print(SIM_HANDS)
        return
    do(arm, electronics.run, arm)


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
    line = "Arm: %s | Board: %s | Camera: %s" % (arm_text, board, camera)
    if state.get("view_url"):
        line += " | 3D: " + state["view_url"]
    return line


MENU = [("1", "Teach the board", "hands-free at the arm -- a pawn on each square"),
        ("2", "Go home", "resting (folded back), or the home pose you taught"),
        ("3", "Board map", "which squares it's sure about; hover or move a piece"),
        ("4", "Self-test", "hovers over every square it can reach, measures itself"),
        ("5", "Keyboard control", "move each joint or the claw; m marks a square"),
        ("6", "Camera test", "frame rate, brightness, a photo, live camera link"),
        ("7", "Teach the home pose", "hands-free at the arm: where it waits"),
        ("8", "Measure the electronics", "hands-free: touch the top of the black box"),
        ("0", "Rest and quit", "folds back to rest, then quits (q = quit, stay put)")]


def check():
    """--check: says what the arm reports and what the model makes of it. Moves nothing."""
    banner()
    print("  Python %s" % sys.version.split()[0])
    for pid, what, ours in other_arm_programs():
        print("  Also running: %s (pid %d)" % (what, pid))
    try:
        arm = Arm(power_on=False)
    except Exception as e:
        print("  Can't talk to the arm: %s" % e)
        return
    frame = BoardFrame.load()
    arm.set_board(frame)
    angles = arm.read_angles(samples=3)
    print("  Servos: " + "  ".join("%s %s" % (SERVO_NAMES[i + 1], "--" if a is None
                                                else "%.0f" % a) for i, a in enumerate(angles)))
    problem = arm.reading_problem(angles)
    if problem:
        print("  Problem: %s" % problem)
    else:
        x, y, z, approach = kinematics.forward(*angles[:4])
        m, link, ob, _ = arm.zone.margin(angles)
        print("  Claw tip x=%.0f y=%.0f z=%.0f mm, tilted %.0f deg from straight down"
              % (x, y, z, abs(180 - approach)))
        print("  Closest to anything solid: the %s, %.0f mm from the %s%s"
              % (link, m + config.HARD_CLEAR_MM, ob, " -- TOO CLOSE" if m < 0 else ""))
    print("  Board: %s" % ("not taught" if frame is None else
                           "%d squares taught, squares %.1f mm" % (len(frame.taught),
                                                                  frame.square_size_mm()[0])))
    home = look_pose.load()
    print("  Home pose: %s" % (home if home else "not taught (resting: %s)" % config.POSE_REST))
    print("  Electronics: %s" % ("measured (%s)" % config.ELECTRONICS_FILE
                                 if os.path.exists(config.ELECTRONICS_FILE)
                                 else "the estimate in config.py (not measured yet)"))
    videos = sorted(d for d in os.listdir("/dev") if d.startswith("video")) \
        if os.path.isdir("/dev") else []
    print("  Cameras: %s" % (" ".join("/dev/" + v for v in videos) or "none found"))
    arm.skip_park = True


def main():
    log_path = start_session_log()
    banner()
    stop_other_programs()
    state = {"frame": BoardFrame.load(), "camera": None, "camera_failed": False,
             "jog_taught": {}}
    if SIM:
        import Arm_Lib
        Arm_Lib.reset(config.POSE_REST)
    try:
        arm = Arm()
    except Exception as e:
        print("\nCan't talk to the arm: %s" % e)
        return
    with arm:
        arm.interactive = False          # never stop mid-move to ask
        arm.skip_park = True             # quitting (or a crash) never moves by itself
        arm.set_board(state["frame"])
        home = look_pose.load()
        if home:
            arm.home_pose = home
        state["view_url"] = start_view(arm, state)
        if state["view_url"]:
            print("Live 3D view: %s (open it in a browser on your laptop)" % state["view_url"])
        if log_path:
            print("Session log: %s" % os.path.relpath(log_path, config.ROOT))
        start(arm)

        while True:
            print("\n" + status_line(arm, state))
            for key, name, note in MENU:
                print("  %s  %-24s %s" % (key, name, note))
            if arm.blocked:
                print("  c  Carry on               you looked at the arm and the way is clear")
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
                keyboard_screen(arm, state)
            elif choice == "6":
                camera_screen(state)
            elif choice == "7":
                look_screen(arm, state)
            elif choice == "8":
                electronics_screen(arm, state)
            elif choice == "c" and arm.blocked:
                arm.clear_block()
                print("  OK -- moves are allowed again. Nothing moved.")
            elif choice == "0":
                if can_move(arm):
                    do(arm, rest, arm)
                break
            elif choice == "q":
                break

    if state["camera"] is not None:
        state["camera"].stop()
    if state.get("view"):
        state["view"].shutdown()
    print("Bye! The arm is holding where it is.")


if __name__ == "__main__":
    try:
        if "--check" in sys.argv:
            check()
        else:
            main()
    except KeyboardInterrupt:
        print("\nStopped. (Ctrl-C never moves the arm.)")
