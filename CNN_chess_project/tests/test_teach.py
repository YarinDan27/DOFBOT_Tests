#!/usr/bin/env python3
"""
Hands-free teaching, tested with a pretend hand on the simulated arm
(tests/fakes/Arm_Lib.py) and a pretend clock -- nothing moves, runs in
seconds, safe on the laptop or the Jetson:

    python3 tests/test_teach.py

The "hand" moves the floppy arm onto each square (claw tilted, on top of a
pawn) and holds still, the way you do at the real arm. Also covers: the
wrist going silent (red light), stopping early by lifting high, walking
away (idle), redoing one bad square, the look-at-board view, and run.py's
start-up and menu (no questions at start-up).
"""
import builtins
import io
import os
import random
import sys
import tempfile
import time
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))     # the simulated Arm_Lib, first
sys.path.insert(1, os.path.dirname(HERE))

import Arm_Lib                                                # noqa: E402 (the fake)
import config                                                 # noqa: E402

assert "fakes" in Arm_Lib.__file__, "refusing to run against the real arm"
TMP = tempfile.mkdtemp()
# every file this test writes goes to a temp folder -- set BEFORE importing
# modules whose defaults read these paths
config.LOGS_DIR = os.path.join(TMP, "logs")
config.LOCK_FILE = os.path.join(TMP, "arm.lock")
config.DATA_DIR = os.path.join(TMP, "data")
config.CALIBRATION_DIR = os.path.join(TMP, "calibration")
config.BOARD_FILE = os.path.join(config.CALIBRATION_DIR, "board_arm.json")
config.LOOK_POSE_FILE = os.path.join(config.CALIBRATION_DIR, "look_pose.json")

from robot import arm as arm_mod                              # noqa: E402
from robot import kinematics                                  # noqa: E402
from robot.board_frame import BoardFrame                      # noqa: E402
from sessions import look_pose, self_test, teach              # noqa: E402
import run as run_mod                                         # noqa: E402

random.seed(5)
failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


# ---------------- pretend clock + hand ----------------
CLOCK = {"now": 1000.0, "limit": None}


class Hand:
    """
    Moves the arm while its power is off, following a script of
    (seconds, pose) steps that starts when the power first goes off.
    pose None = leave it where it is. Does nothing while the power is on
    (you can't move a powered arm by hand).
    """

    def __init__(self, steps):
        self.steps = steps
        self.t0 = None

    def tick(self, now):
        if Arm_Lib.STATE["torque"]:
            return
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        for seconds, pose in self.steps:
            if t < seconds:
                if pose is not None:
                    Arm_Lib.set_pose(pose)
                return
            t -= seconds
        last = [p for _, p in self.steps if p is not None]
        if last:
            Arm_Lib.set_pose(last[-1])


HAND = {"hand": None}


def fake_time():
    return CLOCK["now"]


def fake_sleep(seconds):
    CLOCK["now"] += max(float(seconds), 0.001)
    if CLOCK["limit"] is not None and CLOCK["now"] > CLOCK["limit"]:
        raise AssertionError("stuck: the pretend clock ran past its limit")
    if HAND["hand"] is not None:
        HAND["hand"].tick(CLOCK["now"])


time.time = fake_time
time.sleep = fake_sleep


def start_scenario(hand_steps, limit_s=1500):
    HAND["hand"] = Hand(hand_steps)
    CLOCK["limit"] = CLOCK["now"] + limit_s


ANSWERS = []


def scripted_input(prompt=""):
    for i, (needle, answer) in enumerate(ANSWERS):
        if needle in prompt:
            ANSWERS.pop(i)
            return answer
    raise AssertionError("unexpected question: %r" % prompt)


builtins.input = scripted_input


def quiet(fn, *args, **kwargs):
    """Runs fn with its printing captured; returns (result, text)."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        result = fn(*args, **kwargs)
    return result, buf.getvalue()


# ---------------- the real board (pawn tops) ----------------
SQ_MM = 28.0
PAWN_TOP_Z = 27.0


def true_xyz(sq):
    f, r = "abcdefgh".index(sq[0]), int(sq[1]) - 1
    return (-98.0 + SQ_MM * f, 95.0 + SQ_MM * r, PAWN_TOP_Z)


def hand_pose(sq, tilt_first=20, off_x=0.0):
    """Servo angles with the closed claw tip on top of the pawn on sq, claw tilted."""
    x, y, z = true_xyz(sq)
    x += off_x
    tilts = [tilt_first] + [t for d in range(1, 80) for t in (tilt_first + d, tilt_first - d)]
    for tilt in tilts:
        if tilt < 0:
            continue
        s = kinematics.inverse(x, y, z, approach_deg=180.0 - tilt)
        if s is not None and all(2.0 <= s[i] <= 178.0 for i in (1, 2, 3, 4)):
            return [s[1], s[2], s[3], s[4], 90.0, config.CLAW_MAX]
    raise AssertionError("no hand pose for " + sq)


def silent_wrist(pose):
    """Same place, but the wrist bent far past its scale: it can't report."""
    return pose[:3] + [-30.0] + pose[4:]


HIGH_POSE = [90.0, 90.0, 90.0, 60.0, 90.0, config.CLAW_MAX]    # claw way up high

SQUARES = list(config.TEACH_SQUARES)
COUNTDOWN = [1] * 5 + [6]          # five short beeps, then the long one: power off


def teach_steps(squares, wobble_square=None):
    steps = [(3.0, None)]                       # a pause before you start: not recorded
    for sq in squares:
        if sq == wobble_square:
            steps.append((2.0, silent_wrist(hand_pose(sq, tilt_first=0))))
        steps.append((3.0, hand_pose(sq)))
    return steps


def taught_error(frame, sq):
    got = frame.taught[sq]
    want = true_xyz(sq)
    return max(abs(got[i] - want[i]) for i in range(3))


# =====================================================================
print("--- 1. teach 5 squares hands-free (one with the wrist silent at first) ---")
Arm_Lib.reset()
a = arm_mod.Arm()
a.interactive = False
start_scenario(teach_steps(SQUARES, wobble_square="b6"))
ANSWERS[:] = [("Squares to teach", "")]
frame, out = quiet(teach.run, a, None, None)
check("no other questions (saved by itself)", not ANSWERS and frame is not None)
check("all 5 squares recorded", frame is not None and sorted(frame.taught) == sorted(SQUARES))
if frame is not None:
    worst = max(taught_error(frame, sq) for sq in SQUARES)
    check("each recorded within 4 mm of the real pawn top", worst <= 4.0, "%.1f mm" % worst)
    check("square size found", abs(frame.square_size_mm()[0] - SQ_MM) < 1.0,
          "%.1f mm" % frame.square_size_mm()[0])
    check("saved to the board file", os.path.exists(config.BOARD_FILE))
beeps = Arm_Lib.STATE["beeps"]
check("beeps: countdown, then 1,2,3,4,5 for the squares, then a long one",
      beeps == COUNTDOWN + [1] * 15 + [6], str(beeps))
lights = Arm_Lib.STATE["lights"]
check("red light while the wrist couldn't report", teach.RED in lights)
check("green light when a square was recorded", lights.count(teach.GREEN) == 5)
check("power is back on at the end", Arm_Lib.STATE["torque"] and a.powered)
check("the first pause before moving wasn't recorded as a square",
      "B3 recorded" in out and taught_error(frame, "b3") <= 4.0)
check("never touched the board", Arm_Lib.STATE["contacts"] == 0)
check("tells you what to fix while the wrist is silent", "straighten the wrist" in out)

# =====================================================================
print("\n--- 2. stop early: lift the claw up high and hold still ---")
Arm_Lib.reset()
a = arm_mod.Arm()
a.interactive = False
steps = teach_steps(SQUARES[:2]) + [(10.0, HIGH_POSE)]
start_scenario(steps)
ANSWERS[:] = [("Squares to teach", ""), ("q = quit", "q")]
frame, out = quiet(teach.run, a, None, None)
check("2 squares recorded, then it stopped", "B3 recorded" in out and "G3 recorded" in out
      and "B6 recorded" not in out)
check("asked at the computer what to do (only 2 squares)", not ANSWERS)
check("nothing saved", frame is None)
beeps = Arm_Lib.STATE["beeps"]
check("two beeps = stopped early, power on", beeps[-2:] == [1, 1] and 6 not in beeps[6:],
      str(beeps))
check("power is back on", Arm_Lib.STATE["torque"])

# =====================================================================
print("\n--- 3. you walk away: nothing moves for a while ---")
Arm_Lib.reset()
a = arm_mod.Arm()
a.interactive = False
start_scenario([(1.0, None)])
ANSWERS[:] = [("Squares to teach", ""), ("q = quit", "q")]
frame, out = quiet(teach.run, a, None, None)
check("stops by itself after the idle time", "Nothing moved" in out)
check("power back on, long beep", Arm_Lib.STATE["torque"] and Arm_Lib.STATE["beeps"][-1] == 6)

# =====================================================================
print("\n--- 4. one bad touch is flagged; redo just that square ---")
Arm_Lib.reset()
a = arm_mod.Arm()
a.interactive = False
bad = hand_pose("g6", off_x=12.0)          # 12 mm to the side of the pawn
steps = [(3.0, None)] + [(3.0, hand_pose(sq) if sq != "g6" else bad) for sq in SQUARES]
start_scenario(steps)
ANSWERS[:] = [("Squares to teach", ""), ("a square = teach/redo", "g6")]
real_teach_at_arm = teach._teach_at_arm


def redo_with_good_hand(arm, squares, camera, photo_dir, frame):
    if squares == ["g6"]:
        start_scenario([(3.0, None), (3.0, hand_pose("g6"))])
    return real_teach_at_arm(arm, squares, camera, photo_dir, frame)


teach._teach_at_arm = redo_with_good_hand
frame, out = quiet(teach.run, a, None, None)
teach._teach_at_arm = real_teach_at_arm
check("the bad touch was flagged", "G6" in out and "probably off" in out)
check("after redoing it, saved without more questions", frame is not None and not ANSWERS)
if frame is not None:
    check("g6 is right now", taught_error(frame, "g6") <= 4.0, "%.1f mm" % taught_error(frame, "g6"))

# =====================================================================
print("\n--- 5. teach the look-at-board view hands-free ---")
Arm_Lib.reset()
a = arm_mod.Arm()
a.interactive = False
a.set_board(BoardFrame.load())
view = [90.0, 100.0, 40.0, 30.0, 90.0, 150.0]
start_scenario([(2.0, None), (1.0, [90.0, 95.0, 70.0, 50.0, 90.0, 150.0]), (6.0, view)])
pose, out = quiet(look_pose.run, a, None)
check("saved the view", pose is not None and os.path.exists(config.LOOK_POSE_FILE))
if pose is not None:
    check("it's the pose you held", all(abs(pose[i] - view[i]) <= 2 for i in range(4)), str(pose))
    check("becomes home", a.home_pose == pose)
check("long beep = saved", Arm_Lib.STATE["beeps"][-1] == 6)
check("power back on", Arm_Lib.STATE["torque"])

# =====================================================================
print("\n--- 6. run.py: start-up asks nothing, goes home, menu works ---")
run_mod.start_camera = lambda state, retry=False: None
run_mod.other_arm_programs = lambda: []        # never touch real processes from a test
Arm_Lib.reset(hand_pose("e4"))           # left on top of a piece, power on
start_scenario([], limit_s=3000)
ANSWERS[:] = [("Choose", "3"), ("A square = hover", "e5"), ("A square = hover", "e3 e4"),
              ("A square = hover", ""), ("Choose", "4"), ("Choose", "2"), ("Choose", "0")]
_, out = quiet(run_mod.main)
home = look_pose.load()
check("banner shows the version and the last fix",
      "v%s" % config.VERSION in out and "Last fix:" in out)
check("start-up asked nothing; menu answers all used", not ANSWERS)
check("went home by itself at start-up", "Going home in 3 s" in out and "Home." in out)
check("map printed with the legend", "not tested yet" in out and "* = you taught it" in out)
check("hovered and measured e5", "Hovering over e5: off by" in out)
check("moved a piece", "Moved e3 -> e4" in out)
check("self-test measured squares", "Self-test: hovering" in out and "[1/" in out)
check("0 = rest and quit: ends folded back at rest, power on", Arm_Lib.STATE["torque"] and
      all(abs(Arm_Lib.STATE["pos"][i] - config.POSE_REST[i - 1]) <= 4 for i in range(1, 5)))
check("never touched the board", Arm_Lib.STATE["contacts"] == 0,
      "%d contacts" % Arm_Lib.STATE["contacts"])
results = self_test.latest_results(BoardFrame.load())
check("self-test results kept for the map", len(results) > 20, "%d squares" % len(results))

# =====================================================================
print("\n--- 7. run.py: a joint can't report at start-up -> waits, powers on, doesn't move ---")
pose = hand_pose("e4")
Arm_Lib.reset(silent_wrist(pose))
Arm_Lib.STATE["torque"] = False
start_scenario([(5.0, None), (2.0, pose)])
ANSWERS[:] = [("Choose", "q")]
before = dict(Arm_Lib.STATE["pos"])
_, out = quiet(run_mod.main)
check("said which joint and what to do", "wrist" in out and "straighten the wrist" in out)
check("powered on by itself once it could", "Power is on" in out and Arm_Lib.STATE["torque"])
check("didn't move afterwards (you may be holding it)",
      all(abs(Arm_Lib.STATE["pos"][i] - pose[i - 1]) <= 1 for i in range(1, 5)))
check("no questions besides the menu", not ANSWERS)

# =====================================================================
print("\n--- 7c. run.py: stiff arm (power on) with a joint that can't report ---")
pose = hand_pose("e4")
Arm_Lib.reset(pose[:2] + [-30.0] + pose[3:])       # elbow far past its scale, power ON
start_scenario([(3.0, None), (2.0, pose)])
ANSWERS[:] = [("Choose", "q")]
Arm_Lib.STATE["beeps"] = []
_, out = quiet(run_mod.main)
check("countdown, then power off so you can move it", "Power goes OFF in" in out
      and "Power OFF" in out)
check("powers back on by itself once it can (long beep)", Arm_Lib.STATE["torque"]
      and "Power is on" in out and Arm_Lib.STATE["beeps"][-1] == 6, str(Arm_Lib.STATE["beeps"]))
check("red light while the joint couldn't report", (90, 0, 0) in Arm_Lib.STATE["lights"])
check("says it once, not over and over", out.count("waiting: elbow") == 1,
      "%d times" % out.count("waiting: elbow"))
check("doesn't move afterwards", all(abs(Arm_Lib.STATE["pos"][i] - pose[i - 1]) <= 1
                                     for i in range(1, 5)))

print("\n--- 7d. run.py stops Yahboom's driver by itself ---")
killed = []
running = [[(4242, "Yahboom's ROS arm driver (roslaunch /tmp/arm.launch)", False)]]
real_others, real_kill = run_mod.other_arm_programs, run_mod._sudo_kill
assert real_others() == [], "test isolation"
run_mod.other_arm_programs = lambda: running[0]


def fake_kill(pids):
    killed.extend(pids)
    running[0] = []
    return True


run_mod._sudo_kill = fake_kill
_, out = quiet(run_mod.stop_other_programs)
run_mod.other_arm_programs, run_mod._sudo_kill = real_others, real_kill
check("stops it with sudo, no question", killed == [4242] and "Stopped it" in out)

# =====================================================================
print("\n--- 7a. run.py from Yahboom's folded pose (shoulder ~182, wrist ~-2) ---")
FOLD = [86.0, 182.0, 18.0, -2.0, 91.0, 166.0]
Arm_Lib.reset(FOLD)
start_scenario([], limit_s=3000)
ANSWERS[:] = [("Choose", "q")]
_, out = quiet(run_mod.main)
check("reads the joints past the end of their scale", "can't report" not in out
      and "Waiting for every joint" not in out)
check("powers on in place and goes home by itself", "Going home in 3 s" in out and "Home." in out)
check("ends home", all(abs(Arm_Lib.STATE["pos"][i] - home[i - 1]) <= 4 for i in range(1, 5)))
check("never touched the board", Arm_Lib.STATE["contacts"] == 0)

print("\n--- 7a2. a wrist just past 0 (claw vertical on a piece) still reports ---")
Arm_Lib.reset()
a = arm_mod.Arm()
Arm_Lib.set_pose([90.0, 60.0, 30.0, -5.0, 90.0, 150.0])
got = a.read_angles(samples=1)
check("wrist reads about -5 (not nothing, not 185)", got[3] is not None and abs(got[3] + 5) <= 2,
      str(got[3]))
Arm_Lib.set_pose([90.0, 60.0, 30.0, -30.0, 90.0, 150.0])
check("far past the scale = no answer", a.read_angles(samples=1)[3] is None)

# =====================================================================
print("\n--- 7b. every command in every motion log was a small step ---")
import json                                                   # noqa: E402
biggest, where, lag = 0.0, "", 0.0
folder = os.path.join(config.LOGS_DIR, "motion")
for name in sorted(os.listdir(folder)):
    last_sent, last_got = {}, None
    with open(os.path.join(folder, name)) as fh:
        for line in fh:
            event = json.loads(line)
            if event.get("event") in ("limp", "stiffen", "blocked", "unblocked"):
                last_sent, last_got = {}, None
                continue
            if event.get("event") != "move" or "got" not in event:
                continue
            for sid, angle in event["sent"].items():
                sid = int(sid)
                if sid > 4:
                    continue
                if sid in last_sent and abs(angle - last_sent[sid]) > biggest:
                    biggest, where = abs(angle - last_sent[sid]), "%s servo %d" % (name, sid)
                if last_got is not None and last_got[sid - 1] is not None:
                    lag = max(lag, abs(angle - last_got[sid - 1]))
                last_sent[sid] = angle
            last_got = event["got"]
check("no command moved a joint more than %.0f deg from the one before"
      % config.CONTINUOUS_DEG, biggest <= config.CONTINUOUS_DEG + 0.01,
      "biggest %.1f deg (%s)" % (biggest, where))
check("and none was further than the gate allows from where the joint really was",
      lag <= config.GATE_STEP_DEG, "worst %.1f deg" % lag)

# =====================================================================
print("\n--- 8. coloured map: plain text when colours are off ---")
frame = BoardFrame.load()
text = run_mod.board_map(frame, self_test.latest_results(frame), colour=False)
check("8 ranks + files row", text.count("\n") >= 9 and " a " in text)
check("no colour codes", "\033[" not in text)
coloured = run_mod.board_map(frame, self_test.latest_results(frame), colour=True)
check("colour codes when on", "\033[30;42m" in coloured)

print("\n%d failure(s)" % len(failures))
if failures:
    for f in failures:
        print("  - " + f)
    sys.exit(1)
