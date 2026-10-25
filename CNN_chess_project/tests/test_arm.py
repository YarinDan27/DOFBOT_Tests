#!/usr/bin/env python3
"""
Runs the real arm code against a simulated arm (tests/fakes/Arm_Lib.py) --
nothing moves, safe to run on the laptop or the Jetson:

    python3 tests/test_arm.py

The simulated arm stalls when it touches the board, and can misreport a
servo -- so this checks that the arm STOPS instead of pushing into the board,
even when the program is wrong about where the arm is.
"""
import builtins
import os
import random
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))     # the simulated Arm_Lib, first
sys.path.insert(1, os.path.dirname(HERE))

import Arm_Lib                                                # noqa: E402 (the fake)
import config                                                 # noqa: E402
from robot import arm as arm_mod                              # noqa: E402
from robot import kinematics                                  # noqa: E402

assert "fakes" in Arm_Lib.__file__, "refusing to run against the real arm"
time.sleep = lambda s: None
random.seed(11)
TMP = tempfile.mkdtemp()
config.LOGS_DIR = TMP

failures = []
ANSWERS = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


def scripted_input(prompt=""):
    for i, (needle, answer, action) in enumerate(ANSWERS):
        if needle in prompt:
            ANSWERS.pop(i)
            if action:
                action()
            return answer
    raise AssertionError("unexpected question: %r" % prompt)


builtins.input = scripted_input


def at(pose, tol=4.0):
    return all(abs(Arm_Lib.STATE["pos"][i] - pose[i - 1]) <= tol for i in range(1, 5))


def low_pose():
    s, _ = kinematics.solve_reachable(0, 180, -8)
    return [s[1], s[2], s[3], s[4], 90, 150]


print("--- 1. normal start from standing up ---")
Arm_Lib.reset()
ANSWERS[:] = [("Enter = go home slowly", "", None)]
a = arm_mod.Arm()
a.start_up()
check("goes to the ready pose", at(config.POSE_READY))
check("never touched the board", Arm_Lib.STATE["contacts"] == 0)
check("not blocked", not a.blocked)

print("\n--- 2. start with the claw down on the board (readings correct) ---")
Arm_Lib.reset(low_pose())
ANSWERS[:] = [("Enter = go home slowly", "", None)]
a = arm_mod.Arm()
a.start_up()
check("rises and goes home", at(config.POSE_READY))
check("never pushed into the board", Arm_Lib.STATE["contacts"] == 0)

print("\n--- 3. a servo misreports (like today): the arm must stop, not push ---")
worst = 0.0
for sid, bias in ((2, 25.0), (2, -25.0), (3, 30.0), (4, 40.0), (4, -40.0)):
    Arm_Lib.reset(low_pose())
    Arm_Lib.READ_BIAS[sid] = bias
    ANSWERS[:] = [("Enter = go home slowly", "", None),
                  ("HOLD THE ARM and press Enter", "", None),
                  ("Move it well above the board", "", None),
                  ("Press Enter when you're holding it", "", None),
                  ("Enter = try again", "q", None)] * 2
    a = arm_mod.Arm()
    outcome = "went home"
    try:
        a.start_up()
    except arm_mod.ArmBlocked:
        outcome = "blocked"
    except (arm_mod.StillLimp, RuntimeError, arm_mod.UnsafeMove) as e:
        outcome = "refused (%s)" % type(e).__name__
    pushed = Arm_Lib.STATE["contacts"]
    # how hard is it still pushing? (sent target vs real position, after the stop)
    print("    servo %d reads %+.0f deg off -> %s, %d contact(s)" % (sid, bias, outcome, pushed))
    if outcome == "blocked":
        check("   holds still after stopping", a.blocked)
        try:
            a.go_home()
            check("   refuses to move again until you say it's clear", False)
        except arm_mod.ArmBlocked:
            check("   refuses to move again until you say it's clear", True)
    check("   never kept pushing: at most one stalled sub-step per servo", pushed <= 2,
          "%d contacts" % pushed)

print("\n--- 3b. something on the board the program doesn't know about ---")
from robot.board_frame import BoardFrame                      # noqa: E402
frame = BoardFrame.fit({"a1": (-77.0, 126.0, -13.0), "h1": (77.0, 126.0, -13.0),
                        "a6": (-77.0, 236.0, -13.0)})
Arm_Lib.reset()
ANSWERS[:] = [("Enter = go home slowly", "", None)]
a = arm_mod.Arm()
a.set_board(frame)
a.start_up()
Arm_Lib.BOARD["z"] = 60.0          # e.g. a box sitting on the board, 73 mm tall
x, y, z = frame.square_xyz("e4")
outcome = "reached e4"
try:
    a.goto(x, y, z + config.HOVER_MM)
except arm_mod.ArmBlocked as e:
    outcome = "blocked: %s" % e
Arm_Lib.BOARD["z"] = -13.0
print("    " + outcome)
check("stops on contact instead of pushing on", outcome.startswith("blocked"))
check("only one or two sub-steps pushed before it stopped",
      Arm_Lib.STATE["contacts"] <= 4, "%d contacts" % Arm_Lib.STATE["contacts"])

print("\n--- 4. a servo doesn't answer at start: you move the arm, then it starts ---")
Arm_Lib.reset(low_pose())
Arm_Lib.SILENT.add(4)


def lift_and_fix():
    Arm_Lib.set_pose([90, 90, 90, 90, 90, 150])
    Arm_Lib.SILENT.clear()


ANSWERS[:] = [("HOLD THE ARM and press Enter", "", None),
              ("Move it well above the board", "", lift_and_fix),
              ("Press Enter when you're holding it", "", None),
              ("Enter = go home slowly", "", None)]
a = arm_mod.Arm()
check("power stayed off-limits at first (no blind switch-on)", not a.powered)
a.start_up()
check("after you moved it, it starts and goes home", at(config.POSE_READY))

print("\n--- 5. a joint past the move limits: power on in place, come back in checked steps ---")
Arm_Lib.reset([90, 90, 90, 178, 90, 150])     # wrist past its 170 limit
before = dict(Arm_Lib.STATE["pos"])
a = arm_mod.Arm()
check("switches on without moving (no jump)", a.powered and all(
    abs(Arm_Lib.STATE["pos"][i] - before[i]) < 1.5 for i in range(1, 5)))
ANSWERS[:] = [("Enter = go home slowly", "", None)]
a.start_up()
check("then goes home inside the limits", at(config.POSE_READY))
check("without touching anything", Arm_Lib.STATE["contacts"] == 0)

print("\n--- 5a. tonight's pose: left limp at a2, elbow at 4 ---")
Arm_Lib.reset([118, 79, 4, 26, 89, 166])
Arm_Lib.STATE["torque"] = False
ANSWERS[:] = [("Enter = go home slowly", "", None),
              ("HOLD THE ARM and press Enter", "", None),
              ("Move it well above the board", "",
               lambda: Arm_Lib.set_pose([118, 90, 90, 90, 89, 166])),
              ("Press Enter when you're holding it", "", None)]
a = arm_mod.Arm()
check("switches on where it lies (no endless 'try again')", a.powered)
a.start_up()
check("gets home", at(config.POSE_READY))
check("without pushing into the board", Arm_Lib.STATE["contacts"] == 0)

print("\n--- 5b. teaching: an elbow past the robot's own limit is still recorded ---")
from sessions import teach                                     # noqa: E402
Arm_Lib.reset()
a = arm_mod.Arm()
a.limp()
Arm_Lib.set_pose([90, 30, 4, 60, 90, 150])     # like a2 tonight: elbow at 4 (limit 10)
point = teach._record(a, None, "a2", TMP)
check("records the square instead of asking again", point is not None)
before = dict(Arm_Lib.STATE["pos"])
a.stiffen_here()
check("power comes back on right there, without moving", a.powered and all(
    abs(Arm_Lib.STATE["pos"][i] - before[i]) < 1.5 for i in range(1, 5)))
check("a look pose there would still be refused, with which way to move",
      "tilt it back up" in (arm_mod.pose_problem(a.read_angles(3)) or ""))

print("\n--- 6. Ctrl-C at the start question: nothing moves ---")
Arm_Lib.reset(low_pose())
before = dict(Arm_Lib.STATE["pos"])


def ctrl_c(prompt=""):
    raise KeyboardInterrupt


builtins.input = ctrl_c
try:
    with arm_mod.Arm() as a:
        a.start_up()
except KeyboardInterrupt:
    pass
builtins.input = scripted_input
check("arm didn't move", all(abs(Arm_Lib.STATE["pos"][i] - before[i]) < 3 for i in range(1, 5)))

print("\n--- 6b. finds other programs talking to the arm ---")
import subprocess                                              # noqa: E402
ros = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)",
                        "/tmp/arm.launch"])
copy = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)", "run.py"],
                        cwd=config.ROOT)
time.sleep = __import__("time").sleep
__import__("time").sleep(0.5)
found = arm_mod.other_arm_programs()
ros.kill()
copy.kill()
check("spots Yahboom's ROS arm driver", any(p == ros.pid and not ours for p, _, ours in found))
check("spots another copy of this program", any(p == copy.pid and ours for p, _, ours in found))

print("\n--- 7. every move is logged ---")
logs = os.listdir(os.path.join(TMP, "motion"))
check("motion logs written", len(logs) > 0, "%d files" % len(logs))

shutil.rmtree(TMP, ignore_errors=True)
print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("All checks passed.")
