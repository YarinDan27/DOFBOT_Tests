#!/usr/bin/env python3
"""
v2.5: the arm's shape against what's around it, the planner, smooth motion
and keyboard control -- on the simulated arm (tests/fakes/Arm_Lib.py), where
the electronics box and turning base are solid. Nothing real moves:

    python3 tests/test_world.py
"""
import builtins
import io
import json
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))
sys.path.insert(1, os.path.dirname(HERE))

import Arm_Lib                                                # noqa: E402 (the fake)
import config                                                 # noqa: E402

assert "fakes" in Arm_Lib.__file__, "refusing to run against the real arm"
TMP = tempfile.mkdtemp()
config.LOGS_DIR = os.path.join(TMP, "logs")
config.LOCK_FILE = os.path.join(TMP, "arm.lock")
config.DATA_DIR = os.path.join(TMP, "data")
config.CALIBRATION_DIR = os.path.join(TMP, "calibration")
config.BOARD_FILE = os.path.join(config.CALIBRATION_DIR, "board_arm.json")
config.LOOK_POSE_FILE = os.path.join(config.CALIBRATION_DIR, "look_pose.json")
config.ELECTRONICS_FILE = os.path.join(config.CALIBRATION_DIR, "electronics.json")

from robot import arm as arm_mod                              # noqa: E402
from robot import body, kinematics, motion, safety, world    # noqa: E402
from robot.board_frame import BoardFrame                      # noqa: E402
from sessions import electronics, jog                         # noqa: E402
from viz import live, scene                                   # noqa: E402

# a pretend clock: sleeping moves it on instantly, so moves take no real time
CLOCK = [1000.0]
REAL_TIME = time.time
time.time = lambda: CLOCK[0]
time.sleep = lambda s: CLOCK.__setitem__(0, CLOCK[0] + max(0.0, float(s)))
random.seed(3)
failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


def quiet(fn, *args, **kwargs):
    buf = io.StringIO()
    with redirect_stdout(buf):
        result = fn(*args, **kwargs)
    return result, buf.getvalue()


W = world.World.default()
REST = list(config.POSE_REST)
FOLD = [86.0, 182.0, 18.0, -2.0, 91.0, 166.0]        # your arm, read 2026-10-25
IDEAL = [90.0, 190.0, 0.0, 0.0, 90.0, 150.0]         # "shoulder ~190, elbow 0, wrist 0"
CRASH = [90.0, 190.0, 180.0, 180.0, 90.0, 150.0]     # "elbow and wrist at 180: damage"

# =====================================================================
print("--- 1. the arm's shape against the electronics, base and table ---")
check("resting pose is allowed", W.margin(REST)[0] >= 0, "%.1f mm spare" % W.margin(REST)[0])
check("your folded pose is allowed", W.margin(FOLD)[0] >= 0)
check("your ideal rest (shoulder 190, elbow 0, wrist 0) is allowed", W.margin(IDEAL)[0] >= 0)
m, link, ob, _ = W.margin(CRASH)
check("shoulder 190 + elbow 180 + wrist 180 is refused", m < 0, "%s into the %s" % (link, ob))
problem = W.body_problem([90, 180, 120, 90])
check("folding the forearm down over the electronics is refused, saying what hits",
      problem is not None and "electronics" in problem, problem or "")
check("the old per-servo limits are gone: shoulder 185 is inside its range",
      kinematics.within_limits({2: 185.0}) and not kinematics.within_limits({2: 195.0}))
check("upright and ready are clear",
      W.margin([90, 90, 90, 90])[0] > 20 and W.margin(config.POSE_READY)[0] > 20)
check("the shape has five parts incl. the camera bar",
      [c[0] for c in body.capsules(REST)] == ["upper arm", "forearm", "wrist", "gripper",
                                              "camera"])

# =====================================================================
print("\n--- 2. planned moves between the poses that matter ---")


def dense_ok(path, zone, step=0.5):
    """Re-checks a planned path independently, every `step` degrees."""
    for a, b in zip(path, path[1:]):
        n = max(1, int(max(abs(b[i] - a[i]) for i in range(4)) / step))
        m0 = zone.margin(a)[0]
        for k in range(1, n + 1):
            q = [a[i] + (b[i] - a[i]) * k / float(n) for i in range(4)]
            if zone.margin(q)[0] < min(m0, 0.0) - 1.0:
                return False, q
    return True, None


def stage_path(start, stages):
    return arm_mod.Arm._stage_path(start, stages)


worst_time = 0.0
for name, a, b in (("fold -> ready", FOLD, config.POSE_READY),
                   ("fold -> rest", FOLD, REST),
                   ("rest -> ready", REST, config.POSE_READY),
                   ("ready -> rest", config.POSE_READY, REST),
                   ("upright -> ideal rest", [90, 90, 90, 90, 90, 150], IDEAL),
                   ("a2 reading -> rest", [118, 79, 4, 26, 89, 166], REST)):
    t0 = REAL_TIME()
    stages = safety.plan_pose_move(a, b, W)
    worst_time = max(worst_time, REAL_TIME() - t0)
    ok = stages is not None
    detail = ""
    if ok:
        ok, bad = dense_ok(stage_path(a, stages), W)
        detail = "" if ok else "too close at %s" % [round(v) for v in bad]
    check(name + ": safe plan, re-checked every 0.5 deg", ok, detail)
check("planning is quick", worst_time < 2.0, "%.2f s worst" % worst_time)

# a move that needs the search: a post in the way of turning the base
post = world.Box.between("post", (-35, 35), (90, 150), (-15, 250), 5.0)
walled = world.World(W.polygon, W.board_z, W.obstacles + [post])


def pose_at(x, y, z):
    sol, _ = kinematics.solve_reachable(x, y, z)
    return [sol[1], sol[2], sol[3], sol[4]]


start, target = pose_at(-170, 140, 100), pose_at(170, 140, 100)
direct = safety._find_stages(start, target, walled)
path = safety.search_path(start, target, walled, seed=1, time_limit=20)
ok = path is not None and dense_ok(path, walled)[0]
check("with a post in the way, the search finds a way round", direct is None and ok,
      "%d hops" % (len(path) - 1) if path else "none")
check("the search gives the same answer every time",
      path == safety.search_path(start, target, walled, seed=1, time_limit=20))

# =====================================================================
print("\n--- 3. smooth motion ---")
fr = motion.fractions(90.0)
steps = [b - a for a, b in zip([0.0] + fr, fr)]
check("a 90 degree move ends exactly there", abs(fr[-1] - 1.0) < 1e-9)
check("it only ever moves forward", all(s >= -1e-12 for s in steps))
peak = max(steps) * 90.0
check("no tick moves faster than the top speed",
      peak <= config.JOINT_SPEED_DEG_S * config.CONTROL_DT + 1e-6, "%.1f deg/tick" % peak)
check("it starts and ends gently", steps[0] < max(steps) and steps[-1] < max(steps))
check("90 degrees takes about 2.8 s", abs(motion.duration(90.0) - 2.75) < 0.01,
      "%.2f s" % motion.duration(90.0))
check("a tiny move is one tick", motion.fractions(0.1) == [1.0])

# real moves on the simulated arm
Arm_Lib.reset(FOLD)
a = arm_mod.Arm()
sent = []
real_move = a.move_servos


def recording(angles, duration_ms=400, wait=True, check=True):
    sent.append(dict(angles))
    return real_move(angles, duration_ms=duration_ms, wait=wait, check=check)


a.move_servos = recording
quiet(a.move_to_pose, REST)
at_rest = all(abs(Arm_Lib.STATE["pos"][i] - REST[i - 1]) <= 4 for i in range(1, 5))
check("from your fold it gets to rest", at_rest)
moves4 = [s for s in sent if any(k <= 4 for k in s)]
jumps = []
prev = FOLD[:4]
for s in moves4:
    cur = [s.get(k, prev[k - 1]) for k in (1, 2, 3, 4)]
    jumps.append(max(abs(cur[i] - prev[i]) for i in range(4)))
    prev = cur
check("joints move together in small steps", max(jumps) <= config.JOINT_SPEED_DEG_S *
      config.CONTROL_DT + 0.5, "biggest %.1f deg" % max(jumps))
check("never touched anything", Arm_Lib.STATE["contacts"] == 0)

# an obstacle the model doesn't know: the real box is taller than modelled
Arm_Lib.reset(config.POSE_READY)
Arm_Lib.ELECTRONICS["z"] = (0.0, 130.0)
a = arm_mod.Arm()
outcome = "arrived"
try:
    quiet(a.move_to_pose, REST)
except arm_mod.ArmBlocked as e:
    outcome = "blocked: %s" % e
Arm_Lib.ELECTRONICS["z"] = tuple(config.ELECTRONICS["z"])
check("an unknown obstacle stops a smooth move", outcome.startswith("blocked"), outcome[:70])
check("and it stopped within a step or two", Arm_Lib.STATE["contacts"] <= 3,
      "%d contacts" % Arm_Lib.STATE["contacts"])
check("then it holds still and refuses to move", a.blocked)

# =====================================================================
print("\n--- 4. your board, from the squares you taught on 2026-10-23/24 ---")
frame = BoardFrame.fit({"a8": (-62.0, 248.0, 25.0), "h1": (73.0, 108.0, 26.0),
                        "a1": (-67.0, 116.0, 26.0)})
Arm_Lib.reset(REST)
a = arm_mod.Arm()
a.set_board(frame)
reach = frame.reach_map()
ok_squares = [sq for sq, v in reach.items() if v == "ok"]
print("    squares the arm can hover over and grab: %d of 64" % len(ok_squares))
check("most of the board is reachable", len(ok_squares) >= 40, "%d" % len(ok_squares))
visited, refused = 0, []
for sq in ("e4", "a1", "h1", "d8", "h8", "e2"):
    if reach[sq] != "ok":
        continue
    x, y, z = frame.square_xyz(sq)
    try:
        quiet(a.goto, x, y, z + config.HOVER_MM)
        visited += 1
    except (arm_mod.OutOfReach, arm_mod.UnsafeMove):
        refused.append(sq)
quiet(a.go_home)
check("hovers over squares and comes back to rest", visited >= 4 and not a.blocked,
      "%d visited, refused %s" % (visited, refused))
check("without touching anything", Arm_Lib.STATE["contacts"] == 0,
      "%d contacts" % Arm_Lib.STATE["contacts"])

# =====================================================================
print("\n--- 5. the electronics model learns from you ---")
tall = world.World(W.polygon, W.board_z, [o for o in W.obstacles if o.name != "electronics"]
                   + [world.Box.between("electronics", (-55, 55), (-220, -60), (0, 100), 6.0)])
check("with a too-tall guess, your rest pose looks blocked", tall.margin(REST)[0] < 0)
top = world.make_room_for(tall, REST)
check("holding the arm there lowers the box until it fits", top is not None and
      tall.margin(REST)[0] >= 0, "top now %s mm" % top)
check("and saves it", os.path.exists(config.ELECTRONICS_FILE) and
      abs(world.electronics_box().half[2] * 2 - top) < 0.6)
os.remove(config.ELECTRONICS_FILE)
box = electronics.box_from([(-50.0, -62.0, 61.0), (52.0, -60.0, 63.0), (0.0, -215.0, 60.0)])
check("three touched corners make a box", box["x"] == (-50.0, 52.0) and
      box["y"] == (-215.0, -60.0) and box["z"] == (0.0, 63.0))

# =====================================================================
print("\n--- 6. keyboard control ---")
Arm_Lib.reset(config.POSE_READY)
a = arm_mod.Arm()
keys = ["2wwwwd", "1ii", "33oooooooo", "x"]


def typed(prompt=""):
    return keys.pop(0)


builtins.input = typed
_, out = quiet(jog.run, a)
check("moves joints and the claw from keys", "shoulder" in out and out.count("| claw") >= 6)
check("refuses a step that would hit something, saying why",
      "not moving:" in out, " ".join([l.strip() for l in out.splitlines()
                                      if "not moving" in l][:1]))
check("never touched anything", Arm_Lib.STATE["contacts"] == 0)
Arm_Lib.reset(config.POSE_READY)
a = arm_mod.Arm()
keys = ["m", "e4", "x"]
taught = {}
quiet(jog.run, a, None, taught)
check("m marks the claw tip as a square", "e4" in taught)

# =====================================================================
print("\n--- 7. the 3D view ---")
data = scene.build(world.World.for_board(frame), frame, a)
text = json.dumps(data)
check("the scene is plain JSON", len(text) > 1000)
html = live.page(data)
check("the page embeds it", "/*SCENE_DATA*/" not in html and '"obstacles"' in html)
node = shutil.which("node")
if node:
    src = open(os.path.join(os.path.dirname(HERE), "viz", "viewer.html")).read()
    start_i = src.index("function jointPoints")
    end_i = src.index("const LINKS")
    js = src[start_i:end_i] + (
        "const P = %s; const out = [];"
        "for (const a of %s) out.push(jointPoints(P, a));"
        "console.log(JSON.stringify(out));" % (json.dumps(scene.params()),
                                               json.dumps([REST[:4], FOLD[:4], [120, 60, 30, 40]])))
    res = json.loads(subprocess.check_output([node, "-e", js]).decode())
    worst = 0.0
    for got, angles in zip(res, [REST, FOLD, [120, 60, 30, 40]]):
        want = body.joint_points(angles)
        for k in want:
            worst = max(worst, max(abs(got[k][i] - want[k][i]) for i in range(3)))
    check("the page draws the arm exactly where Python puts it", worst < 1e-6,
          "worst %.1e mm" % worst)
else:
    print("  (skipped the page-vs-Python check: node isn't installed)")

shutil.rmtree(TMP, ignore_errors=True)
print("\n%d failure(s)" % len(failures))
if failures:
    for f in failures:
        print("  - " + f)
    sys.exit(1)
