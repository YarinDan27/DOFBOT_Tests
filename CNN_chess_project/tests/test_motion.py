#!/usr/bin/env python3
"""
v2.6: motion against servos that behave like real ones -- the lessons from
Microduck (Pollen Robotics), checked on the simulated arm:

  * a servo model (delay, speed, gear play, sag, noise, missed reads) and a
    range of random servos to run every move against ("domain randomisation")
  * the safety gate every command goes through
  * the fixed-rate control loop: keeps its schedule, survives a missed read
  * one program on the arm at a time (the bus lock)

Nothing real moves:   python3 tests/test_motion.py
"""
import os
import random
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))
sys.path.insert(1, os.path.dirname(HERE))

import Arm_Lib                                                # noqa: E402 (the fake)
import config                                                 # noqa: E402
import servo_model                                            # noqa: E402

assert "fakes" in Arm_Lib.__file__, "refusing to run against the real arm"
TMP = tempfile.mkdtemp()
config.LOGS_DIR = os.path.join(TMP, "logs")
config.LOCK_FILE = os.path.join(TMP, "arm.lock")
config.DATA_DIR = os.path.join(TMP, "data")
config.CALIBRATION_DIR = os.path.join(TMP, "calibration")
config.ELECTRONICS_FILE = os.path.join(config.CALIBRATION_DIR, "electronics.json")

from robot import arm as arm_mod                              # noqa: E402
from robot import motion, safety, world                       # noqa: E402
from robot.board_frame import BoardFrame                      # noqa: E402

CLOCK = [1000.0]
EVENTS = []                  # (clock time, function) to run when the clock passes it


def fake_time():
    return CLOCK[0]


def fake_sleep(s):
    CLOCK[0] += max(0.0, float(s))
    while EVENTS and EVENTS[0][0] <= CLOCK[0]:
        EVENTS.pop(0)[1]()


time.time = fake_time
time.sleep = fake_sleep
failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


REST = list(config.POSE_REST)
FOLD = [86.0, 182.0, 18.0, -2.0, 91.0, 166.0]
FRAME = BoardFrame.fit({"a8": (-62.0, 248.0, 25.0), "h1": (73.0, 108.0, 26.0),
                        "a1": (-67.0, 116.0, 26.0)})


def new_arm(pose, params=None, seed=0):
    Arm_Lib.use_servo_model(params, seed)
    Arm_Lib.reset(pose)
    a = arm_mod.Arm()
    a.set_board(FRAME)
    return a


def at(pose, tol=5.0):
    return all(abs(Arm_Lib.STATE["pos"][i] - pose[i - 1]) <= tol for i in range(1, 5))


# =====================================================================
print("--- 1. the servo model behaves like a servo ---")
m = servo_model.ServoModel({"delay_s": 0.02, "max_speed": 200.0, "noise": 0.0, "sag": 0.0,
                            "backlash": 0.0})
m.command(2, 90.0, 120.0, 1000, now=0.0)
check("nothing happens during the command delay", m.motor(2, 90.0, 0.01) == 90.0)
check("halfway through the time it was given, it's halfway",
      abs(m.motor(2, 90.0, 0.52) - 105.0) < 0.01)
m.command(2, 90.0, 150.0, 50, now=0.0)
check("it never goes faster than its top speed (60 deg at 200 deg/s takes 0.3 s)",
      abs(m.motor(2, 90.0, 0.17) - 120.0) < 0.5 and abs(m.motor(2, 90.0, 0.32) - 150.0) < 0.5)
sag = servo_model.ServoModel({"sag": 3.0, "backlash": 1.0}).droop([90, 45, 90, 90])
check("an arm leaning toward the board droops further toward it",
      sag[1] < -1.0, "shoulder %+.1f deg" % sag[1])

# =====================================================================
print("\n--- 2. every move, against 12 random servo sets + the worst one (domain randomisation) ---")
rng = random.Random(7)
stops, worst_lag, slowest = [], 0.0, 0.0
WORST = {"max_speed": 90.0, "sag": 4.0, "backlash": 2.0, "delay_s": 0.05, "noise": 0.6,
         "miss_rate": 0.03}                      # every range at its bad end
for k in range(13):
    params = servo_model.randomised(rng) if k < 12 else WORST
    a = new_arm(FOLD, params, seed=k)
    t0 = CLOCK[0]
    try:
        a.move_to_pose(REST)
        for sq in ("e4", "a1", "h8"):
            x, y, z = FRAME.square_xyz(sq)
            a.goto(x, y, z + config.HOVER_MM)
        a.go_home()
    except (arm_mod.ArmBlocked, arm_mod.UnsafeMove, arm_mod.OutOfReach) as e:
        stops.append("set %d: %s (%s)" % (k, e, ", ".join("%s %.2g" % kv
                                                           for kv in sorted(params.items()))))
        Arm_Lib.ELECTRONICS["z"] = tuple(config.ELECTRONICS["z"])
    slowest = max(slowest, CLOCK[0] - t0)
    if Arm_Lib.STATE["contacts"]:
        stops.append("set %d touched something" % k)
check("no false stops and no contacts with any of them", not stops, "; ".join(stops)[:300])
check("and every tour ended back home", at(REST) or bool(stops))
print("    slowest tour: %.0f s" % slowest)

# =====================================================================
print("\n--- 3. something the model doesn't know about: it stops within a few ticks ---")
for k, params in enumerate((servo_model.DEFAULTS, {"max_speed": 90.0, "sag": 4.0,
                                                   "backlash": 2.0, "delay_s": 0.05})):
    a = new_arm(config.POSE_READY, params, seed=k)
    Arm_Lib.ELECTRONICS["z"] = (0.0, 130.0)           # the real box is taller than modelled
    outcome, t_stop = "arrived", None
    try:
        a.move_to_pose(REST)
    except arm_mod.ArmBlocked:
        outcome, t_stop = "blocked", CLOCK[0]
    Arm_Lib.ELECTRONICS["z"] = tuple(config.ELECTRONICS["z"])
    first = Arm_Lib.STATE.get("first_contact_t")
    late = (t_stop - first) if (t_stop and first) else None
    check("servo set %d: stopped, %.2f s after touching" % (k, late if late is not None else -1),
          outcome == "blocked" and late is not None and late <= 3 * config.CONTROL_DT + 0.05)

# =====================================================================
print("\n--- 4. the safety gate: nothing unchecked reaches a motor ---")
a = new_arm(REST, False)
before = dict(Arm_Lib.STATE["pos"])


def refused(fn, kind):
    try:
        fn()
    except kind:
        return True
    except Exception:
        return False
    return False


check("a target that isn't a number is refused",
      refused(lambda: a.move_servos({2: float("nan")}, wait=False), arm_mod.UnsafeAngle))
check("a target past the servo's range is refused",
      refused(lambda: a.move_servos({3: 300.0}, wait=False, check=False), arm_mod.UnsafeAngle))
check("a 40 degree jump in one command is refused",
      refused(lambda: a.move_servos({2: REST[1] - 40.0}, wait=False), arm_mod.UnsafeMove))
check("and none of them reached the servos", all(
    abs(Arm_Lib.STATE["pos"][i] - before[i]) < 1e-9 for i in range(1, 7)))

# a "planner bug": a direct move that swings the forearm down into the electronics
W = world.World.default()
start = [90.0, 180.0, 40.0, 40.0, 90.0, 150.0]           # leaning back, clear of the box
a = new_arm(start, False)
real_plan = safety.plan_pose_move
safety.plan_pose_move = lambda s, t, z, search=True: [{3: 150.0}]
outcome = "moved"
try:
    a.move_to_pose([90, 180, 40, 90, 90, 150])          # any target; the "bug" sends elbow 150
except arm_mod.UnsafeMove as e:
    outcome = str(e)
safety.plan_pose_move = real_plan
check("a planner bug driving into the electronics is stopped by the gate",
      "electronics" in outcome, outcome[:90])
check("before anything touched", Arm_Lib.STATE["contacts"] == 0)
check("and it stopped short of the box",
      W.margin([Arm_Lib.STATE["pos"][i] for i in range(1, 5)])[0] > -1.0,
      "%.1f mm at %s, floor %s" % (W.margin([Arm_Lib.STATE["pos"][i] for i in range(1, 5)])[0], [round(Arm_Lib.STATE["pos"][i], 1) for i in range(1, 5)], a._floor_margin))

# =====================================================================
print("\n--- 5. the control loop ---")
# a servo that misses a read now and then: the move pauses a tick, then carries on
a = new_arm(config.POSE_READY, False)
EVENTS[:] = [(CLOCK[0] + 0.45, lambda: Arm_Lib.SILENT.add(3)),
             (CLOCK[0] + 0.56, lambda: Arm_Lib.SILENT.discard(3))]
outcome = "arrived"
try:
    a.move_to_pose(REST)
except arm_mod.ArmBlocked as e:
    outcome = str(e)
EVENTS[:] = []
check("one missed read doesn't stop the move", outcome == "arrived" and at(REST), outcome)
check("it was counted", a.health["read_misses"] >= 1, "%d" % a.health["read_misses"])

a = new_arm(config.POSE_READY, False)
EVENTS[:] = [(CLOCK[0] + 0.45, lambda: Arm_Lib.SILENT.add(3))]
outcome = "arrived"
try:
    a.move_to_pose(REST)
except arm_mod.ArmBlocked as e:
    outcome = str(e)
EVENTS[:] = []
Arm_Lib.SILENT.clear()
check("a servo that stays silent stops it", "stopped answering" in outcome, outcome[:60])

# slow reads: the loop keeps its schedule instead of running in slow motion
real_read = Arm_Lib.Arm_Device.Arm_serial_servo_read


def slow_read(self, sid):
    fake_sleep(0.012)                 # 4 servos = ~50 ms of every 100 ms tick
    return real_read(self, sid)


Arm_Lib.Arm_Device.Arm_serial_servo_read = slow_read
a = new_arm(config.POSE_READY, False)
t0 = CLOCK[0]
a.move_to_pose(REST)
took = CLOCK[0] - t0
planned = motion.duration(max(abs(REST[i] - config.POSE_READY[i]) for i in range(4)))
check("with slow reads, a move still takes about its planned time",
      took <= planned * 1.1 + 0.6, "%.2f s for a %.2f s move" % (took, planned))


def very_slow_read(self, sid):
    fake_sleep(0.04)                  # 160 ms of reads per 100 ms tick
    return real_read(self, sid)


Arm_Lib.Arm_Device.Arm_serial_servo_read = very_slow_read
a = new_arm(config.POSE_READY, False)
t0 = CLOCK[0]
a.move_to_pose(REST)
took = CLOCK[0] - t0
Arm_Lib.Arm_Device.Arm_serial_servo_read = real_read
check("when ticks run late it skips them instead of bursting (and says so)",
      a.health["overruns"] > 0 and took <= planned * 1.3 + 1.0,
      "%d skipped, %.2f s" % (a.health["overruns"], took))

# =====================================================================
print("\n--- 6. one program on the arm ---")
code = ("import sys; sys.path.insert(0, %r); import config; config.LOCK_FILE = %r\n"
        "from robot import arm\n"
        "try:\n    arm.take_bus_lock(); print('GOT IT')\n"
        "except RuntimeError as e:\n    print('REFUSED', e)\n"
        % (os.path.dirname(HERE), config.LOCK_FILE))
out = subprocess.check_output([sys.executable, "-c", code]).decode()
check("a second program can't take the arm while this one has it",
      out.startswith("REFUSED") and "pid %d" % os.getpid() in out, out.strip()[:90])

print("\n%d failure(s)" % len(failures))
if failures:
    for f in failures:
        print("  - " + f)
    sys.exit(1)
