#!/usr/bin/env python3
"""
python3 run.py --servo-test -- measures how the real servos behave (v2.7.2).

2.5-2.7 were tuned against a simulated servo whose numbers were guesses, and
the real arm kept stopping "N degrees short". This measures instead of
guessing. It moves only a little, from a test pose well above the board
(upright, claw pointing forward-down), and prints a short table to paste
into the chat:

  registers   what each servo's register says right now (pos 0 = no answer)
  bus         how long a read and a write take, and any I2C errors
  step        each joint told +10 then -10 degrees: how long before it moves,
              how long to get 90% of the way, how far off it ends
  sweep       the shoulder streamed 3 degrees a tick (like a real move), three
              ways: four messages back to back, four messages 10 ms apart, one
              write6 message -- does it ever not move when told to?
  clamp       shoulder, elbow, wrist told a little PAST 0-180: do they go there,
              or stop at the end of the scale?
  backlash    the base sent to 90 from above and from below: how far apart
              it ends

Everything also goes to logs/servo_test/<time>.json. Nothing here changes
how the program moves; the next version is tuned from these numbers.
"""
import contextlib
import io
import json
import os
import time

import config
from robot import safety
from robot.arm import ArmBlocked, OutOfReach, UnsafeAngle, UnsafeMove

TEST_POSE = [90, 90, 60, 60, 90, config.CLAW_OPEN]
SHORT = {1: "base", 2: "shoulder", 3: "elbow", 4: "wrist", 5: "roll", 6: "claw"}
# a little past the end of the scale, from a pose where that's out in the open:
# (servo, pose to start from, inside value, past value)
CLAMP_TESTS = [(2, [90, 172, 0, 0, 90, config.CLAW_OPEN], 178.0, 184.0),
               (3, [90, 90, 8, 90, 90, config.CLAW_OPEN], 3.0, -4.0),
               (4, [90, 90, 60, 8, 90, config.CLAW_OPEN], 3.0, -4.0)]
ERRORS = {"i2c": 0}


@contextlib.contextmanager
def _counting():
    """Arm_Lib prints 'I2C error' instead of raising: count those, don't show them."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        yield
    ERRORS["i2c"] += buf.getvalue().count("error")


def _pos(word):
    if not isinstance(word, int):
        return None
    return (word >> 8 & 0xff) | (word << 8 & 0xff00)


def registers(arm):
    """Every servo's register as it is: (our angle, pos, Arm_Lib's angle)."""
    out = {}
    for sid in range(1, 7):
        ours = arm._read_wide(sid)
        word = arm.last_raw.get(sid)
        with _counting():
            lib = arm.bus.Arm_serial_servo_read(sid)
        out[sid] = {"angle": ours, "pos": _pos(word), "word": word if isinstance(word, int)
                    else str(word), "arm_lib": lib}
    return out


def _cmd(arm, angles, ms, mode="single", gap=0.0):
    """
    Sends exactly these angles -- NOT clamped to 0-180 (that's what the clamp
    test asks about) -- through the arm's safety gate, the way `mode` says.
    """
    arm._gate(angles)
    with _counting():
        if mode == "write6":
            full = [round(float(angles.get(sid, arm.sent[sid - 1])), 1) for sid in range(1, 7)]
            with arm.bus_lock:
                arm.bus.Arm_serial_servo_write6(*(full + [int(ms)]))
            for sid in range(1, 7):
                arm.sent[sid - 1] = full[sid - 1]
        else:
            for k, (sid, a) in enumerate(angles.items()):
                if k and gap:
                    time.sleep(gap)
                with arm.bus_lock:
                    arm.bus.Arm_serial_servo_write(sid, round(float(a), 1), int(ms))
                arm.sent[sid - 1] = round(float(a), 1)


def _read(arm, sid, samples=5):
    vals = []
    for _ in range(samples):
        v = arm._read_once(sid)
        if v is not None:
            vals.append(v)
        time.sleep(0.01)
    return sum(vals) / len(vals) if vals else None


def bus_timing(arm):
    t = time.time()
    for _ in range(20):
        arm._read_once(1)
    read_ms = (time.time() - t) / 20 * 1000
    hold = {sid: arm.sent[sid - 1] for sid in (1, 2, 3, 4) if arm.sent[sid - 1] is not None}
    t = time.time()
    for _ in range(10):
        _cmd(arm, hold, 200)
    write_ms = (time.time() - t) / 10 / max(1, len(hold)) * 1000
    write6_ms = None
    if None not in arm.sent and hasattr(arm.bus, "Arm_serial_servo_write6"):
        t = time.time()
        for _ in range(10):
            _cmd(arm, hold, 200, mode="write6")
        write6_ms = (time.time() - t) / 10 * 1000
    return {"read_ms": round(read_ms, 1), "write_ms": round(write_ms, 1),
            "write6_ms": None if write6_ms is None else round(write6_ms, 1)}


def step(arm, sid, delta, ms=300, listen=1.2):
    """Tells one joint `delta` degrees further, then watches it for `listen` seconds."""
    before = _read(arm, sid)
    target = arm.sent[sid - 1] + delta
    _cmd(arm, {sid: target}, ms)
    t0 = time.time()
    trace = []
    while time.time() - t0 < listen:
        v = arm._read_once(sid)
        if v is not None:
            trace.append((round(time.time() - t0, 3), v))
        time.sleep(0.005)
    sign = 1.0 if delta > 0 else -1.0
    result = {"sent": round(target, 1), "before": before, "trace": trace}
    if before is None or len(trace) < 5:
        return result
    end = sum(v for _, v in trace[-8:]) / len(trace[-8:])
    moved = [t for t, v in trace if (v - before) * sign >= 1.0]
    most = [t for t, v in trace if (v - before) * sign >= 0.9 * abs(delta)]
    result.update({"delay_ms": int(moved[0] * 1000) if moved else None,
                   "t90_ms": int(most[0] * 1000) if most else None,
                   "end_err": round(end - target, 1),
                   "overshoot": round(max(0.0, max((v - end) * sign for _, v in trace)), 1)})
    return result


def sweep(arm, mode, gap=0.0, sid=2, per_tick=3.0, ticks=5):
    """The shoulder streamed like a real move: down `ticks` ticks, back up, all 4 joints told
    every tick. Counts ticks where the command moved but the joint didn't."""
    dt = config.CONTROL_DT
    home = {s: arm.sent[s - 1] for s in (1, 2, 3, 4)}
    cmds = [home[sid] - per_tick * k for k in range(1, ticks + 1)]
    cmds += [cmds[-1] + per_tick * k for k in range(1, ticks + 1)]
    rows, prev = [], _read(arm, sid, 2)
    deadline = time.time()
    for c in cmds:
        tick = dict(home)
        tick[sid] = c
        _cmd(arm, tick, config.STREAM_MS, mode=mode, gap=gap)
        deadline += dt
        time.sleep(max(0.0, deadline - time.time()))
        got = arm._read_once(sid)
        rows.append((round(c, 1), got))
    time.sleep(0.5)
    final = _read(arm, sid)
    stuck = 0
    for k in range(2, len(rows)):            # the first ticks: still getting going
        a, b = rows[k - 1][1], rows[k][1]
        if a is not None and b is not None and abs(b - a) < 0.3:
            stuck += 1
    lags = [abs(g - c) for c, g in rows if g is not None]
    return {"mode": mode if not gap else "%s+%dms" % (mode, gap * 1000), "rows": rows,
            "stuck": stuck, "of": len(rows) - 2, "worst_lag": round(max(lags), 1) if lags else None,
            "end_err": None if final is None else round(final - cmds[-1], 1),
            "before": prev}


def clamp(arm, sid, pose, inside, past):
    """From `pose`, tell joint `sid` `inside` then `past` (beyond 0-180): where does it end?"""
    arm.move_to_pose(pose)
    out = {}
    for value in (inside, past):
        target = dict((s, arm.sent[s - 1]) for s in (1, 2, 3, 4))
        target[sid] = value
        if not safety.is_clear([target[s] for s in (1, 2, 3, 4)], arm.zone):
            out[str(value)] = "skipped (not clear)"
            continue
        _cmd(arm, {sid: value}, 500)
        time.sleep(1.0)
        got = _read(arm, sid)
        out[str(value)] = None if got is None else round(got, 1)
        out["pos_" + str(value)] = _pos(arm.last_raw.get(sid))
    _cmd(arm, {sid: pose[sid - 1]}, 600)
    time.sleep(0.8)
    return out


def backlash(arm, center=90.0, side=4.0, rounds=3):
    above, below = [], []
    for _ in range(rounds):
        for start, bucket in ((center + side, above), (center - side, below)):
            _cmd(arm, {1: start}, 300)
            time.sleep(0.6)
            _cmd(arm, {1: center}, 300)
            time.sleep(0.8)
            v = _read(arm, 1)
            if v is not None:
                bucket.append(v)
    return {"from_above": above, "from_below": below}


def _mean(vals):
    return sum(vals) / len(vals) if vals else None


def _f(v, fmt="%.1f"):
    return "--" if v is None else fmt % v


def run(arm, power_on):
    """`power_on(arm)`: run.py's power-on (moves nothing; push the arm if a joint can't
    report). Returns the results dict."""
    report = {"version": config.VERSION, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
              "write_mode": config.WRITE_MODE}
    regs = registers(arm)
    report["registers"] = regs
    print("Registers (angle [pos], pos 0 = no answer):")
    cells = ["%s %s [%s]" % (SHORT[s], _f(regs[s]["angle"]), _f(regs[s]["pos"], "%d"))
             for s in range(1, 7)]
    print("  " + "   ".join(cells[:3]))
    print("  " + "   ".join(cells[3:]))
    if not power_on(arm):
        print("  Power isn't on -- can't move for the rest of the test.")
        return _save(report)
    report["bus"] = bus = bus_timing(arm)
    print("Bus: read %s ms, write %s ms per servo, write6 %s ms"
          % (_f(bus["read_ms"]), _f(bus["write_ms"]), _f(bus["write6_ms"])))
    print("Going to the test pose (upright, claw forward-down)...", flush=True)
    try:
        arm.move_to_pose(TEST_POSE)
    except (ArmBlocked, UnsafeMove, OutOfReach, UnsafeAngle, RuntimeError) as e:
        report["test_pose"] = str(e)
        print("  Couldn't: %s" % e)
        return _save(report)
    arm.clear_block()

    print("Step +10 / -10, sent with 300 ms (delay, 90% at, ends off by):")
    report["step"] = {}
    for sid in (1, 2, 3, 4):
        up = step(arm, sid, 10.0)
        down = step(arm, sid, -10.0)
        report["step"][SHORT[sid]] = [up, down]
        print("  %-8s +10: %s ms, %s ms, %s    -10: %s ms, %s ms, %s" % (
            SHORT[sid], _f(up.get("delay_ms"), "%d"), _f(up.get("t90_ms"), "%d"),
            _f(up.get("end_err"), "%+.1f"), _f(down.get("delay_ms"), "%d"),
            _f(down.get("t90_ms"), "%d"), _f(down.get("end_err"), "%+.1f")), flush=True)

    print("Shoulder streamed 3 deg a tick (didn't move / ticks, worst lag, ends off by):")
    report["sweep"] = []
    cells = []
    for mode, gap in (("single", 0.0), ("single", 0.010), ("write6", 0.0)):
        if mode == "write6" and (None in arm.sent or not hasattr(arm.bus,
                                                                 "Arm_serial_servo_write6")):
            continue
        r = sweep(arm, mode, gap)
        report["sweep"].append(r)
        cells.append("%s %d/%d, %s, %s" % (r["mode"], r["stuck"], r["of"], _f(r["worst_lag"]),
                                           _f(r["end_err"], "%+.1f")))
    print("  " + "   ".join(cells))

    print("Told past 0-180 (sent -> read):", flush=True)
    report["clamp"] = {}
    cells = []
    for sid, pose, inside, past in CLAMP_TESTS:
        try:
            r = clamp(arm, sid, pose, inside, past)
        except (ArmBlocked, UnsafeMove, OutOfReach, UnsafeAngle) as e:
            r = {"error": str(e)}
            arm.clear_block()
        report["clamp"][SHORT[sid]] = r
        if "error" in r:
            cells.append("%s: couldn't (%s)" % (SHORT[sid], r["error"][:40]))
        else:
            cells.append("%s %g->%s %g->%s" % (SHORT[sid], inside, r.get(str(inside)), past,
                                               r.get(str(past))))
    print("  " + "   ".join(cells))

    try:
        arm.move_to_pose(TEST_POSE)
        b = backlash(arm)
        report["backlash"] = b
        hi, lo = _mean(b["from_above"]), _mean(b["from_below"])
        print("Base sent to 90: from above %s, from below %s -> %s deg apart"
              % (_f(hi), _f(lo), _f(None if hi is None or lo is None else hi - lo)))
    except (ArmBlocked, UnsafeMove, OutOfReach, UnsafeAngle) as e:
        report["backlash"] = str(e)
        print("Base backlash: couldn't (%s)" % e)
    report["i2c_errors"] = ERRORS["i2c"]
    if ERRORS["i2c"]:
        print("I2C errors: %d" % ERRORS["i2c"])
    return _save(report)


def _save(report):
    folder = os.path.join(config.LOGS_DIR, "servo_test")
    try:
        if not os.path.isdir(folder):
            os.makedirs(folder)
        path = os.path.join(folder, time.strftime("%Y%m%d-%H%M%S") + ".json")
        with open(path, "w") as fh:
            json.dump(report, fh, indent=1, default=str)
        print("Saved %s -- paste the lines above into the chat." % os.path.relpath(
            path, config.ROOT))
    except OSError as e:
        print("(couldn't save the details: %s)" % e)
    return report
