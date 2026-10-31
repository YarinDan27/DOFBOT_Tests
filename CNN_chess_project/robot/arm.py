#!/usr/bin/env python3
"""
Talks to the servos. The only file that does.

Safety rules, all enforced here:
  1. every target is inside what its servo can do (config.JOINT_RANGE), and
     (v2.5) every pose and every move is checked against the arm's real shape
     -- solid links with thickness -- and what's around it: the electronics,
     the turning base, the base plate, the table and the board
     (robot/world.py). No per-servo "safety limits": any combination of
     joints is fine if the arm is clear, and refused if it isn't.
  2. moves near the board follow a planned straight line in space, cut into
     small steps, instead of letting the joints swing wherever they like
  3. the WHOLE path is solved before the first step runs -- if any point on it
     is unreachable, nothing moves at all
  4. every joint swing -- going home, parking, the jump at the start of a
     straight line -- is simulated by robot/safety.py first, and refused if
     any part of the arm would sweep over the board low
  5. power comes on aimed at exactly where the arm already is: no jump, even
     if a joint is past its move limits. If a servo can't be read, power does
     NOT come on and nothing moves -- you move the arm by hand instead
  6. (v0.2.2) every move is checked against what the servos report back,
     in small sub-steps. If a joint falls short of where it was sent --
     something is in the way, or the arm isn't where the program thought --
     it stops and holds still (ArmBlocked). Rules 4 and 5 trust the servo
     readings; this one catches it when the readings were wrong.

Every command and read-back goes to logs/motion/, so a bad move can be
replayed afterwards.
"""
import json
import math
import os
import sys
import threading
import time

import config
from robot import kinematics, motion, safety, world


class OutOfReach(Exception):
    pass


class UnsafeAngle(Exception):
    pass


class UnsafeMove(Exception):
    pass


class ArmBlocked(Exception):
    """A joint didn't reach where it was sent. The arm is holding still."""


class StillLimp(Exception):
    """You chose to leave servo power off. The arm is floppy: put it down."""


SERVO_NAMES = {1: "base", 2: "shoulder", 3: "elbow", 4: "wrist", 5: "wrist roll", 6: "claw"}


def silent_servos(angles, ids=(1, 2, 3, 4)):
    """
    Which of these servos didn't answer, as text ('servo 3 (elbow)'), or None
    if they all did. Servos 1-4 decide where the claw is; 5 and 6 only turn
    or open it in place, so they're optional.
    """
    silent = [i for i in ids if angles[i - 1] is None]
    if not silent:
        return None
    return " and ".join("servo %d (%s)" % (i, SERVO_NAMES[i]) for i in silent)


OUT_OF_RANGE_HINT = ("A servo can't report its angle while its joint is pushed past "
                     "0-180 degrees -- ease that joint back toward the middle.")

# What to physically do when a servo can't report (its joint is past 0-180).
FIX_BY_HAND = {
    1: "turn the base back toward the middle",
    2: "bring the upper arm back toward upright",
    3: "move the elbow back toward the middle of its swing",
    4: "straighten the wrist -- line the claw up with the forearm",
}


def outside_limits(angles):
    """[(servo, angle, low, high)] for servos 1-4 outside what the servo can do
    (config.JOINT_RANGE)."""
    out = []
    for sid in (1, 2, 3, 4):
        a = angles[sid - 1]
        if a is None:
            continue
        low, high = config.JOINT_RANGE[sid]
        if a < low or a > high:
            out.append((sid, a, low, high))
    return out


def _which_way(sid, too_low):
    if sid == 1:
        return "turn the base back toward the middle"
    if too_low:
        return "that joint is bent too far toward the board -- tilt it back up a little"
    return ("that joint is bent too far back, toward the electronics -- tilt it "
            "forward a little")


def limit_problem(angles):
    """Text if a servo is past what it can do (JOINT_RANGE), or None."""
    bad = outside_limits(angles)
    if not bad:
        return None
    sid, a, low, high = bad[0]
    return ("servo %d (%s) is at %.0f, past its range %.0f-%.0f: %s"
            % (sid, SERVO_NAMES[sid], a, low, high, _which_way(sid, a < low)))


def pose_problem(angles, zone=None):
    """
    Why the arm can't be asked to go to (or start from) these angles, or
    None: a servo that didn't answer, one past its range, or -- with a World
    -- the arm's shape too close to something solid. Includes what to do.
    """
    silent = silent_servos(angles)
    if silent:
        return silent + " didn't answer. " + OUT_OF_RANGE_HINT
    problem = limit_problem(angles)
    if problem or zone is None or not hasattr(zone, "body_problem"):
        return problem
    return zone.body_problem(angles)


def other_arm_programs():
    """
    Other programs that may be talking to the same servos (and camera):
    [(pid, what, ours)] -- ours=True for another copy of this program (which
    we can stop), False for something else (e.g. Yahboom's ROS driver, root).
    Two programs on one arm means mixed-up readings and fighting commands.
    """
    found = []
    me = os.getpid()
    root = os.path.realpath(config.ROOT)
    try:
        pids = [int(d) for d in os.listdir("/proc") if d.isdigit()]
    except OSError:
        return found
    for pid in pids:
        if pid in (me, os.getppid()):
            continue
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as fh:
                args = [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        if not args:
            continue
        cmd = " ".join(args)
        # judged by the program's own arguments, not by words anywhere in a
        # command line (a shell running "grep run.py" is not this program)
        program = os.path.basename(args[0])
        if (program.startswith("python") or program in ("roslaunch", "rosrun")) and any(
                a.endswith("arm.launch") or
                os.path.splitext(os.path.basename(a))[0] in ("dofbot_driver", "arm_driver")
                for a in args[1:]):
            found.append((pid, "Yahboom's ROS arm driver (%s)" % cmd[:70], False))
        elif program.startswith("python") and len(args) > 1 and \
                os.path.basename(args[1]) == "run.py":
            try:
                cwd = os.path.realpath(os.readlink("/proc/%d/cwd" % pid))
            except OSError:
                cwd = ""
            if cwd == root:
                found.append((pid, "another copy of this program (%s)" % cmd[:70], True))
    return found


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


_LOCK = {"file": None}


def take_bus_lock():
    """
    Only one program drives the arm (after Microduck's robotd, which holds a
    lock on its bus for as long as it runs). An exclusive lock on
    config.LOCK_FILE, kept until this program exits -- the system lets go of
    it even if the program is killed. A second copy gets a RuntimeError
    naming the first. The file itself is never deleted.
    """
    if _LOCK["file"] is not None:
        return
    try:
        import fcntl
    except ImportError:          # not Linux: nothing to lock with
        return
    folder = os.path.dirname(config.LOCK_FILE)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    fh = open(config.LOCK_FILE, "a+")
    try:
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (IOError, OSError):
        fh.seek(0)
        owner = fh.read().strip()
        fh.close()
        raise RuntimeError("another program is driving the arm (%s) -- only one may at a "
                           "time" % (owner or "it didn't say which"))
    fh.seek(0)
    fh.truncate()
    fh.write("pid %d: %s" % (os.getpid(), " ".join(sys.argv)[:120]))
    fh.flush()
    _LOCK["file"] = fh


def _outside_by(servo_id, angle):
    """How far past its range a servo target is (0 = inside)."""
    low, high = config.JOINT_RANGE[servo_id]
    return max(0.0, low - angle, angle - high)


def _clamp(servo_id, angle):
    low, high = config.JOINT_RANGE[servo_id]
    return max(low, min(high, angle))


class Arm:
    def __init__(self, power_on=True):
        # Imported here, not at the top, so the planning code can still be
        # imported and tested on the laptop, where Arm_Lib doesn't exist.
        take_bus_lock()
        from Arm_Lib import Arm_Device
        self.bus = Arm_Device()
        # one I2C conversation at a time (the live 3D view runs in a thread)
        self.bus_lock = threading.RLock()
        time.sleep(0.1)
        self.is_limp = False
        self.powered = False        # True once stiffen_here() has switched on safely
        self.blocked = False
        self.skip_park = False      # set when you quit with Ctrl-C: no moving on the way out
        self.stop_event = threading.Event()   # STOP button: checked before every sub-step
        self.interactive = True     # False: never stop to ask in the terminal (run.py)
        self.zone = world.World.default()
        self.home_pose = list(config.POSE_REST)
        # for the live 3D view: what the servos last said / were last sent
        self.last_angles = [None] * 6
        self.last_read = [None] * 6     # what the servos last SAID (last_angles also takes sends)
        self.last_seen_t = 0.0
        self.last_plan = []         # [[s1..s4], ...] of the move being made
        # what each servo was last sent: the gate compares every command with it
        self.sent = [None] * 6
        # how close to something solid the arm was when last FOUND somewhere
        # (power-on, holding still): moves may never take it closer than that
        self._floor_margin = None
        # how the control loop is doing (shown in the 3D view, saved in the log)
        self.health = {"moves": 0, "ticks": 0, "overruns": 0, "read_misses": 0,
                       "refused": 0, "worst_tick_s": 0.0, "hz": None}
        self._log_file = None
        self._open_log()
        # Power on aimed at where it already is -- only if servos 1-4 answer.
        # Otherwise nothing changes; start_up() deals with it.
        self.start_problem = None
        if not power_on:                  # run.py --check: only look
            return
        try:
            self.stiffen_here(required=False)
        except ArmBlocked as e:
            self.start_problem = e        # reported by start_up(), nothing more moves

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is StillLimp or (self.is_limp and exc_type is KeyboardInterrupt):
            print("\n  Servo power is OFF -- the arm is floppy. Lay it down gently.")
            return exc_type is StillLimp       # StillLimp: end quietly, no traceback
        if self.blocked:
            print("\n  Not parking: the last move was blocked. The arm is holding "
                  "still where it stopped.")
            return False
        if exc_type is KeyboardInterrupt or self.skip_park:
            if exc_type is KeyboardInterrupt:
                print("\n  Not parking (you pressed Ctrl-C). The arm is holding still.")
            return False
        if self.is_limp and not self.interactive:
            print("\n  Servo power is OFF -- the arm is floppy. Lay it down gently.")
            return False
        try:
            if self.is_limp:
                self.power_on_with_help()
            self.park()
        except StillLimp:
            print("  Servo power left OFF -- lay the arm down gently.")
        except Exception as e:
            print("  Didn't park (%s)." % e)
        return False

    # ---------------- motion log ----------------
    def _open_log(self):
        try:
            folder = os.path.join(config.LOGS_DIR, "motion")
            if not os.path.isdir(folder):
                os.makedirs(folder)
            path = os.path.join(folder, time.strftime("%Y%m%d-%H%M%S") + ".log")
            self._log_file = open(path, "a")
        except Exception:
            self._log_file = None

    def log(self, event, **data):
        if self._log_file is None:
            return
        try:
            data["t"] = round(time.time(), 3)
            data["event"] = event
            self._log_file.write(json.dumps(data) + "\n")
            self._log_file.flush()
        except Exception:
            pass

    # ---------------- where things are ----------------
    def set_board(self, frame):
        """
        What's around the arm: the taught board (or, before one is taught, a
        big area in front of the arm) plus the electronics, base and table.
        """
        self.zone = world.World.for_board(frame)

    # ---------------- raw servo access ----------------
    def _check(self, servo_id, angle):
        low, high = config.JOINT_RANGE[servo_id]
        if not (low <= angle <= high):
            raise UnsafeAngle(
                "servo %d angle %.1f is past its range %s-%s" % (servo_id, angle, low, high))

    def _write(self, servo_id, angle, duration_ms):
        """The bus write itself. Only _gate()-checked moves and hold-still
        commands (stay exactly where you are) call this."""
        with self.bus_lock:
            self.bus.Arm_serial_servo_write(servo_id, int(round(angle)), int(duration_ms))
        self.last_angles[servo_id - 1] = float(angle)
        self.sent[servo_id - 1] = float(angle)

    def _found_at(self, angles):
        """The arm was found here (not sent): how close it is becomes the floor."""
        if hasattr(self.zone, "margin") and None not in list(angles[:4]):
            self._floor_margin = self.zone.margin(list(angles[:4]))[0]

    def _refuse(self, error):
        self.health["refused"] += 1
        self.log("refused", why=str(error))
        raise error

    def _gate(self, angles):
        """
        The last check before a move command reaches a motor, whoever planned
        it -- after Microduck's safety layer, which owns the only write
        handle so nothing above it can send a target unchecked. Refused (not
        clamped), and nothing is sent, if:
          * a target isn't a number
          * it's past what the servo can do (JOINT_RANGE) -- unless it's on
            the way back in from further out
          * servos 1-4 would jump more than GATE_STEP_DEG from where they were
            last sent: every planned move goes in small steps, so a big jump
            means a bug
          * the commanded pose is too close to something solid
            (robot/world.py) -- unless the arm was FOUND at least that close
            (powered on there, or stopped there), when it may move about but
            never closer: a move can't creep in a little at a time
        """
        for sid, a in angles.items():
            if not isinstance(a, (int, float)) or not math.isfinite(a):
                self._refuse(UnsafeAngle("servo %d was sent %r, which isn't an angle" % (sid, a)))
            last = self.sent[sid - 1]
            over = _outside_by(sid, a)
            if over > 0.0 and (last is None or over > _outside_by(sid, last) + 0.5):
                low, high = config.JOINT_RANGE[sid]
                self._refuse(UnsafeAngle("servo %d angle %.1f is past its range %s-%s"
                                         % (sid, a, low, high)))
            if sid <= 4 and last is not None and abs(a - last) > config.GATE_STEP_DEG:
                self._refuse(UnsafeMove(
                    "servo %d (%s) was told to jump %.0f degrees in one go -- moves go in small "
                    "steps, so that's a bug; nothing was sent" % (sid, SERVO_NAMES[sid],
                                                                  abs(a - last))))
        joints = [angles.get(sid, self.sent[sid - 1]) for sid in (1, 2, 3, 4)]
        if None in joints or not hasattr(self.zone, "margin"):
            return
        # the servos get whole degrees (_write rounds): check exactly that pose
        joints = [float(int(round(a))) for a in joints]
        m, link, ob, _ = self.zone.margin(joints)
        floor = self._floor_margin if self._floor_margin is not None else 0.0
        if m < 0.0 and m < min(floor, 0.0) - 0.5:
            self._refuse(UnsafeMove("that would put the %s too close to the %s; nothing "
                                    "was sent" % (link, ob)))

    def set_servo(self, servo_id, angle, duration_ms=400):
        self._check(servo_id, angle)
        self._gate({servo_id: angle})
        self._write(servo_id, angle, duration_ms)

    def read_servo(self, servo_id):
        """
        One servo's angle, or None.

        Arm_Lib's Arm_serial_servo_read throws away any angle outside 0-180
        (servo 5: 0-270) -- so a joint just past the end of its scale, like
        the shoulder at ~182 in the folded pose, or the wrist at -5 with the
        claw vertical on a piece, looks like "no answer". When that happens,
        _read_wide reads the same register with the same maths, minus that
        cut-off: up to WIDE_READ_DEG past either end.

        (NOT Arm_serial_servo_read_any: it skips the 180-minus flip that
        servos 2-4 need, so a wrist at -6 comes back as 186. That is what v0.2.1
        used, and the likely cause of the 2026-10-24 crash.)
        """
        for _ in range(3):
            with self.bus_lock:
                value = self.bus.Arm_serial_servo_read(servo_id)
                if value is None:
                    value = self._read_wide(servo_id)
            if value is not None:
                return float(value)
            time.sleep(0.05)
        return None

    def _read_wide(self, servo_id):
        """Arm_serial_servo_read without the 0-180 cut-off (see read_servo)."""
        bus = getattr(self.bus, "bus", None)
        addr = getattr(self.bus, "addr", None)
        if bus is None or addr is None:
            return None
        try:
            bus.write_byte_data(addr, servo_id + 0x30, 0x0)
            time.sleep(0.003)
            word = bus.read_word_data(addr, servo_id + 0x30)
        except Exception:
            return None
        if not word:
            return None                         # 0 = no data from that servo
        pos = (word >> 8 & 0xff) | (word << 8 & 0xff00)
        if servo_id == 5:
            angle, top = 270.0 * (pos - 380) / (3700 - 380), 270.0
        else:
            angle, top = 180.0 * (pos - 900) / (3100 - 900), 180.0
        if not -config.WIDE_READ_DEG <= angle <= top + config.WIDE_READ_DEG:
            return None
        angle = int(angle)                      # whole degrees, like Arm_Lib
        if servo_id in (2, 3, 4):
            angle = 180 - angle                 # same flip as Arm_Lib
        return float(angle)

    def read_all(self):
        return [self.read_servo(i) for i in range(1, 7)]

    def read_angles(self, samples=1, ids=(1, 2, 3, 4, 5, 6)):
        """
        Servo angles (list of 6, None for unread ones), each averaged over
        `samples` readings. Averaging matters when teaching: a single reading
        can be off by a degree, and one degree at the shoulder is ~3mm at the claw.
        """
        totals = [0.0] * 6
        counts = [0] * 6
        for _ in range(samples):
            for sid in ids:
                value = self.read_servo(sid)
                if value is not None:
                    totals[sid - 1] += value
                    counts[sid - 1] += 1
            if samples > 1:
                time.sleep(0.02)
        out = [totals[i] / counts[i] if counts[i] else None for i in range(6)]
        for i in range(6):
            if out[i] is not None:
                self.last_angles[i] = out[i]
                self.last_read[i] = out[i]
        self.last_seen_t = time.time()
        return out

    def move_servos(self, angles, duration_ms=400, wait=True, check=True):
        """
        Sends angles (through the gate), no read-back. Use _move_checked for
        servos 1-4. check=False only for sub-steps on the way back INTO the
        range from a pose past it (the gate still allows only that).
        """
        if check:
            for servo_id, angle in angles.items():
                self._check(servo_id, angle)
        self._gate(angles)
        for servo_id, angle in angles.items():
            self._write(servo_id, angle, duration_ms)
        if wait:
            time.sleep(duration_ms / 1000.0 + 0.05)

    def health_line(self):
        """One line on how the motor loop has been doing (shown on quit and in the 3D view)."""
        h = self.health
        if not h["moves"]:
            return "Motor loop: no moves yet"
        return ("Motor loop: %d move%s at %.1f Hz (aim %.0f), %d late ticks skipped, "
                "%d missed reads, %d commands refused"
                % (h["moves"], "" if h["moves"] == 1 else "s", h["hz"] or 0.0,
                   1.0 / config.CONTROL_DT, h["overruns"], h["read_misses"], h["refused"]))

    def _start_watch(self, watch, sids, begin=None):
        """
        Tell the stall watch where each joint sits relative to its command
        before a move: a loaded joint sags a few degrees and stays about that
        far off all move long. That's not "behind"; only what changes is.
        """
        sids = [sid for sid in sids if sid <= 4 and sid not in watch.offset]
        if not sids:
            return
        if time.time() - self.last_seen_t > 0.3:
            self._read_joints_once()
        for sid in sids:
            cmd = self.sent[sid - 1]
            if cmd is None and begin is not None:
                cmd = begin.get(sid)
            if self.last_read[sid - 1] is not None and cmd is not None:
                watch.start(sid, self.last_read[sid - 1] - cmd)

    def _move_checked(self, angles, duration_ms, check=True, watch=None):
        """
        Sends angles, waits, then reads servos 1-4 back. A joint that stops
        following (motion.StallWatch, shared across the steps of one plan) or
        is more than SETTLE_DEG off on its own = blocked: hold still right
        there and raise ArmBlocked. Refuses to move at all while blocked.
        """
        self._ensure_not_blocked()
        if self.stop_event.is_set():
            self._hold_still(self.read_angles(samples=1, ids=(1, 2, 3, 4)))
            raise ArmBlocked("you pressed STOP")
        if watch is None:
            watch = motion.StallWatch(lag_max=config.SETTLE_DEG)
        self._start_watch(watch, angles)
        self.move_servos(angles, duration_ms=duration_ms, check=check)
        ids = [sid for sid in angles if sid <= 4]
        if not ids:
            self.log("move", sent=angles, ms=duration_ms)
            return
        got = self.read_angles(samples=1, ids=(1, 2, 3, 4))
        self.log("move", sent=angles, ms=duration_ms, got=got[:4])
        for sid in ids:
            if got[sid - 1] is None:
                self._hold_still(got)
                raise ArmBlocked("servo %d (%s) stopped answering during a move"
                                 % (sid, SERVO_NAMES[sid]))
            stalled, short = watch.update(sid, angles[sid], got[sid - 1])
            if stalled:
                self._hold_still(got)
                raise ArmBlocked(
                    "servo %d (%s) stopped %.0f degrees short of where it was sent -- "
                    "something is in the way, or the arm wasn't where I thought"
                    % (sid, SERVO_NAMES[sid], short))

    def _ensure_not_blocked(self):
        if self.blocked:
            raise ArmBlocked("the last move was blocked -- not moving until you say "
                             "the way is clear")

    def _hold_still(self, got):
        """Stop pushing: make every target where the servo actually is now."""
        self.blocked = True
        for sid in (1, 2, 3, 4):
            if got[sid - 1] is not None:
                self._write(sid, got[sid - 1], 300)
        self._found_at(got)
        self.log("blocked", got=got[:4])

    def clear_block(self):
        """You checked the arm and the way is clear: allow moves again."""
        self.blocked = False
        self.stop_event.clear()
        self.log("unblocked")

    # ---------------- feedback you can see/hear at the arm ----------------
    def beep(self, times=1, long=False):
        """
        Beeps on the arm's buzzer (if the board has one): `times` short beeps,
        or one long one. This is how the arm talks to you when you're standing
        at it, away from the computer.
        """
        if not hasattr(self.bus, "Arm_Buzzer_On"):
            return
        # Arm_Buzzer_On(n) sounds for n x 100 ms, then stops by itself
        units = 6 if long else 1
        for i in range(1 if long else max(1, int(times))):
            try:
                with self.bus_lock:
                    self.bus.Arm_Buzzer_On(units)
            except Exception:
                return
            time.sleep(units * 0.1 + 0.2)

    def power_on_when_readable(self, timeout=None, show=True, light=False):
        """
        Switches power on in place -- holding still, no jump -- as soon as
        servos 1-4 all report. No Enter needed. Says which joint is missing
        (a new line only when that changes -- no screen spam). light=True:
        red while a joint can't report. Returns True once powered, False if
        `timeout` seconds pass. ArmBlocked passes through (a servo didn't
        stay where it said it was).
        """
        start = time.time()
        said = None
        while True:
            angles = self.read_angles(samples=1, ids=(1, 2, 3, 4))
            silent = [s for s in (1, 2, 3, 4) if angles[s - 1] is None]
            if not silent:
                try:
                    self.stiffen_here()
                except RuntimeError:
                    pass                 # one went quiet again in between: keep waiting
                else:
                    if light:
                        self.light(0, 0, 0)
                    return True
            else:
                message = "  waiting: %s can't report -- %s" % (
                    SERVO_NAMES[silent[0]], FIX_BY_HAND[silent[0]])
                if message != said:
                    said = message
                    if show:
                        print(message, flush=True)
                    if light:
                        self.light(90, 0, 0)
            if timeout is not None and time.time() - start > timeout:
                return False
            time.sleep(0.3)

    def light(self, red, green, blue):
        """The arm's RGB light (if the board has one)."""
        try:
            if hasattr(self.bus, "Arm_RGB_set"):
                with self.bus_lock:
                    self.bus.Arm_RGB_set(int(red), int(green), int(blue))
        except Exception:
            pass

    # ---------------- teaching by hand ----------------
    def limp(self):
        """Servo power off: the arm goes floppy. HOLD IT before calling this."""
        with self.bus_lock:
            self.bus.Arm_serial_set_torque(0)
        self.is_limp = True
        self.powered = False
        self.log("limp")

    def stiffen_here(self, required=True):
        """
        Servo power on WITHOUT a jump. A servo switched on drives to its last
        target, which could be far from where the arm is now -- so first make
        every target "where you already are", then switch on.

        Needs servos 1-4 to answer. A pose past the robot's move limits is
        fine: the servos are told to stay exactly where they are, so nothing
        moves -- getting back inside the limits is a separate, checked move.
        If a servo is silent: required -> RuntimeError; not required -> leave
        the power alone and return False.
        """
        angles = self.read_angles(samples=3)
        silent = silent_servos(angles)
        if silent:
            first = [s for s in (1, 2, 3, 4) if angles[s - 1] is None][0]
            problem = "%s can't report its angle -- %s" % (silent, FIX_BY_HAND[first])
            self.log("stiffen_refused", got=angles, why=problem)
            if required:
                raise RuntimeError(problem)
            return False
        for servo_id, angle in enumerate(angles, start=1):
            if angle is None:          # wrist roll / claw: optional
                print("  (%s didn't answer -- it may turn a little when power "
                      "comes on)" % SERVO_NAMES[servo_id])
                continue
            # exactly where it is -- even past a move limit: holding still is safe
            self._write(servo_id, angle, 800)
        time.sleep(0.1)
        with self.bus_lock:
            self.bus.Arm_serial_set_torque(1)
        time.sleep(0.9)
        self.is_limp = False
        self.powered = True
        self.log("stiffen", got=angles)
        self._found_at(angles)
        # Every servo should now be exactly where it said it was. If one isn't,
        # its reading was wrong (it just moved to the wrong angle) or something
        # is pressing on the arm. Don't chase it: hold, and stop everything.
        after = self.read_angles(samples=1, ids=(1, 2, 3, 4))
        for sid in (1, 2, 3, 4):
            if after[sid - 1] is None or abs(after[sid - 1] - angles[sid - 1]) > config.STALL_DEG:
                self.blocked = True
                self.log("stiffen_mismatch", told=angles[:4], got=after[:4])
                raise ArmBlocked(
                    "right after power-on, servo %d (%s) reads %s but was told to stay at "
                    "%.0f -- its reading is wrong, or something is pressing on the arm"
                    % (sid, SERVO_NAMES[sid], "nothing" if after[sid - 1] is None
                       else "%.0f" % after[sid - 1], angles[sid - 1]))
        return True

    def live_status(self):
        """
        One line describing what the servos say right now: the claw tip and
        its tilt if everything answers, otherwise which joint to move and how.
        Returns (line, ready).
        """
        angles = self.read_angles(samples=1, ids=(1, 2, 3, 4))
        joints = "  ".join("%s %s" % (SERVO_NAMES[s], "--" if angles[s - 1] is None
                                      else "%.0f" % angles[s - 1]) for s in (1, 2, 3, 4))
        silent = [s for s in (1, 2, 3, 4) if angles[s - 1] is None]
        if silent:
            return "%s  | %s" % (joints, FIX_BY_HAND[silent[0]]), False
        x, y, z, approach = kinematics.forward(*angles[:4])
        return ("%s  | claw x=%.0f y=%.0f z=%.0f, tilt %.0f deg  | ready"
                % (joints, x, y, z, abs(180.0 - approach))), True

    def live_prompt(self, prompt):
        """
        Asks `prompt` while showing the servo readings live, updated a few
        times a second, so you can see the moment a joint becomes readable.
        Returns what you typed. (Not a terminal, e.g. in tests: plain input.)
        """
        import select
        import sys
        try:
            interactive = sys.stdin.isatty() and sys.stdout.isatty()
        except (AttributeError, ValueError):
            interactive = False
        if not interactive:
            return input(prompt)
        print(prompt)
        width = 0
        while True:
            line, _ = self.live_status()
            line = "  " + line
            sys.stdout.write("\r" + line.ljust(width))
            sys.stdout.flush()
            width = max(width, len(line))
            ready, _, _ = select.select([sys.stdin], [], [], 0.3)
            if ready:
                typed = sys.stdin.readline()
                sys.stdout.write("\n")
                return typed.rstrip("\n")

    def power_on_with_help(self):
        """
        Asks you to hold the arm, then powers on without a jump. While it waits
        it shows the readings live and says which joint to move if one can't
        report. You can leave the power off instead (StillLimp).
        """
        print("\nHOLD THE ARM -- servo power is coming back on.")
        while True:
            ans = self.live_prompt("  Hold it. When the line below says 'ready', press Enter "
                                   "(q = leave the power off and quit):")
            if ans.strip().lower() == "q":
                raise StillLimp()
            try:
                self.stiffen_here()
                return
            except RuntimeError as e:
                print("  Not yet: %s" % e)

    def lift_by_hand(self, why="the arm is down near the board and every way out "
                              "would sweep over it"):
        """You move the arm by hand (power off), then it powers on there."""
        print("\n  I can't move by myself: %s." % why)
        if not self.is_limp:
            input("  HOLD THE ARM and press Enter -- its power goes off: ")
            self.limp()
        self.live_prompt("  Move it well above the board, roughly upright, with the wrist "
                         "straight. Keep holding, then press Enter:")
        self.power_on_with_help()

    # ---------------- where are we ----------------
    def current_angles(self):
        """All six angles (averaged). Raises if any of servos 1-4 didn't answer."""
        angles = self.read_angles(samples=2)
        silent = silent_servos(angles)
        if silent:
            raise RuntimeError("can't tell where the arm is -- %s didn't answer. %s"
                               % (silent, OUT_OF_RANGE_HINT))
        return angles

    def current_xyz(self):
        a = self.current_angles()
        x, y, z, _ = kinematics.forward(a[0], a[1], a[2], a[3])
        return (x, y, z)

    # ---------------- planning ----------------
    @staticmethod
    def _body_ok(sol, zone):
        """The arm's shape is clear of everything solid (no World: always True)."""
        if zone is None or not hasattr(zone, "margin"):
            return True
        return zone.margin([sol[1], sol[2], sol[3], sol[4]])[0] >= 0.0

    @staticmethod
    def _smooth_solution(point, prev, zone=None):
        """
        Servo angles for `point` that follow on smoothly from `prev` (servos
        1-4): no joint turns more than LINE_MAX_TURN_DEG. Of those, the
        smallest turn (a hand pointing more nearly straight down breaks ties)
        whose shape is clear of everything solid. None if none is smooth.
        """
        found = []
        app = config.APPROACH_STRAIGHT
        while app >= config.APPROACH_MIN - 1e-9:
            sol = kinematics.inverse(point[0], point[1], point[2], approach_deg=app,
                                     prefer=prev)
            if sol is not None and kinematics.within_limits(sol):
                turn = max(abs(sol[k] - prev[k - 1]) for k in range(1, 5))
                if turn <= config.LINE_MAX_TURN_DEG:
                    found.append((turn + 0.02 * (config.APPROACH_STRAIGHT - app), sol))
            app -= 2.0
        for _, sol in sorted(found, key=lambda f: f[0]):
            if Arm._body_ok(sol, zone):
                return sol
        return None

    @staticmethod
    def _follow(a, b, prev, depth=3, zone=None):
        """
        Smooth servo settings from point a (where the arm has angles `prev`)
        to point b: one step if it can, otherwise split in half (up to `depth`
        times) -- close to the base the joints turn a lot per mm. None if no
        smooth way.
        """
        sol = Arm._smooth_solution(b, prev, zone)
        if sol is not None:
            return [sol]
        if depth <= 0:
            return None
        mid = tuple((a[j] + b[j]) / 2.0 for j in range(3))
        first = Arm._follow(a, mid, prev, depth - 1, zone)
        if first is None:
            return None
        second = Arm._follow(mid, b, [first[-1][k] for k in range(1, 5)], depth - 1, zone)
        if second is None:
            return None
        return first + second

    @staticmethod
    def plan_line(start, end, prev=None, swing_first=True, zone=None):
        """
        Straight line in space -> list of servo settings. Every step follows
        on smoothly from the one before (no joint turns more than
        LINE_MAX_TURN_DEG in one step -- no sudden elbow flip halfway), and
        with a World every pose along it keeps the arm's shape clear of the
        electronics, base and table.
        prev = the angles (servos 1-4) the line starts from. If the first step
        can't follow on smoothly from them and swing_first is True, it may be
        a joint swing instead (run_plan simulates that one and runs it in
        sub-steps). Raises OutOfReach if any point can't be reached smoothly.
        """
        dist = _distance(start, end)
        steps = max(1, int(math.ceil(dist / config.STEP_MM)))
        plan = []
        here = tuple(start)
        for i in range(1, steps + 1):
            f = i / float(steps)
            point = tuple(start[j] + (end[j] - start[j]) * f for j in range(3))
            part = Arm._follow(here, point, prev, zone=zone) if prev is not None else None
            if part is None and not plan and (prev is None or swing_first):
                servos, _ = kinematics.solve_reachable(
                    *point, accept=lambda sol: Arm._body_ok(sol, zone))
                part = [servos] if servos is not None else None
            if part is None:
                raise OutOfReach(
                    "path passes through (%.0f, %.0f, %.0f) which can't be reached "
                    "smoothly" % point)
            plan.extend(part)
            prev = [part[-1][k] for k in range(1, 5)]
            here = point
        return plan

    @staticmethod
    def _last(plan):
        return [plan[-1][k] for k in range(1, 5)]

    def run_plan(self, plan, duration_ms=None):
        """
        Runs a straight-line plan, checking every step against the servos'
        read-back. Steps are 10 mm apart, but the FIRST step starts from
        wherever the arm really is -- if that's far from the plan, it's a
        joint swing: simulated first, then done in small sub-steps. (Any
        other big step -- there shouldn't be one -- is treated the same way.)
        """
        if not plan:
            return
        self._ensure_not_blocked()
        prev = self.current_angles()[:4]
        duration_ms = duration_ms or config.STEP_MS
        watch = motion.StallWatch(lag_max=config.STALL_LAG_MAX)
        for servos in plan:
            target = [servos[k] for k in range(1, 5)]
            if max(abs(target[i] - prev[i]) for i in range(4)) > config.CONTINUOUS_DEG:
                ok, where = safety.check_transition(prev, target, self.zone)
                if not ok:
                    raise UnsafeMove("getting onto that path would swing the arm low over "
                                     "the board near (%.0f, %.0f, %.0f)" % where)
                self._run_stages(prev, [dict(servos)])
            else:
                self._move_checked(servos, duration_ms, watch=watch)
            prev = target

    def move_line(self, end, duration_ms=None):
        """Straight line from wherever we are to `end`. Checked before it moves."""
        angles = self.current_angles()
        start = kinematics.forward(*angles[:4])[:3]
        plan = self.plan_line(start, end, prev=angles[:4], zone=self.zone)
        self.last_plan = [self._four(p) for p in plan]
        self.run_plan(plan, duration_ms)

    @staticmethod
    def _four(servos):
        return [round(float(servos[k]), 1) for k in range(1, 5)]

    # ---------------- joint moves (simulated first) ----------------
    def move_to_pose(self, pose, ask_for_help=True):
        """
        Joint move to a full pose (6 angles). robot/safety.py simulates it --
        the whole arm against the board, electronics, base and table -- and if
        moving everything at once isn't safe, finds an order, a way round, or
        (last) searches for a path. If the arm is low over the board, it rises
        straight up first. Nothing safe: asks you to lift it (interactive) or
        raises UnsafeMove.
        """
        self._ensure_not_blocked()
        for servo_id, angle in enumerate(pose, start=1):
            self._check(servo_id, angle)
        problem = pose_problem(list(pose[:4]) + [None, None], self.zone)
        if problem:
            raise UnsafeMove("that pose isn't safe: " + problem)
        while True:
            start = self.current_angles()
            low_over_board = any(self.zone.danger(p) for p in safety.link_points(start))
            if low_over_board and self.lift_clear() > 0:
                start = self.current_angles()
            stages = safety.plan_pose_move(start, pose, self.zone)
            if stages is not None:
                self.last_plan = self._stage_path(start, stages)
                self._run_stages(start, stages)
                return
            if not ask_for_help or not self.interactive:
                raise UnsafeMove("no safe way to move from here -- power off, lift the "
                                 "arm up by hand (wrist straight), then power on")
            self.lift_by_hand()

    @staticmethod
    def _stage_path(start, stages):
        """The poses (servos 1-4) a list of stages passes through, for the 3D view."""
        current = [float(a) for a in start[:4]]
        out = [[round(a, 1) for a in current]]
        for stage in stages:
            for sid, angle in stage.items():
                if sid <= 4:
                    current[sid - 1] = float(angle)
            out.append([round(a, 1) for a in current])
        return out

    def _run_stages(self, start, stages):
        """
        Runs joint-move stages. Smooth (config.SMOOTH_MOTION): all joints of a
        stage move together along a speed profile, a new target every
        CONTROL_DT, each read back. Otherwise: sub-steps of at most
        JOINT_SUBSTEP_DEG, each read back after the servo stops. Either way,
        if anything is in the way the arm stops within a step or two instead
        of pushing for a whole move.
        """
        current = list(start)
        for stage in stages:
            for sid, angle in stage.items():
                self._check(sid, angle)          # every END target is inside the range
            joints = {sid: a for sid, a in stage.items()
                      if sid <= 4 and current[sid - 1] is not None}
            others = {sid: a for sid, a in stage.items() if sid not in joints}
            # start from where the servo really is -- so the path is exactly
            # the one safety.py simulated
            begin = {sid: current[sid - 1] for sid in joints}
            if config.SMOOTH_MOTION and joints:
                self._stream(begin, joints, others)
                for sid, a in stage.items():
                    current[sid - 1] = a
                continue
            biggest = max([abs(joints[s] - begin[s]) for s in joints] or [0.0])
            n = max(1, int(math.ceil(biggest / config.JOINT_SUBSTEP_DEG)))
            step_deg = biggest / n
            ms = max(config.JOINT_MIN_MS, int(step_deg * config.JOINT_MS_PER_DEG))
            for k in range(1, n + 1):
                sub = {s: begin[s] + (joints[s] - begin[s]) * k / float(n) for s in joints}
                if k == n:
                    sub.update(others)
                # in-between values lie between the start and a checked target,
                # so they're only outside the limits on the way back in
                self._move_checked(sub, ms, check=False)
            for sid, a in stage.items():
                current[sid - 1] = a

    def _read_joints_once(self):
        """
        Servos 1-4 for the control loop: one quick retry each, not read_servo's
        three spaced 50 ms apart, which would blow the tick. None for one that
        didn't answer.
        """
        out = [None] * 6
        for sid in (1, 2, 3, 4):
            for attempt in (1, 2):           # one immediate retry: an I2C hiccup is common
                with self.bus_lock:
                    value = self.bus.Arm_serial_servo_read(sid)
                    if value is None:
                        value = self._read_wide(sid)
                if value is not None:
                    out[sid - 1] = float(value)
                    self.last_angles[sid - 1] = out[sid - 1]
                    self.last_read[sid - 1] = out[sid - 1]
                    break
        self.last_seen_t = time.time()
        return out

    def _stream(self, begin, end, others):
        """
        One smooth joint move (see robot/motion.py), run as a fixed-rate
        control loop -- the way Microduck's robotd runs its 50 Hz loop:

          * every CONTROL_DT the target is where the speed profile says the
            joints should be at that moment (time, not tick count), sent with
            a little extra time so the servos keep moving, then read back
          * the schedule is kept: a late tick doesn't push every later one
            back (a slow loop would otherwise run the move in slow motion),
            and a tick that's very late is skipped, not made up for with a
            burst of commands
          * a read that gets no answer doesn't stop the arm by itself: the
            move pauses (the next target is the same) and carries on when the
            servo answers again; READ_MISS_LIMIT misses in a row = hold still
          * a joint that stops following its commands (motion.StallWatch) =
            something is in the way: hold still, ArmBlocked. At the end, every
            joint must settle within SETTLE_DEG of where it was sent.
        """
        dt = config.CONTROL_DT
        biggest = max([abs(end[s] - begin[s]) for s in end] or [0.0])
        total = motion.duration(biggest)
        ms = int(dt * 1000 * 1.5)
        start = time.time()
        paused = 0.0                 # trajectory time skipped while reads failed
        deadline = start
        misses = 0
        ticks = 0
        last_good = None
        watch = motion.StallWatch()
        self._start_watch(watch, end, begin)
        self.health["moves"] += 1
        while True:
            self._ensure_not_blocked()
            if self.stop_event.is_set():
                self._hold_still(self.read_angles(samples=1, ids=(1, 2, 3, 4)))
                raise ArmBlocked("you pressed STOP")
            t = time.time() - start - paused
            f = 1.0 if biggest < 1e-9 or t >= total else motion.position(t, biggest) / biggest
            target = {s: begin[s] + (end[s] - begin[s]) * f for s in end}
            send = dict(target)
            if f >= 1.0:
                send.update(others)
            self.move_servos(send, duration_ms=ms, wait=False, check=False)
            # keep the schedule; skip ticks rather than burst after a slow one
            deadline += dt
            now = time.time()
            if now > deadline + dt:
                self.health["overruns"] += 1
                deadline = now
            time.sleep(max(0.0, deadline - now))
            got = self._read_joints_once()
            ticks += 1
            self.health["ticks"] += 1
            self.log("move", sent=send, ms=ms, got=got[:4])
            silent = [sid for sid in target if got[sid - 1] is None]
            if silent:
                misses += 1
                self.health["read_misses"] += 1
                if misses >= config.READ_MISS_LIMIT:
                    self._hold_still(last_good or got)
                    raise ArmBlocked("servo %d (%s) stopped answering during a move"
                                     % (silent[0], SERVO_NAMES[silent[0]]))
                paused += dt                 # hold this target until it answers again
                continue
            misses = 0
            last_good = got
            for sid in target:
                stalled, behind = watch.update(sid, target[sid], got[sid - 1])
                if stalled:
                    self._hold_still(got)
                    raise ArmBlocked(
                        "servo %d (%s) stopped %.0f degrees short of where it was sent -- "
                        "something is in the way, or the arm wasn't where I thought"
                        % (sid, SERVO_NAMES[sid], behind))
            if f >= 1.0:
                break
        elapsed = time.time() - start
        if ticks and elapsed > 0:
            self.health["hz"] = round(ticks / elapsed, 1)
            self.health["worst_tick_s"] = max(self.health["worst_tick_s"],
                                              round(elapsed / ticks, 3))
        # let it arrive, then make sure it did: within SETTLE_DEG, and either
        # close (SETTLE_CLOSE_DEG) or no longer moving -- so the next move
        # starts from a joint at rest, not one still catching up
        got, prev, close_enough = None, None, False
        for attempt in range(8):
            got = self._read_joints_once()
            known = [sid for sid in end if got[sid - 1] is not None]
            if len(known) == len(end):
                short = max(abs(got[sid - 1] - end[sid]) for sid in end)
                close_enough = short <= config.SETTLE_DEG
                still = prev is not None and all(
                    prev[sid - 1] is not None
                    and abs(got[sid - 1] - prev[sid - 1]) < config.SETTLE_STILL_DEG
                    for sid in end)
                if close_enough and (short <= config.SETTLE_CLOSE_DEG or still):
                    return
            prev = got
            time.sleep(dt)
        if close_enough:
            return
        self._hold_still(got)
        worst = max(end, key=lambda sid: 999.0 if got[sid - 1] is None
                    else abs(got[sid - 1] - end[sid]))
        behind = 999.0 if got[worst - 1] is None else abs(got[worst - 1] - end[worst])
        raise ArmBlocked("servo %d (%s) stopped %.0f degrees short of where it was sent -- "
                         "something is in the way, or the arm wasn't where I thought"
                         % (worst, SERVO_NAMES[worst], behind))

    def ready(self):
        """Joint move to a pose that straight-line planning can start from."""
        self.move_to_pose(config.POSE_READY)

    def go_home(self):
        """Joint move to the home pose: the taught home pose, or resting (POSE_REST)."""
        self.move_to_pose(self.home_pose)

    def start_up(self, confirm=True):
        """
        Run once at the start. Makes sure every reading is trustworthy (else
        you move the arm by hand), says where it thinks the claw is, waits for
        your OK, then goes home in small checked steps.
        """
        if self.start_problem is not None:
            raise self.start_problem
        while True:
            angles = self.read_angles(samples=3)
            silent = silent_servos(angles)
            if silent is None:
                break
            first = [s for s in (1, 2, 3, 4) if angles[s - 1] is None][0]
            print("\n  Can't start from here: %s can't report its angle (bent past the "
                  "end of its scale). Fix: %s." % (silent, FIX_BY_HAND[first]))
            self.lift_by_hand(why="I can't read where the arm is")
        if not self.powered:
            self.stiffen_here()
            angles = self.current_angles()
        x, y, z, _ = kinematics.forward(*angles[:4])
        problem = self.reading_problem(angles)
        if problem:
            raise RuntimeError(problem + ", so I won't move")
        clear = safety.is_clear(angles, self.zone)
        print("  I think the claw is at x=%.0f y=%.0f z=%.0f mm -- %s."
              % (x, y, z, "clear of the board" if clear else "LOW, near the board"))
        body = self.zone.body_problem(angles) if hasattr(self.zone, "body_problem") else None
        if body:
            print("  (%s -- I'll only move it away from there)" % body)
        print("  (Standing straight up, z would be about %.0f. If this doesn't match "
              "what you see, press Ctrl-C.)" % (config.H + config.L1 + config.L2 + config.L3))
        if confirm:
            input("  Enter = go home slowly: ")
        self.log("start_up", got=angles)
        self.go_home()

    def reading_problem(self, angles):
        """
        Why these readings can't be trusted to move from, or None: a servo
        that didn't answer, or angles that would put part of the arm under
        the table (a servo misreporting).
        """
        silent = [s for s in (1, 2, 3, 4) if angles[s - 1] is None]
        if silent:
            return "the %s can't report its angle -- %s" % (SERVO_NAMES[silent[0]],
                                                             FIX_BY_HAND[silent[0]])
        if hasattr(self.zone, "margin"):
            m, link, ob, _ = self.zone.margin(angles)
            if m < -config.IMPOSSIBLE_BELOW_MM and ob in ("electronics", "turret", "base plate"):
                self.log("impossible_pose", got=angles, inside=ob, margin=m)
                return ("the servo readings put the %s %.0f mm inside the %s, which can't be "
                        "true -- a servo is reporting a wrong angle (readings: %s)"
                        % (link, -m, ob, ", ".join("%s %.0f" % (SERVO_NAMES[i + 1], angles[i])
                                                    for i in range(4))))
        lowest = min(p[2] for p in safety.link_points(angles))
        if lowest < self.zone.board_z - config.IMPOSSIBLE_BELOW_MM:
            self.log("impossible_pose", got=angles, lowest=lowest)
            return ("the servo readings put part of the arm %.0f mm under the table, which "
                    "can't be true -- a servo is reporting a wrong angle (readings: %s)"
                    % (self.zone.board_z - lowest,
                       ", ".join("%s %.0f" % (SERVO_NAMES[i + 1], angles[i]) for i in range(4))))
        return None

    def can_plan_from_here(self):
        try:
            servos, _ = kinematics.solve_reachable(*self.current_xyz())
        except RuntimeError:
            return False
        return servos is not None

    def lift_clear(self):
        """
        Rise straight up, up to TRAVEL_MM, starting from however the arm is
        bent right now: each 5 mm step uses the joint solution nearest the
        previous one, so the hand only tilts gradually -- small nudges, never a
        swing. Use before any joint move when the claw might be near pieces.
        Returns how far it rose (0 if it couldn't).
        """
        self._ensure_not_blocked()
        try:
            angles = self.current_angles()
        except RuntimeError:
            return 0.0
        rise = config.TRAVEL_MM
        while rise >= 10.0:
            plan = self._vertical_plan(angles, rise, self.zone)
            if plan:
                try:
                    self.run_plan(plan)
                except UnsafeMove:
                    return 0.0
                return rise
            rise -= 10.0
        return 0.0

    @staticmethod
    def _vertical_plan(angles, rise, zone):
        """Straight up by `rise` mm in LIFT_STEP_MM steps, or None if it can't."""
        x, y, z, approach = kinematics.forward(*angles[:4])
        steps = max(1, int(math.ceil(rise / config.LIFT_STEP_MM)))
        plan, prev = [], list(angles[:4])
        for i in range(1, steps + 1):
            sol, approach = Arm._next_step(
                x, y, z + rise * i / float(steps), prev, approach, zone)
            if sol is None:
                return None
            plan.append(sol)
            prev = [sol[k] for k in range(1, 5)]
        return plan

    @staticmethod
    def _next_step(x, y, z, prev, approach, zone):
        """
        Servo angles for (x, y, z), reached from `prev` by a step that keeps
        the claw on its straight line (safety.step_ok). Tries hand angles
        closest to `approach` first, and within those the smallest turn.
        Returns (angles, hand angle used), or (None, approach) if nothing works.
        """
        for delta in range(0, 62, 2):
            options = (approach,) if delta == 0 else (approach - delta, approach + delta)
            found = []
            for app in options:
                if not 60.0 <= app <= 180.0:
                    continue
                sol = kinematics.inverse(x, y, z, approach_deg=app, prefer=prev)
                if sol is None or not kinematics.within_limits(sol):
                    continue
                change = max(abs(sol[k] - prev[k - 1]) for k in range(1, 5))
                found.append((change, app, sol))
            for change, app, sol in sorted(found, key=lambda f: f[0]):
                if safety.step_ok(prev, [sol[k] for k in range(1, 5)], zone):
                    return sol, app
        return None, approach

    def goto(self, x, y, z):
        """
        Safe travel: rise to a clearance height, move across, then descend.

        The clearance height is chosen by trying the highest first and working
        down -- higher is safer for the pieces, but the arm can't reach as far
        out when it's high, so the route has to be reachable at BOTH ends and
        everywhere in between. The whole route is solved before the arm moves.
        """
        if not self.can_plan_from_here():
            self.ready()
        try:
            return self._goto(x, y, z)
        except (UnsafeMove, OutOfReach):
            # No smooth safe route from here (e.g. from an unusual home pose,
            # or straight across close to the base). From the ready pose --
            # high, in the middle -- there usually is one.
            if self._near(config.POSE_READY):
                raise
            self.ready()
            return self._goto(x, y, z)

    def _near(self, pose, tol=3.0):
        try:
            angles = self.current_angles()
        except RuntimeError:
            return False
        return max(abs(angles[i] - pose[i]) for i in range(4)) <= tol

    def _goto(self, x, y, z):
        angles = self.current_angles()
        start = kinematics.forward(*angles[:4])[:3]
        target = (x, y, z)

        highest = z + config.TRAVEL_MM
        lowest = z          # worst case: cross at the target's own height
        height = highest
        while height >= lowest - 0.01:
            try:
                up = (start[0], start[1], height)
                across = (x, y, height)
                rise = self.plan_line(start, up, prev=angles[:4], zone=self.zone)
                over = self.plan_line(up, across, prev=self._last(rise), swing_first=False,
                                      zone=self.zone)
                down = self.plan_line(across, target, prev=self._last(over),
                                      swing_first=False, zone=self.zone)
                plan = rise + over + down
            except OutOfReach:
                height -= 10.0
                continue
            self.last_plan = [self._four(angles)] + [self._four(p) for p in plan]
            self.run_plan(plan)
            return height
        raise OutOfReach(
            "no safe route to (%.0f, %.0f, %.0f) -- the arm can't clear the "
            "board and still reach that far" % (x, y, z))

    def park(self):
        """Lift clear of the board, then fold back to rest (POSE_REST) -- if that's safe."""
        if self.blocked:
            return
        try:
            self.lift_clear()
        except ArmBlocked:
            raise
        except Exception:
            pass
        self.move_to_pose(config.POSE_REST, ask_for_help=False)

    # ---------------- gripper ----------------
    def open_claw(self):
        self.set_servo(6, config.CLAW_OPEN, 500)
        time.sleep(0.6)

    def close_claw(self):
        """Closes until it meets resistance. True = something is held."""
        angle = config.CLAW_OPEN
        while angle < config.CLAW_MAX:
            angle += config.CLAW_STEP
            self.set_servo(6, angle, 120)
            time.sleep(0.03)
            feedback = self.read_servo(6)
            if feedback is not None and abs(angle - feedback) > config.CLAW_STALL_GAP:
                return True
        return False

    # ---------------- pick and place ----------------
    def move_piece(self, from_xyz, to_xyz):
        fx, fy, fz = from_xyz
        tx, ty, tz = to_xyz
        hover = config.HOVER_MM

        self.open_claw()
        self.goto(fx, fy, fz + hover)
        self.move_line((fx, fy, fz))
        gripped = self.close_claw()
        self.move_line((fx, fy, fz + hover))
        self.goto(tx, ty, tz + hover)
        self.move_line((tx, ty, tz))
        self.open_claw()
        self.move_line((tx, ty, tz + hover))
        return gripped
