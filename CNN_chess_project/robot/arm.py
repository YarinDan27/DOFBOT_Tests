#!/usr/bin/env python3
"""
Talks to the servos. The only file that does.

Safety rules, all enforced here:
  1. every angle is checked against config.SERVO_LIMITS before anything moves
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
import time

import config
from robot import kinematics, safety


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


def outside_limits(angles):
    """[(servo, angle, low, high)] for servos 1-4 past config.SERVO_LIMITS (+ slack)."""
    out = []
    for sid in (1, 2, 3, 4):
        a = angles[sid - 1]
        if a is None:
            continue
        low, high = config.SERVO_LIMITS[sid]
        if a < low - config.LIMIT_SLACK_DEG or a > high + config.LIMIT_SLACK_DEG:
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
    """
    Text if a servo is past the range the ROBOT may move in (SERVO_LIMITS), or
    None. Fine for measuring (teaching); not fine to power on or move from.
    """
    bad = outside_limits(angles)
    if not bad:
        return None
    sid, a, low, high = bad[0]
    return ("servo %d (%s) is at %.0f, past the robot's safe range %.0f-%.0f: %s"
            % (sid, SERVO_NAMES[sid], a, low, high, _which_way(sid, a < low)))


def pose_problem(angles):
    """
    Why these readings can't be trusted to power on or move from, or None:
    a servo that didn't answer, or one past its safe range. Includes what to do.
    """
    silent = silent_servos(angles)
    if silent:
        return silent + " didn't answer. " + OUT_OF_RANGE_HINT
    return limit_problem(angles)


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
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        if "arm.launch" in cmd or "dofbot_driver" in cmd or "arm_driver" in cmd:
            found.append((pid, "Yahboom's ROS arm driver (%s)" % cmd[:70], False))
        elif "python" in cmd and "run.py" in cmd:
            try:
                cwd = os.path.realpath(os.readlink("/proc/%d/cwd" % pid))
            except OSError:
                cwd = ""
            if cwd == root:
                found.append((pid, "another copy of this program (%s)" % cmd[:70], True))
    return found


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _clamp(servo_id, angle):
    low, high = config.SERVO_LIMITS[servo_id]
    return max(low, min(high, angle))


class Arm:
    def __init__(self):
        # Imported here, not at the top, so the planning code can still be
        # imported and tested on the laptop, where Arm_Lib doesn't exist.
        from Arm_Lib import Arm_Device
        self.bus = Arm_Device()
        time.sleep(0.1)
        self.is_limp = False
        self.powered = False        # True once stiffen_here() has switched on safely
        self.blocked = False
        self.skip_park = False      # set when you quit with Ctrl-C: no moving on the way out
        self.zone = safety.Zone.default()
        self.home_pose = list(config.POSE_READY)
        self._log_file = None
        self._open_log()
        # Power on aimed at where it already is -- only if servos 1-4 answer.
        # Otherwise nothing changes; start_up() deals with it.
        self.start_problem = None
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
            print("\n  Not parking (you pressed Ctrl-C). The arm is holding still.")
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
        """Taught board -> the danger zone is that board. None -> a big default area."""
        self.zone = safety.Zone.from_board(frame) if frame is not None else safety.Zone.default()

    # ---------------- raw servo access ----------------
    def _check(self, servo_id, angle):
        low, high = config.SERVO_LIMITS[servo_id]
        if not (low <= angle <= high):
            raise UnsafeAngle(
                "servo %d angle %.1f outside safe range %s-%s" % (servo_id, angle, low, high))

    def set_servo(self, servo_id, angle, duration_ms=400):
        self._check(servo_id, angle)
        self.bus.Arm_serial_servo_write(servo_id, int(round(angle)), int(duration_ms))

    def read_servo(self, servo_id):
        """
        One servo's angle, or None. (Arm_Lib returns None when the angle is
        outside 0-180, e.g. a joint pushed past its range. There is NO fallback:
        v0.2.1 tried Arm_serial_servo_read_any, and a bad reading from it is the
        likely cause of the 2026-10-24 crash.)
        """
        for _ in range(3):
            value = self.bus.Arm_serial_servo_read(servo_id)
            if value is not None:
                return float(value)
            time.sleep(0.05)
        return None

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
        return [totals[i] / counts[i] if counts[i] else None for i in range(6)]

    def move_servos(self, angles, duration_ms=400, wait=True, check=True):
        """
        Raw: sends angles, no read-back. Use _move_checked for servos 1-4.
        check=False only for sub-steps on the way back INTO the safe range
        from a pose past it (the end target itself is always checked).
        """
        if check:
            for servo_id, angle in angles.items():
                self._check(servo_id, angle)
        for servo_id, angle in angles.items():
            self.bus.Arm_serial_servo_write(servo_id, int(round(angle)), int(duration_ms))
        if wait:
            time.sleep(duration_ms / 1000.0 + 0.05)

    def _move_checked(self, angles, duration_ms, check=True):
        """
        Sends angles, waits, then reads servos 1-4 back. Any of them more than
        STALL_DEG short of where it was sent = blocked: hold still right there
        and raise ArmBlocked. Refuses to move at all while blocked.
        """
        self._ensure_not_blocked()
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
            short = abs(got[sid - 1] - angles[sid])
            if short > config.STALL_DEG:
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
                self.bus.Arm_serial_servo_write(sid, int(round(got[sid - 1])), 300)
        self.log("blocked", got=got[:4])

    def clear_block(self):
        """You checked the arm and the way is clear: allow moves again."""
        self.blocked = False
        self.log("unblocked")

    # ---------------- teaching by hand ----------------
    def limp(self):
        """Servo power off: the arm goes floppy. HOLD IT before calling this."""
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
            problem = silent + " didn't answer. " + OUT_OF_RANGE_HINT
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
            self.bus.Arm_serial_servo_write(servo_id, int(round(angle)), 800)
        time.sleep(0.1)
        self.bus.Arm_serial_set_torque(1)
        time.sleep(0.9)
        self.is_limp = False
        self.powered = True
        self.log("stiffen", got=angles)
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

    def power_on_with_help(self):
        """
        Asks you to hold the arm, then powers on without a jump. If a servo
        can't be read or is out of range, you can keep trying or leave the
        power off (StillLimp).
        """
        print("\nHOLD THE ARM -- servo power is coming back on.")
        input("Press Enter when you're holding it: ")
        while True:
            try:
                self.stiffen_here()
                return
            except RuntimeError as e:
                print("  Can't switch on yet: %s" % e)
                ans = input("  Move it, keep holding, then: Enter = try again   "
                            "q = leave the power off and quit: ")
                if ans.strip().lower() == "q":
                    raise StillLimp()

    def lift_by_hand(self, why="the arm is down near the board and every way out "
                              "would sweep over it"):
        """You move the arm by hand (power off), then it powers on there."""
        print("\n  I can't move by myself: %s." % why)
        if not self.is_limp:
            input("  HOLD THE ARM and press Enter -- its power goes off: ")
            self.limp()
        input("  Move it well above the board, roughly upright, keep holding, "
              "and press Enter: ")
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
    def plan_line(start, end):
        """Straight line in space -> list of servo settings. Raises if any point fails."""
        dist = _distance(start, end)
        steps = max(1, int(math.ceil(dist / config.STEP_MM)))
        plan = []
        for i in range(1, steps + 1):
            f = i / float(steps)
            point = tuple(start[j] + (end[j] - start[j]) * f for j in range(3))
            servos, _ = kinematics.solve_reachable(*point)
            if servos is None:
                raise OutOfReach(
                    "path passes through (%.0f, %.0f, %.0f) which can't be reached"
                    % point)
            plan.append(servos)
        return plan

    def run_plan(self, plan, duration_ms=None):
        """
        Runs a straight-line plan, checking every step against the servos'
        read-back. Its later steps are 10 mm apart, but the FIRST step starts
        from wherever the arm really is -- if that's far from the plan, it's a
        joint swing: simulated first, then done in small sub-steps.
        """
        if not plan:
            return
        self._ensure_not_blocked()
        start = self.current_angles()
        first = [plan[0][i] for i in range(1, 5)]
        duration_ms = duration_ms or config.STEP_MS
        if max(abs(first[i] - start[i]) for i in range(4)) > config.CONTINUOUS_DEG:
            ok, where = safety.check_transition(start[:4], first, self.zone)
            if not ok:
                raise UnsafeMove("getting onto that path would swing the arm low over "
                                 "the board near (%.0f, %.0f, %.0f)" % where)
            self._run_stages(start, [dict(plan[0])])
            plan = plan[1:]
        for servos in plan:
            self._move_checked(servos, duration_ms)

    def move_line(self, end, duration_ms=None):
        """Straight line from wherever we are to `end`. Checked before it moves."""
        plan = self.plan_line(self.current_xyz(), end)
        self.run_plan(plan, duration_ms)

    # ---------------- joint moves (simulated first) ----------------
    def move_to_pose(self, pose, ask_for_help=True):
        """
        Joint move to a full pose (6 angles). robot/safety.py simulates it and,
        if moving everything at once would sweep over the board, finds an order
        that doesn't. If the arm is low over the board, it rises straight up
        first. Last resort: asks you to lift it (or raises UnsafeMove).
        """
        self._ensure_not_blocked()
        for servo_id, angle in enumerate(pose, start=1):
            self._check(servo_id, angle)
        while True:
            start = self.current_angles()
            if not safety.is_clear(start, self.zone) and self.lift_clear() > 0:
                start = self.current_angles()
            stages = safety.plan_pose_move(start, pose, self.zone)
            if stages is not None:
                self._run_stages(start, stages)
                return
            if not ask_for_help:
                raise UnsafeMove("no safe way to move from here")
            self.lift_by_hand()

    def _run_stages(self, start, stages):
        """
        Runs joint-move stages in sub-steps of at most JOINT_SUBSTEP_DEG, each
        checked against the read-back -- so if anything is in the way, the arm
        stops within a couple of sub-steps instead of pushing for a whole move.
        """
        current = list(start)
        for stage in stages:
            for sid, angle in stage.items():
                self._check(sid, angle)          # every END target is inside the limits
            joints = {sid: a for sid, a in stage.items()
                      if sid <= 4 and current[sid - 1] is not None}
            others = {sid: a for sid, a in stage.items() if sid not in joints}
            # start from where the servo really is -- even past a limit -- so the
            # path is exactly the one safety.py simulated
            begin = {sid: current[sid - 1] for sid in joints}
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

    def ready(self):
        """Joint move to a pose that straight-line planning can start from."""
        self.move_to_pose(config.POSE_READY)

    def go_home(self):
        """Joint move to the home pose: the taught look-at-board pose, or ready."""
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
            print("\n  Can't start from here: %s didn't answer. %s" % (silent, OUT_OF_RANGE_HINT))
            self.lift_by_hand(why="I can't read where the arm is")
        if not self.powered:
            self.stiffen_here()
            angles = self.current_angles()
        x, y, z, _ = kinematics.forward(*angles[:4])
        lowest = min(p[2] for p in safety.link_points(angles))
        if lowest < self.zone.board_z - config.IMPOSSIBLE_BELOW_MM:
            self.log("impossible_pose", got=angles, lowest=lowest)
            raise RuntimeError(
                "the servo readings put part of the arm %.0f mm under the table, which "
                "can't be true -- a servo is reporting a wrong angle, so I won't move. "
                "Readings: %s" % (self.zone.board_z - lowest,
                                  ", ".join("%s %.0f" % (SERVO_NAMES[i + 1], angles[i])
                                            for i in range(4))))
        clear = safety.is_clear(angles, self.zone)
        print("  I think the claw is at x=%.0f y=%.0f z=%.0f mm -- %s."
              % (x, y, z, "clear of the board" if clear else "LOW, near the board"))
        for sid, a, low, high in outside_limits(angles):
            print("  (%s is at %.0f, past the %.0f-%.0f it moves in -- I'll bring it "
                  "back in small checked steps)" % (SERVO_NAMES[sid], a, low, high))
        print("  (Standing straight up, z would be about %.0f. If this doesn't match "
              "what you see, press Ctrl-C.)" % (config.H + config.L1 + config.L2 + config.L3))
        if confirm:
            input("  Enter = go home slowly: ")
        self.log("start_up", got=angles)
        self.go_home()

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
        except UnsafeMove:
            # Getting onto the path from here would swing low (e.g. from an
            # unusual home pose). From the ready pose that jump is high up.
            self.ready()
            return self._goto(x, y, z)

    def _goto(self, x, y, z):
        start = self.current_xyz()
        target = (x, y, z)

        highest = z + config.TRAVEL_MM
        lowest = z          # worst case: cross at the target's own height
        height = highest
        while height >= lowest - 0.01:
            try:
                up = (start[0], start[1], height)
                across = (x, y, height)
                plan = (self.plan_line(start, up)
                        + self.plan_line(up, across)
                        + self.plan_line(across, target))
            except OutOfReach:
                height -= 10.0
                continue
            self.run_plan(plan)
            return height
        raise OutOfReach(
            "no safe route to (%.0f, %.0f, %.0f) -- the arm can't clear the "
            "board and still reach that far" % (x, y, z))

    def park(self):
        """Lift clear of the board, then stand up straight -- if that's safe."""
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
