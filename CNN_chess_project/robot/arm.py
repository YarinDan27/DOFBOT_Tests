#!/usr/bin/env python3
"""
Talks to the servos. The only file that does.

Safety rules, all enforced here:
  1. every angle is checked against config.SERVO_LIMITS before anything moves
  2. moves near the board follow a planned straight line in space, cut into
     small steps, instead of letting the joints swing wherever they like
  3. the WHOLE path is solved before the first step runs -- if any point on it
     is unreachable, nothing moves at all
  4. (new) every joint swing -- going home, parking, the jump at the start of
     a straight line -- is simulated by robot/safety.py first, and refused if
     any part of the arm would sweep over the board low
  5. (new) power comes on aimed at where the arm already is: no jump

If it's down near the board, it rises straight up before anything else. If
it can't find a safe way at all, it asks you to lift it by hand.
"""
import math
import time

import config
from robot import kinematics, safety


class OutOfReach(Exception):
    pass


class UnsafeAngle(Exception):
    pass


class UnsafeMove(Exception):
    pass


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
        self.zone = safety.Zone.default()
        self.home_pose = list(config.POSE_READY)
        # Power on aimed at where it already is. If the servos can't be read,
        # this switches on anyway (as the old code did) -- but nothing moves
        # until start_up() has checked where the arm is.
        self.stiffen_here(required=False)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.is_limp:
                print("\n  The arm is still limp. HOLD IT, then press Enter.")
                input()
                self.stiffen_here()
            self.park()
        except Exception as e:
            print("  Didn't park (%s). The arm is holding where it is." % e)
        return False

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
        for _ in range(3):
            value = self.bus.Arm_serial_servo_read(servo_id)
            if value is not None:
                return float(value)
            time.sleep(0.05)
        return None

    def read_all(self):
        return [self.read_servo(i) for i in range(1, 7)]

    def read_angles(self, samples=1):
        """
        All six servo angles, each averaged over `samples` readings.
        Averaging matters when teaching: a single reading can be off by a
        degree, and one degree at the shoulder is ~3mm at the claw.
        A servo that never answered comes back as None.
        """
        totals = [0.0] * 6
        counts = [0] * 6
        for _ in range(samples):
            for i, value in enumerate(self.read_all()):
                if value is not None:
                    totals[i] += value
                    counts[i] += 1
            time.sleep(0.02)
        return [totals[i] / counts[i] if counts[i] else None for i in range(6)]

    def move_servos(self, angles, duration_ms=400, wait=True):
        for servo_id, angle in angles.items():
            self._check(servo_id, angle)
        for servo_id, angle in angles.items():
            self.bus.Arm_serial_servo_write(servo_id, int(round(angle)), int(duration_ms))
        if wait:
            time.sleep(duration_ms / 1000.0 + 0.05)

    # ---------------- teaching by hand ----------------
    def limp(self):
        """Servo power off: the arm goes floppy. HOLD IT before calling this."""
        self.bus.Arm_serial_set_torque(0)
        self.is_limp = True

    def stiffen_here(self, required=True):
        """
        Servo power on WITHOUT a jump. A servo switched on drives to its last
        target, which could be far from where the arm is now -- so first make
        every target "where you already are", then switch on.
        """
        angles = self.read_angles(samples=3)
        if any(a is None for a in angles[:5]):
            if required:
                raise RuntimeError("couldn't read every servo -- keep holding the arm "
                                   "and try again")
            self.bus.Arm_serial_set_torque(1)
            self.is_limp = False
            return
        for servo_id, angle in enumerate(angles, start=1):
            if angle is None:          # claw didn't answer: leave it alone
                continue
            self.bus.Arm_serial_servo_write(servo_id, int(round(_clamp(servo_id, angle))), 800)
        time.sleep(0.1)
        self.bus.Arm_serial_set_torque(1)
        time.sleep(0.9)
        self.is_limp = False

    def power_on_with_help(self):
        """Asks you to hold the arm, then powers on without a jump."""
        print("\nHOLD THE ARM -- servo power is coming back on.")
        input("Press Enter when you're holding it: ")
        while True:
            try:
                self.stiffen_here()
                return
            except RuntimeError as e:
                input("  %s. Press Enter to retry: " % e)

    def lift_by_hand(self):
        """No safe way to move by itself: you lift it, then it powers on there."""
        print("\n  The arm is down near the board and every way out would sweep over it.")
        input("  HOLD THE ARM and press Enter -- its power goes off: ")
        self.limp()
        input("  Lift it well above the board (roughly upright), keep holding, "
              "and press Enter: ")
        self.power_on_with_help()

    # ---------------- where are we ----------------
    def current_angles(self):
        """All six angles (averaged). Raises if servos 1-4 can't be read."""
        angles = self.read_angles(samples=2)
        if any(a is None for a in angles[:4]):
            raise RuntimeError("a servo didn't answer -- can't tell where the arm is")
        return angles

    def current_xyz(self):
        s = self.read_all()
        if any(v is None for v in s[:4]):
            raise RuntimeError("a servo didn't answer -- can't tell where the arm is")
        x, y, z, _ = kinematics.forward(s[0], s[1], s[2], s[3])
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
        Runs a straight-line plan. Its later steps are 10 mm apart, but the
        FIRST step starts from wherever the arm really is -- if that's far
        from the plan, it's a joint swing, so it gets simulated first.
        """
        if not plan:
            return
        start = self.current_angles()
        first = [plan[0][i] for i in range(1, 5)]
        if max(abs(first[i] - start[i]) for i in range(4)) > config.CONTINUOUS_DEG:
            ok, where = safety.check_transition(start[:4], first, self.zone)
            if not ok:
                raise UnsafeMove("getting onto that path would swing the arm low over "
                                 "the board near (%.0f, %.0f, %.0f)" % where)
        duration_ms = duration_ms or config.STEP_MS
        for servos in plan:
            self.move_servos(servos, duration_ms=duration_ms)

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
        current = list(start)
        for stage in stages:
            turns = [abs(a - current[sid - 1]) for sid, a in stage.items()
                     if current[sid - 1] is not None]
            ms = max(config.JOINT_MIN_MS, int(max(turns or [0]) * config.JOINT_MS_PER_DEG))
            self.move_servos(stage, duration_ms=ms)
            for sid, a in stage.items():
                current[sid - 1] = a

    def ready(self):
        """Joint move to a pose that straight-line planning can start from."""
        self.move_to_pose(config.POSE_READY)

    def go_home(self):
        """Joint move to the home pose: the taught look-at-board pose, or ready."""
        self.move_to_pose(self.home_pose)

    def start_up(self):
        """Run once at the start: wherever the arm is, get it home safely."""
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
        bent right now: each 10 mm step uses the joint solution nearest the
        previous one, so the hand only tilts gradually -- small nudges, never a
        swing. Use before any joint move when the claw might be near pieces.
        Returns how far it rose (0 if it couldn't).
        """
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
        try:
            self.lift_clear()
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
