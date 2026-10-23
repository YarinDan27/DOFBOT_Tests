#!/usr/bin/env python3
"""
Talks to the servos. The only file that does.

Three safety rules, all enforced here (carried over from old_chess_project):
  1. every angle is checked against config.SERVO_LIMITS before anything moves
  2. moves follow a planned straight line in space, cut into small steps,
     instead of letting the joints swing wherever they like
  3. the WHOLE path is solved before the first step runs -- if any point on it
     is unreachable, nothing moves at all

New here: limp() / stiffen_here() for teaching by hand, averaged servo
readings, and lift_clear() so the arm never swings sideways near the board.
"""
import math
import time

import config
from robot import kinematics


class OutOfReach(Exception):
    pass


class UnsafeAngle(Exception):
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
        self.bus.Arm_serial_set_torque(1)
        self.is_limp = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.is_limp:
                print("\n  The arm is still limp. HOLD IT, then press Enter.")
                input()
                self.stiffen_here()
            self.park()
        except Exception:
            pass
        return False

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

    def stiffen_here(self):
        """
        Servo power back on WITHOUT a jump. A servo switched back on drives to
        its last target, which could be far from where your hand left it -- so
        first make every target "where you already are", then switch on.
        """
        angles = self.read_angles(samples=3)
        if any(a is None for a in angles[:5]):
            raise RuntimeError("couldn't read every servo -- keep holding the arm "
                               "and try again")
        for servo_id, angle in enumerate(angles, start=1):
            if angle is None:          # claw didn't answer: leave it alone
                continue
            self.bus.Arm_serial_servo_write(servo_id, int(round(_clamp(servo_id, angle))), 800)
        time.sleep(0.1)
        self.bus.Arm_serial_set_torque(1)
        time.sleep(0.9)
        self.is_limp = False

    # ---------------- where are we ----------------
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
        duration_ms = duration_ms or config.STEP_MS
        for servos in plan:
            self.move_servos(servos, duration_ms=duration_ms)

    def move_line(self, end, duration_ms=None):
        """Straight line from wherever we are to `end`. Checked before it moves."""
        plan = self.plan_line(self.current_xyz(), end)
        self.run_plan(plan, duration_ms)

    def ready(self):
        """Move (in joint space) to a pose that straight-line planning can start from."""
        pose = {i + 1: v for i, v in enumerate(config.POSE_READY)}
        self.move_servos(pose, duration_ms=1200)

    def can_plan_from_here(self):
        try:
            servos, _ = kinematics.solve_reachable(*self.current_xyz())
        except RuntimeError:
            return False
        return servos is not None

    def lift_clear(self):
        """
        Rise straight up, as high as the arm can manage (up to TRAVEL_MM).
        Use this before any joint-space move when the claw might be near
        pieces -- a joint-space swing is what drove the arm into the board.
        Returns how far it rose (0 if it couldn't plan from here).
        """
        if not self.can_plan_from_here():
            return 0.0
        start = self.current_xyz()
        rise = config.TRAVEL_MM
        while rise >= 10.0:
            try:
                plan = self.plan_line(start, (start[0], start[1], start[2] + rise))
            except OutOfReach:
                rise -= 10.0
                continue
            self.run_plan(plan)
            return rise
        return 0.0

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
        """Lift clear of the board, then stand up straight."""
        try:
            self.lift_clear()
        except Exception:
            pass
        pose = {i + 1: v for i, v in enumerate(config.POSE_REST)}
        self.move_servos(pose, duration_ms=1200)

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
