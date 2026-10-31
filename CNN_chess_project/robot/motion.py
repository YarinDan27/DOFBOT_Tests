#!/usr/bin/env python3
"""
How a joint move unfolds in time. Pure math, runs anywhere.

v2.4 and before moved in start-stop sub-steps: send 5 degrees, wait for the
servo to arrive and stop, read it back, repeat -- safe, but jerky, and slow.

Now (config.SMOOTH_MOTION) a move follows a speed profile: it speeds up
gently, cruises at JOINT_SPEED_DEG_S, and slows down before the end. Every
CONTROL_DT seconds the next target is sent with a little more time than the
tick, so the servo never quite arrives -- it keeps moving -- and each target
is still read back: a joint that stops following its commands (StallWatch,
below) means something is in the way, and the arm holds still.

All joints in one move start and finish together: each covers its own
distance in the same time, so the arm follows exactly the straight line in
joint space that robot/safety.py simulated.
"""
import math

import config


def duration(distance, speed=None, accel=None):
    """Seconds to cover `distance` degrees with a trapezoid speed profile."""
    speed = speed or config.JOINT_SPEED_DEG_S
    accel = accel or config.JOINT_ACCEL_DEG_S2
    distance = abs(float(distance))
    if distance < 1e-9:
        return 0.0
    if distance < speed * speed / accel:          # never reaches full speed
        return 2.0 * math.sqrt(distance / accel)
    return distance / speed + speed / accel


def position(t, distance, speed=None, accel=None):
    """Degrees covered after t seconds (0 .. distance) on that profile."""
    speed = speed or config.JOINT_SPEED_DEG_S
    accel = accel or config.JOINT_ACCEL_DEG_S2
    distance = abs(float(distance))
    total = duration(distance, speed, accel)
    if t <= 0.0:
        return 0.0
    if t >= total:
        return distance
    if distance < speed * speed / accel:
        peak = total / 2.0
        if t <= peak:
            return 0.5 * accel * t * t
        left = total - t
        return distance - 0.5 * accel * left * left
    ramp = speed / accel
    if t <= ramp:
        return 0.5 * accel * t * t
    if t >= total - ramp:
        left = total - t
        return distance - 0.5 * accel * left * left
    return 0.5 * accel * ramp * ramp + speed * (t - ramp)


def fractions(distance, dt=None, speed=None, accel=None):
    """
    How far along the move (0..1) at each tick: dt, 2dt, ... ending exactly
    at 1.0. A tiny move is a single tick.
    """
    dt = dt or config.CONTROL_DT
    distance = abs(float(distance))
    total = duration(distance, speed, accel)
    if distance < 1e-9 or total <= dt:
        return [1.0]
    n = int(math.ceil(total / dt))
    return [position(k * total / n, distance, speed, accel) / distance for k in range(1, n + 1)]


def joint_targets(begin, end, dt=None, speed=None, accel=None):
    """
    [{servo: angle}] for each tick of a joint move from `begin` to `end`
    (dicts servo -> angle). The joint that moves furthest sets the timing;
    the others scale to finish together.
    """
    biggest = max([abs(end[s] - begin[s]) for s in end] or [0.0])
    return [{s: begin[s] + (end[s] - begin[s]) * f for s in end}
            for f in fractions(biggest, dt, speed, accel)]


class StallWatch(object):
    """
    Is a joint still following its commands? A real collision stops the
    joint while the commands keep going; a slow or sagging servo lags but
    keeps moving. So a stall is called when, over the last WINDOW reads, the
    command moved at least MIN_CMD degrees and the joint moved less than
    FOLLOW of that, while it's more than STALL_DEG behind -- or when it's
    more than LAG_MAX behind whatever it's doing. "Behind" doesn't count the
    few degrees a loaded joint already sat off its command when the move
    began (start()). v2.5 used only "more than STALL_DEG + one step behind",
    which a slow, sagging servo can reach without touching anything: the
    randomised servo tests found it.
    """

    WINDOW = 2
    MIN_CMD = 3.0
    FOLLOW = 0.25

    def __init__(self, stall_deg=None, lag_max=None):
        self.stall_deg = config.STALL_DEG if stall_deg is None else stall_deg
        self.lag_max = config.STALL_LAG_MAX if lag_max is None else lag_max
        self.hist = {}
        self.offset = {}

    def start(self, sid, offset):
        """
        The joint sat `offset` degrees off its command when the move began
        (sag and gear play under its own weight). Only lag beyond that counts
        -- up to STALL_DEG of it: a joint further off than that wasn't where
        it was thought to be, and the rest still counts.
        """
        self.offset[sid] = max(-self.stall_deg, min(self.stall_deg, float(offset)))

    def update(self, sid, target, got):
        """Returns (stalled, degrees behind)."""
        behind = abs(got - target - self.offset.get(sid, 0.0))
        h = self.hist.setdefault(sid, [])
        h.append((float(target), float(got)))
        del h[:-(self.WINDOW + 1)]
        if behind > self.lag_max:
            return True, behind
        if len(h) > self.WINDOW and behind > self.stall_deg:
            dc = h[-1][0] - h[0][0]
            dg = h[-1][1] - h[0][1]
            if abs(dc) >= self.MIN_CMD and dg * (1.0 if dc > 0 else -1.0) < self.FOLLOW * abs(dc):
                return True, behind
        return False, behind
