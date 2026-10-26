#!/usr/bin/env python3
"""
How a joint move unfolds in time. Pure math, runs anywhere.

v2.4 and before moved in start-stop sub-steps: send 5 degrees, wait for the
servo to arrive and stop, read it back, repeat -- safe, but jerky, and slow.

Now (config.SMOOTH_MOTION) a move follows a speed profile: it speeds up
gently, cruises at JOINT_SPEED_DEG_S, and slows down before the end. Every
CONTROL_DT seconds the next target is sent with a little more time than the
tick, so the servo never quite arrives -- it keeps moving -- and each target
is still read back: a joint that falls behind by more than STALL_DEG plus
one step means something is in the way, and the arm holds still.

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
