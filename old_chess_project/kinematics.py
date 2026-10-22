#!/usr/bin/env python3
"""
Pure math. No hardware, no imports from the robot library.
Runs anywhere, including your laptop.

Frame (standing behind the arm, looking at the board):
    X = right, Y = forward toward the board, Z = up
    Origin = center of the base, on the table surface.

Servo convention (verified on the real arm):
    all servos at 90  -> arm points straight up
    servo 1 increasing -> turns LEFT
    servos 2,3,4 increasing -> lean AWAY from the board
So "tilt toward the board" = 90 - servo_value.
"""
import math
import config

H = config.H
L1 = config.L1
L2 = config.L2
L3 = config.L3          # wrist segment plus the gripper's own length

MAX_REACH_2LINK = L1 + L2


def servo_to_tilt(servo_value):
    """Servo number -> tilt angle in degrees, positive = toward the board."""
    return 90.0 - servo_value


def tilt_to_servo(tilt_deg):
    return 90.0 - tilt_deg


def forward(servo1, servo2, servo3, servo4):
    """Where is the gripper? Returns (x, y, z) in mm, plus the approach angle."""
    phi = math.radians(servo1 - 90.0)          # base rotation, + = left
    t2 = math.radians(servo_to_tilt(servo2))
    t3 = math.radians(servo_to_tilt(servo3))
    t4 = math.radians(servo_to_tilt(servo4))

    a1 = t2            # angle of link 1 from vertical
    a2 = t2 + t3       # link 2
    a3 = t2 + t3 + t4  # link 3 (the hand)

    r = L1 * math.sin(a1) + L2 * math.sin(a2) + L3 * math.sin(a3)
    z = H + L1 * math.cos(a1) + L2 * math.cos(a2) + L3 * math.cos(a3)

    x = -r * math.sin(phi)
    y = r * math.cos(phi)
    return x, y, z, math.degrees(a3)


def within_limits(servos):
    for sid, value in servos.items():
        lo, hi = config.SERVO_LIMITS[sid]
        if not (lo <= value <= hi):
            return False
    return True


def solve_reachable(x, y, z):
    """Tries straight down first, tilts the hand only if it has to."""
    app = config.APPROACH_STRAIGHT
    while app >= config.APPROACH_MIN:
        servos = inverse(x, y, z, approach_deg=app)
        if servos is not None and within_limits(servos):
            return servos, app
        app -= 2.0
    return None, None


def inverse(x, y, z, approach_deg=180.0):
    """
    Target point -> servo values.
    approach_deg is the hand's angle from vertical: 180 = pointing straight down.
    Returns dict with servo1..4, or None if that target can't be reached.
    """
    r = math.hypot(x, y)
    if r < 1e-6:
        return None

    # base rotation: forward is +Y, left is -X
    phi = math.atan2(-x, y)
    servo1 = 90.0 + math.degrees(phi)

    # step back along the hand to find where the wrist must be
    a3 = math.radians(approach_deg)
    wrist_r = r - L3 * math.sin(a3)
    wrist_z = z - L3 * math.cos(a3)

    dr = wrist_r
    dz = wrist_z - H
    d = math.hypot(dr, dz)
    if d > MAX_REACH_2LINK or d < abs(L1 - L2) + 1e-9:
        return None

    beta = math.atan2(dr, dz)  # angle of shoulder->wrist line, from vertical
    cos_alpha = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    best = None
    for sign in (+1.0, -1.0):
        t2 = beta - sign * alpha
        ex = L1 * math.sin(t2)
        ez = H + L1 * math.cos(t2)
        psi = math.atan2(wrist_r - ex, wrist_z - ez)
        t3 = psi - t2
        t4 = a3 - t2 - t3

        servos = {
            1: servo1,
            2: tilt_to_servo(math.degrees(t2)),
            3: tilt_to_servo(math.degrees(t3)),
            4: tilt_to_servo(math.degrees(t4)),
        }
        if all(0.0 <= v <= 180.0 for v in servos.values()):
            # prefer the elbow-forward solution (keeps it away from the electronics)
            score = servos[2]
            if best is None or score < best[0]:
                best = (score, servos)
    return best[1] if best else None
