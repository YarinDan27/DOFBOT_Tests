#!/usr/bin/env python3
"""
The arm's shape: where every part of it is, for a set of servo angles.

kinematics.py only knows where the claw tip ends up. To know whether the arm
hits something on the way, we need the whole arm, so here it is modelled as
five solid parts -- capsules (a line with a radius, like a sausage):

    upper arm   shoulder -> elbow            (servo 2 turns it)
    forearm     elbow -> wrist               (servo 3)
    wrist       wrist -> 45% of the hand     (servo 4 + the wrist-roll servo:
                                              the thick part)
    gripper     the rest, out to the tip     (the claw fingers: thin)
    camera      the depth camera bar on the back of the wrist, across the arm

Same angle conventions as kinematics.py: all servos at 90 = straight up;
servo 1 up = turns left; servos 2-4 up = lean away from the board.
Pure math, runs anywhere.
"""
import math

import config


def joint_points(angles):
    """
    {name: (x, y, z)} for 'shoulder', 'elbow', 'wrist', 'hand' (end of the
    thick wrist part), 'tip' (where the claw fingers close) and the two ends
    of the camera bar, 'camera_a' and 'camera_b'.
    """
    s1, s2, s3, s4 = [float(a) for a in angles[:4]]
    phi = math.radians(s1 - 90.0)
    a1 = math.radians(90.0 - s2)                  # each link's angle from vertical
    a2 = a1 + math.radians(90.0 - s3)
    a3 = a2 + math.radians(90.0 - s4)

    def place(r, z):
        return (-r * math.sin(phi), r * math.cos(phi), z)

    r, z = 0.0, config.H
    out = {"shoulder": place(r, z)}
    r += config.L1 * math.sin(a1)
    z += config.L1 * math.cos(a1)
    out["elbow"] = place(r, z)
    r += config.L2 * math.sin(a2)
    z += config.L2 * math.cos(a2)
    out["wrist"] = place(r, z)
    hr = r + config.L3 * config.WRIST_PART * math.sin(a3)
    hz = z + config.L3 * config.WRIST_PART * math.cos(a3)
    out["hand"] = place(hr, hz)
    # camera bar: on the back of the hand, across the arm
    cr = r + config.CAMERA_ALONG_MM * math.sin(a3) - config.CAMERA_UP_MM * math.cos(a3)
    cz = z + config.CAMERA_ALONG_MM * math.cos(a3) + config.CAMERA_UP_MM * math.sin(a3)
    centre = place(cr, cz)
    across = (math.cos(phi), math.sin(phi), 0.0)     # along the joint axes
    w = config.CAMERA_HALF_WIDTH_MM
    out["camera_a"] = tuple(centre[i] - across[i] * w for i in range(3))
    out["camera_b"] = tuple(centre[i] + across[i] * w for i in range(3))
    r += config.L3 * math.sin(a3)
    z += config.L3 * math.cos(a3)
    out["tip"] = place(r, z)
    return out


LINKS = (("upper arm", "shoulder", "elbow"),
         ("forearm", "elbow", "wrist"),
         ("wrist", "wrist", "hand"),
         ("gripper", "hand", "tip"),
         ("camera", "camera_a", "camera_b"))


def capsules(angles):
    """[(link name, start point, end point, radius mm)] for every part."""
    pts = joint_points(angles)
    return [(name, pts[a], pts[b], config.LINK_RADIUS[name]) for name, a, b in LINKS]


def sample_points(angles, spacing=6.0):
    """
    [(link name, point, radius)]: points every `spacing` mm along each link's
    centre line, ends included. A capsule is all the space within `radius`
    of its centre line, so checking these points (radius + spacing/2) covers it.
    """
    out = []
    for name, a, b, radius in capsules(angles):
        length = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        n = max(1, int(math.ceil(length / spacing)))
        for k in range(n + 1):
            f = k / float(n)
            out.append((name, tuple(a[i] + (b[i] - a[i]) * f for i in range(3)), radius))
    return out
