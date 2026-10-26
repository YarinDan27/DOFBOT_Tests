#!/usr/bin/env python3
"""
Checks joint-space moves before they happen. Pure math, runs anywhere.

The arm moves in two ways:
  * straight lines (arm.plan_line): every 10 mm step solved before moving.
    Used on purpose to go down to pieces.
  * joint moves: each servo turns from its current angle to a target, all at
    the same time. The claw's path is then a curve nobody chose -- that's what
    knocked the board over, twice. So every joint move is simulated here
    first, every SIM_STEP_DEG degrees, and refused if:
      - any part of the arm passes through the board's danger zone
        (lower than CLEAR_MM over the board -- the pieces), or
      - with a World (robot/world.py): any link, with its thickness, comes
        closer than its clearance to the table, base, turret or electronics.

A point that is already in the board zone may still move a little
(NUDGE_MM): letting go of a piece is fine, sweeping across the board is not.
A pose already too close to something solid may only move away from it.

plan_pose_move() finds a safe way between two poses: everything at once, one
servo at a time in every order, via standing upright or the ready pose, and
as a last resort a search (RRT-connect) through the space of joint angles.
"""
import itertools
import math
import random
import time

import config

FILES_SPAN = 8  # squares per side


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def link_points(angles):
    """
    Points along the arm for servo angles [s1, s2, s3, s4, ...]: four along each
    of the three links (shoulder->elbow, elbow->wrist, wrist->claw tip).
    Same geometry as kinematics.forward(); the last point is the claw tip.
    """
    s1, s2, s3, s4 = angles[:4]
    phi = math.radians(s1 - 90.0)
    t2 = math.radians(90.0 - s2)
    t3 = math.radians(90.0 - s3)
    t4 = math.radians(90.0 - s4)
    pts = []
    r0, z0 = 0.0, config.H
    for length, a in ((config.L1, t2), (config.L2, t2 + t3), (config.L3, t2 + t3 + t4)):
        for k in (1, 2, 3, 4):
            f = k / 4.0
            pts.append((r0 + length * f * math.sin(a), z0 + length * f * math.cos(a)))
        r0 += length * math.sin(a)
        z0 += length * math.cos(a)
    return [(-r * math.sin(phi), r * math.cos(phi), z) for r, z in pts]


class Zone:
    """Where the arm must not sweep through."""

    def __init__(self, polygon, board_z):
        self.polygon = [tuple(p) for p in polygon]   # board outline from above, x/y
        self.board_z = float(board_z)

    @classmethod
    def default(cls):
        """Board not taught yet: treat a big area in front of the arm as the board."""
        x0, x1, y0, y1 = config.DEFAULT_ZONE
        return cls([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], config.DEFAULT_BOARD_Z)

    @classmethod
    def from_board(cls, frame):
        """The taught board's outline, grown by ZONE_MARGIN_MM on every side."""
        size = frame.square_size_mm()[0]
        m = 0.5 + config.ZONE_MARGIN_MM / size     # in squares, from the corner centres
        lo, hi = -m, (FILES_SPAN - 1) + m
        polygon = []
        for f, r in ((lo, lo), (hi, lo), (hi, hi), (lo, hi)):
            polygon.append(tuple(frame.origin[i] + frame.file_step[i] * f
                                 + frame.rank_step[i] * r for i in range(2)))
        corner_z = [frame.origin[2] + frame.file_step[2] * f + frame.rank_step[2] * r
                    for f in (0, 7) for r in (0, 7)]
        return cls(polygon, max(corner_z))

    def over_board(self, x, y):
        """Inside the outline? (The outline is convex: same side of every edge.)"""
        sign = 0
        n = len(self.polygon)
        for i in range(n):
            ax, ay = self.polygon[i]
            bx, by = self.polygon[(i + 1) % n]
            cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
            if abs(cross) < 1e-9:
                continue
            s = 1 if cross > 0 else -1
            if sign == 0:
                sign = s
            elif s != sign:
                return False
        return True

    def danger(self, point):
        x, y, z = point
        if z < self.board_z + config.TABLE_CLEAR_MM:
            return True
        return z < self.board_z + config.CLEAR_MM and self.over_board(x, y)


def _margin(zone, angles):
    """Room to the solid obstacles (mm, negative = too close), or None for a plain Zone."""
    fn = getattr(zone, "margin", None)
    return fn(angles)[0] if fn else None


def is_clear(angles, zone):
    """True if no part of the arm is in the danger zone (or too close to anything solid)."""
    if any(zone.danger(p) for p in link_points(angles)):
        return False
    m = _margin(zone, angles)
    return m is None or m >= 0.0


def check_transition(start, end, zone, samples=None):
    """
    Simulates a joint move from `start` to `end` (servos 1-4), all servos
    turning together. Returns (True, None) if it's safe, or (False, point)
    with the first point where some part of the arm would sweep through the
    board zone or come too close to something solid.
    """
    start = [float(a) for a in start[:4]]
    end = [float(a) for a in end[:4]]
    biggest = max(abs(end[j] - start[j]) for j in range(4))
    samples = max(samples or config.SIM_SAMPLES, int(math.ceil(biggest / config.SIM_STEP_DEG)))
    start_pts = link_points(start)
    m0 = _margin(zone, start)
    for i in range(1, samples + 1):
        f = i / float(samples)
        angles = [start[j] + (end[j] - start[j]) * f for j in range(4)]
        for p0, p in zip(start_pts, link_points(angles)):
            if zone.danger(p) and _distance(p, p0) > config.NUDGE_MM:
                return False, p
        if m0 is not None:
            m, link, ob, point = zone.margin(angles)
            # too close is only allowed while getting further away
            if m < 0.0 and m < min(m0, 0.0) - 0.5:
                return False, point
    return True, None


def _distance_to_segment(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    length2 = sum(c * c for c in ab)
    t = 0.0 if length2 < 1e-9 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / length2))
    return _distance(p, [a[i] + ab[i] * t for i in range(3)])


def step_ok(start, end, zone, samples=None):
    """
    One small step of a straight-line move, done as a joint move: the claw
    tip must stay within LINE_TOL_MM of the straight line between its start
    and end, and nothing low over the board may sweep. Joints may turn a fair
    bit (a stretched arm needs to) as long as the claw goes where it should.
    """
    samples = samples or config.SIM_SAMPLES
    tip0 = link_points(start)[-1]
    tip1 = link_points(end)[-1]
    for i in range(1, samples):
        f = i / float(samples)
        angles = [start[j] + (end[j] - start[j]) * f for j in range(4)]
        if _distance_to_segment(link_points(angles)[-1], tip0, tip1) > config.LINE_TOL_MM:
            return False
    return check_transition(start, end, zone, samples)[0]


def _stages_safe(start4, stages, zone):
    current = list(start4)
    for stage in stages:
        nxt = current[:]
        for sid, angle in stage.items():
            nxt[sid - 1] = angle
        if not check_transition(current, nxt, zone)[0]:
            return False
        current = nxt
    return True


def _find_stages(start4, target4, zone):
    """
    First safe way from start4 to target4: all servos together, or else one
    servo at a time in every possible order. None if nothing is safe.
    """
    moving = [sid for sid in (1, 2, 3, 4) if abs(target4[sid - 1] - start4[sid - 1]) > 0.5]
    if not moving:
        return []
    candidates = [[{sid: target4[sid - 1] for sid in moving}]]
    for order in itertools.permutations(moving):
        candidates.append([{sid: target4[sid - 1]} for sid in order])
    for stages in candidates:
        if _stages_safe(start4, stages, zone):
            return stages
    return None


def _in_range(angles):
    for sid in (1, 2, 3, 4):
        low, high = config.JOINT_RANGE[sid]
        if not low <= angles[sid - 1] <= high:
            return False
    return True


def _pose_free(angles, zone):
    return _in_range(angles) and is_clear(angles, zone)


def _edge_free(a, b, zone):
    return check_transition(a, b, zone)[0]


def search_path(start4, target4, zone, seed=1, time_limit=None, step=12.0):
    """
    RRT-connect in the four joint angles: grows a tree of safe poses from each
    end until they meet. Returns [start, ..., target] (lists of 4 angles) with
    every hop checked by check_transition, shortened where it can be, or None.
    Deterministic for a given seed. The start pose may be inside the board
    zone (e.g. holding a piece): its first hop only has to be safe to leave by.
    """
    time_limit = config.SEARCH_TIME_S if time_limit is None else time_limit
    rng = random.Random(seed)
    deadline = time.time() + time_limit
    start4 = [float(a) for a in start4]
    target4 = [float(a) for a in target4]
    if _edge_free(start4, target4, zone):
        return [start4, target4]
    trees = [([start4], [None]), ([target4], [None])]

    def nearest(nodes, q):
        return min(range(len(nodes)),
                   key=lambda i: sum((nodes[i][j] - q[j]) ** 2 for j in range(4)))

    def steer(a, b):
        d = math.sqrt(sum((b[j] - a[j]) ** 2 for j in range(4)))
        if d <= step:
            return list(b)
        return [a[j] + (b[j] - a[j]) * step / d for j in range(4)]

    def extend(tree, q):
        nodes, parents = tree
        i = nearest(nodes, q)
        new = steer(nodes[i], q)
        if not _pose_free(new, zone) or not _edge_free(nodes[i], new, zone):
            return None
        nodes.append(new)
        parents.append(i)
        return len(nodes) - 1

    def path_to_root(tree, i):
        nodes, parents = tree
        out = []
        while i is not None:
            out.append(nodes[i])
            i = parents[i]
        return out

    a, b = 0, 1
    while time.time() < deadline:
        if rng.random() < 0.1:
            q = list(trees[b][0][0])
        else:
            q = [rng.uniform(*config.JOINT_RANGE[sid]) for sid in (1, 2, 3, 4)]
        i = extend(trees[a], q)
        if i is not None:
            q_new = trees[a][0][i]
            # pull the other tree toward the new pose until stuck or joined
            while True:
                j = extend(trees[b], q_new)
                if j is None:
                    break
                if sum((trees[b][0][j][k] - q_new[k]) ** 2 for k in range(4)) < 1e-6:
                    first = path_to_root(trees[a], i)
                    second = path_to_root(trees[b], j)
                    if a == 0:
                        path = list(reversed(first)) + second[1:]
                    else:
                        path = list(reversed(second)) + first[1:]
                    return shorten(path, zone)
        a, b = b, a
    return None


def shorten(path, zone):
    """Drops in-between poses wherever a direct move is just as safe."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _edge_free(path[i], path[j], zone):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _waypoints(start4, target4):
    """Poses worth going through when a direct move isn't safe."""
    base = start4[0]
    return [
        [base, config.CENTER, config.CENTER, config.CENTER],          # standing up
        [base] + [float(a) for a in config.POSE_READY[1:4]],          # high over the board
        [target4[0], config.CENTER, config.CENTER, config.CENTER],
        [base, config.CENTER, start4[2], start4[3]],                  # shoulder up first
    ]


def plan_pose_move(start, target, zone, search=True):
    """
    A safe joint move from the current angles `start` to the pose `target`
    (6 servo angles). Returns a list of stages (each a dict servo -> angle,
    run one after another), or None if every option is unsafe.

    Tries: everything at once; one servo at a time in every order; the same
    via standing upright / the ready pose / shoulder-first; and (search=True)
    a search through joint space.
    """
    start4 = [float(a) for a in start[:4]]
    target4 = [float(a) for a in target[:4]]

    stages = _find_stages(start4, target4, zone)
    if stages is None:
        for mid in _waypoints(start4, target4):
            if not _pose_free(mid, zone):
                continue
            first = _find_stages(start4, mid, zone)
            second = _find_stages(mid, target4, zone) if first is not None else None
            if first is not None and second is not None:
                stages = first + second
                break
    if stages is None and search:
        path = search_path(start4, target4, zone)
        if path is not None:
            stages = [{sid: p[sid - 1] for sid in (1, 2, 3, 4)} for p in path[1:]]
    if stages is None:
        return None

    # wrist roll and claw: with the last stage (or on their own)
    extra = {sid: float(target[sid - 1]) for sid in (5, 6)
             if len(target) >= sid and target[sid - 1] is not None}
    if extra:
        if stages:
            last = dict(stages[-1])
            last.update(extra)
            stages[-1] = last
        else:
            stages = [extra]
    return stages
