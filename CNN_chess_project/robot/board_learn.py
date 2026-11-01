#!/usr/bin/env python3
"""
Learning the board from your touches -- all of them, from every session.

Each time you rest the claw tip on a pawn and hold still, that's one touch:
(square, x y z) from forward kinematics of the servo readings. Touches are
noisy -- a few mm, from where on the pawn top the claw rested and a degree
of servo reading -- so instead of asking you to redo squares until they
agree (v2.6 and before), every touch is used:

  * every touch is kept (calibration/touches.json), across sessions: teach a
    square again and it's averaged in, not swapped for the new one
  * the board -- a rigid grid that can only slide, turn, scale and tilt
    (robot/board_frame.py) -- is fitted to all of them by robust least
    squares: Huber weights, so a touch that disagrees a lot counts for less,
    and one more than OUTLIER_MM off (the claw on the wrong square) is left
    out by itself
  * if a new session's touches don't fit the stored board at all (the board
    was moved), it starts again from the new ones

The fitted squares get better with every touch: random errors shrink about
like 1 / sqrt(number of touches). Pure Python, runs anywhere.
"""
import json
import math
import os
import time

import config
from robot.board_frame import BoardFrame, square_to_indexes

HUBER_MM = 4.0       # touches closer than this to the board count fully
OUTLIER_MM = 15.0    # further than this = the wrong spot: left out
MOVED_MM = 20.0      # a new session this far (typical) from the stored board = it moved
GOOD_MM = 5.0        # typical touch error for "good"
OK_MM = 9.0          # ...for "OK"; above = "rough: teach a few more"


def _distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _median(values):
    v = sorted(values)
    n = len(v)
    if not n:
        return 0.0
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


# ---------------- the touches file ----------------
def load_touches(path=None):
    path = path or config.TOUCHES_FILE
    if not os.path.exists(path):
        return []
    try:
        with open(path) as fh:
            return json.load(fh).get("touches", [])
    except (OSError, ValueError):
        return []


def save_touches(touches, path=None):
    path = path or config.TOUCHES_FILE
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w") as fh:
        json.dump({"touches": touches}, fh, indent=1)


def make_touch(square, point, session=None):
    """One touch record from teach._record's point (xyz, servos, tilt, photo)."""
    return {"square": square, "xyz": [round(float(c), 2) for c in point["xyz"]],
            "servos": [None if a is None else round(float(a), 2) for a in point.get("servos", [])],
            "tilt": round(float(point.get("tilt", 0.0)), 1), "photo": point.get("photo"),
            "session": session or time.strftime("%Y%m%d-%H%M%S")}


# ---------------- fitting ----------------
def _squares(touches, weights):
    return {t["square"] for t, w in zip(touches, weights) if w > 0}


def _fits(touches, weights):
    """Enough different squares, not all in one line, to fit a board?"""
    squares = list(_squares(touches, weights))
    if len(squares) < 3:
        return False
    pts = [square_to_indexes(s) for s in squares]
    (f0, r0) = pts[0]
    for (f1, r1) in pts[1:]:
        for (f2, r2) in pts[1:]:
            if (f1 - f0) * (r2 - r0) - (r1 - r0) * (f2 - f0) != 0:
                return True
    return False


def fit(touches):
    """
    Robust board fit. Returns (frame, weights, distances): weights 0..1 per
    touch (0 = left out), distances = mm from each touch to the fitted board.
    Raises ValueError if they can't make a board (fewer than 3 squares, or
    all in a line).
    """
    if not touches:
        raise ValueError("no touches")
    weights = [1.0] * len(touches)
    if not _fits(touches, weights):
        raise ValueError("need 3 or more different squares, not all in one line")
    obs = [(t["square"], t["xyz"]) for t in touches]
    frame = BoardFrame.fit_weighted(obs, weights)
    for _ in range(30):
        dist = [_distance(frame.square_xyz(sq), xyz) for sq, xyz in obs]
        new = [0.0 if d > OUTLIER_MM else (1.0 if d <= HUBER_MM else HUBER_MM / d) for d in dist]
        if not _fits(touches, new):
            # leaving them out would leave too little: keep every touch, softly weighted
            new = [1.0 if d <= HUBER_MM else HUBER_MM / d for d in dist]
        if max(abs(a - b) for a, b in zip(new, weights)) < 1e-4:
            break
        weights = new
        frame = BoardFrame.fit_weighted(obs, weights)
    dist = [_distance(frame.square_xyz(sq), xyz) for sq, xyz in obs]
    return frame, weights, dist


def summary(touches, weights, dist, new_from=0):
    """
    What the fit says, in numbers: used / left out / typical error /
    quality. Only touches from new_from on (this session's) are listed as
    left out: an old one already said so when it was taught.
    """
    used = [(t, d) for t, w, d in zip(touches, weights, dist) if w > 0]
    left = [(t["square"], d) for i, (t, w, d) in enumerate(zip(touches, weights, dist))
            if w == 0 and i >= new_from]
    rms = math.sqrt(sum(d * d for _, d in used) / len(used)) if used else 0.0
    quality = "good" if rms <= GOOD_MM else ("ok" if rms <= OK_MM else "rough")
    return {"touches": len(used), "squares": len({t["square"] for t, _ in used}),
            "rms_mm": round(rms, 1), "left_out": left, "quality": quality}


def update(new_touches, path=None):
    """
    Adds a session's touches to the stored ones and learns the board from
    all of them. Returns (frame or None, report). Saves the touches and,
    when there's a board, the board file too.
    """
    old = load_touches(path)
    moved = False
    if old and new_touches:
        try:
            frame_old, _, _ = fit(old)
            typical = _median([_distance(frame_old.square_xyz(t["square"]), t["xyz"])
                               for t in new_touches])
            if typical > MOVED_MM:
                moved, old = True, []
        except ValueError:
            pass
    touches = old + list(new_touches)
    save_touches(touches, path)
    report = {"moved": moved, "new": len(new_touches), "all": len(touches)}
    try:
        frame, weights, dist = fit(touches)
    except ValueError as e:
        report["problem"] = str(e)
        report["squares"] = len({t["square"] for t in touches})
        return None, report
    report.update(summary(touches, weights, dist, new_from=len(old)))
    return frame, report


def describe(frame, report):
    """A few short lines for the screen."""
    lines = []
    if report.get("moved"):
        lines.append("The board seems to have moved -- starting fresh from these touches.")
    if frame is None:
        lines.append("Not enough yet: %s (have %d square%s). Teach a few more."
                     % (report.get("problem", "need more squares"), report.get("squares", 0),
                        "" if report.get("squares", 0) == 1 else "s"))
        return lines
    lines.append("Board learned from %d touches on %d squares (squares %.1f mm)."
                 % (report["touches"], report["squares"], frame.square_size_mm()[0]))
    verdict = {"good": "good", "ok": "OK -- a few more squares will sharpen it",
               "rough": "rough -- teach 2-3 more squares (menu 1) to sharpen it"}
    lines.append("Touches agree within about %.0f mm: %s." % (max(1.0, report["rms_mm"]),
                                                             verdict[report["quality"]]))
    if report["left_out"]:
        lines.append("Left out (wrong spot?): %s. Teach it again any time." % ", ".join(
            "%s %.0f mm off" % (sq.upper(), d) for sq, d in report["left_out"]))
    return lines
