#!/usr/bin/env python3
"""
Checks the math with no hardware attached. Runs on the laptop or the Jetson:

    python3 tests/test_geometry.py
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                                   # noqa: E402
from robot import kinematics                                    # noqa: E402
from robot.arm import Arm                                       # noqa: E402
from robot.board_frame import BoardFrame, all_squares, square_to_indexes  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


def fake_board(square_mm=22.0, angle_deg=8.0, origin=(-75.0, 120.0, -13.0)):
    """A made-up board: returns square -> true (x, y, z)."""
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    truth = {}
    for sq in all_squares():
        f, r = square_to_indexes(sq)
        dx, dy = f * square_mm, r * square_mm
        truth[sq] = (origin[0] + dx * c - dy * s, origin[1] + dx * s + dy * c, origin[2])
    return truth


print("--- kinematics: solve a target, then check the arm would land on it ---")
worst = 0.0
for target in [(0, 150, -13), (-50, 160, 30), (50, 140, -13), (0, 230, -13)]:
    servos, _ = kinematics.solve_reachable(*target)
    if servos is None:
        check("reach %s" % (target,), False, "out of reach")
        continue
    lands = kinematics.forward(servos[1], servos[2], servos[3], servos[4])[:3]
    worst = max(worst, max(abs(lands[i] - target[i]) for i in range(3)))
check("forward(inverse(target)) == target", worst < 0.01, "worst %.4f mm" % worst)

print("\n--- board fit: rebuild all 64 squares from a few taught ones ---")
truth = fake_board()
taught = {sq: truth[sq] for sq in ["b2", "g2", "b6", "g6", "e4"]}
frame = BoardFrame.fit(taught)
err = max(max(abs(frame.square_xyz(sq)[i] - truth[sq][i]) for i in range(3))
          for sq in all_squares())
check("5 perfect touches rebuild the board", err < 1e-6, "worst %.2e mm" % err)
check("square size recovered", abs(frame.square_size_mm()[0] - 22.0) < 1e-6,
      "%.3f mm" % frame.square_size_mm()[0])
check("corner angle is 90", abs(frame.corner_angle_deg() - 90.0) < 1e-6)

exact = BoardFrame.fit({sq: truth[sq] for sq in ["a1", "h1", "a6"]})
check("3 perfect touches also rebuild it",
      max(abs(exact.square_xyz("h8")[i] - truth["h8"][i]) for i in range(3)) < 1e-6)

print("\n--- board fit: does it catch a sloppy touch? ---")
random.seed(1)
sloppy = {}
for sq in ["b2", "g2", "b6", "g6", "e4", "d6"]:
    x, y, z = truth[sq]
    sloppy[sq] = (x + random.uniform(-1, 1), y + random.uniform(-1, 1), z)
x, y, z = sloppy["g6"]
sloppy["g6"] = (x + 9.0, y, z)          # one touch 9 mm off
res = BoardFrame.fit(sloppy).residuals()
flagged = sorted(sq for sq, r in res.items() if r > config.TEACH_MAX_RESIDUAL_MM)
check("only the 9 mm-off touch gets flagged", flagged == ["g6"],
      ", ".join("%s %.1f" % (k, res[k]) for k in sorted(res)))

try:
    BoardFrame.fit({sq: truth[sq] for sq in ["a1", "b2", "c3", "d4"]})
    check("squares in a straight line are refused", False)
except ValueError:
    check("squares in a straight line are refused", True)

print("\n--- reach map on the old 25.6 mm board (expect rank 8 out of reach) ---")
old = BoardFrame.fit({"a1": (-82.13, 117.29, -13.0),
                      "h1": (96.70, 111.24, -13.0),
                      "a8": (-82.13 + 6.04, 117.29 + 178.83, -13.0)})
reach = old.reach_map()
check("rank 8 is out of reach", all(reach[f + "8"] != "ok" for f in "abcdefgh"))
check("rank 4 is reachable", all(reach[f + "4"] == "ok" for f in "abcdefgh"))

print("\n--- planner: a straight line checks every step ---")
plan = Arm.plan_line((0, 150, 40), (40, 180, 40))
check("plan_line cuts the move into 10 mm steps", len(plan) == 5, "%d steps" % len(plan))

print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("All checks passed.")
