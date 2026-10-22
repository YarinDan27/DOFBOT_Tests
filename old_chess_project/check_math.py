#!/usr/bin/env python3
"""Checks the math without any hardware. Run this on the laptop."""
import math
import kinematics as k
import chessboard

print(f"--- round trip: pick a target, solve it, confirm the arm lands there ---")
for target in [(0, 150, -60), (-50, 160, -60), (50, 140, -60), (0, 200, -40)]:
    servos, app = k.solve_reachable(*target)
    if servos is None:
        print(f"  {target} -> OUT OF REACH")
        continue
    lands = k.forward(servos[1], servos[2], servos[3], servos[4])[:3]
    err = max(abs(lands[i] - target[i]) for i in range(3))
    print(f"  {target} -> servos " +
          " ".join(f"{servos[i]:6.1f}" for i in range(1, 5)) +
          f" | tilt {app:5.1f} | error {err:.4f}mm")

print("\n--- chessboard math: invent a chessboard, see if it reconstructs all 64 squares ---")
SQ, ANGLE, Z = 22.0, 12.0, -55.0
ORIGIN = (-70.0, 95.0, Z)
c, s = math.cos(math.radians(ANGLE)), math.sin(math.radians(ANGLE))


def truth(sq):
    f, r = chessboard.square_to_indexes(sq)
    dx, dy = f * SQ, r * SQ
    return (ORIGIN[0] + dx * c - dy * s, ORIGIN[1] + dx * s + dy * c, Z)


bf = chessboard.BoardFrame(truth("a1"), truth("h1"), truth("a8"))
worst = max(max(abs(bf.square_xyz(f"{f}{r}")[i] - truth(f"{f}{r}")[i]) for i in range(3))
            for f in chessboard.FILES for r in range(1, 9))
print(f"  worst error over all 64 squares: {worst:.6f} mm")
print(f"  square size it worked out: {bf.square_size_mm()[0]:.2f} mm (real {SQ})")
