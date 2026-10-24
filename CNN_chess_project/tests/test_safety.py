#!/usr/bin/env python3
"""
Checks the joint-move collision check with no hardware attached:

    python3 tests/test_safety.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                   # noqa: E402
from robot import kinematics, safety            # noqa: E402
from robot.board_frame import BoardFrame        # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


def pose_at(x, y, z):
    s, _ = kinematics.solve_reachable(x, y, z)
    return [s[1], s[2], s[3], s[4], 90.0, config.CLAW_OPEN]


REST = list(config.POSE_REST)
READY = list(config.POSE_READY)
zone = safety.Zone.default()

print("--- the normal moves are allowed ---")
check("standing up is clear of the board", safety.is_clear(REST, zone))
check("the ready pose is clear of the board", safety.is_clear(READY, zone))
check("standing up -> ready, all at once, is safe",
      safety.check_transition(REST, READY, zone)[0])
check("planner moves everything at once when that's safe",
      len(safety.plan_pose_move(REST, READY, zone)) == 1)

print("\n--- the move that knocked the board over is refused ---")
low = pose_at(0, 180, config.DEFAULT_BOARD_Z)          # claw down on the board
check("claw on the board is NOT clear", not safety.is_clear(low, zone))
ok, where = safety.check_transition(low, READY, zone)
check("swinging from there straight to ready is refused", not ok,
      "would hit near (%.0f, %.0f, %.0f)" % where if where else "")
lying = [90, 30, 30, 60, 90, config.CLAW_OPEN]        # arm lying forward over the board
check("arm lying over the board: no safe joint move to ready",
      safety.plan_pose_move(lying, READY, zone) is None)

print("\n--- rising straight up first makes it safe ---")
lifted = pose_at(0, 180, config.DEFAULT_BOARD_Z + config.TRAVEL_MM)
check("80 mm up, the claw is clear", safety.is_clear(lifted, zone))
check("from there, ready is reachable safely",
      safety.plan_pose_move(lifted, READY, zone) is not None)

print("\n--- rising straight up by itself from a claw resting on the board ---")
from robot.arm import Arm                                      # noqa: E402
stretched = pose_at(0, 180, config.DEFAULT_BOARD_Z)
lift = Arm._vertical_plan(stretched, config.TRAVEL_MM, zone)
check("it finds a way up 80 mm", lift is not None,
      "%d steps" % len(lift) if lift else "none")
if lift:
    top = kinematics.forward(*[lift[-1][i] for i in range(1, 5)])[:3]
    tip0 = kinematics.forward(*stretched[:4])[:3]
    drift = ((top[0] - tip0[0]) ** 2 + (top[1] - tip0[1]) ** 2) ** 0.5
    check("it ends up 80 mm higher, in the same spot", abs(top[2] - tip0[2] - 80) < 0.5
          and drift < 0.5, "drift %.2f mm" % drift)
    prev = stretched[:4]
    all_ok = True
    for step in lift:
        nxt = [step[i] for i in range(1, 5)]
        all_ok = all_ok and safety.step_ok(prev, nxt, zone)
        prev = nxt
    check("every step keeps the claw on its line", all_ok)
    check("and once up there, it can go home",
          safety.plan_pose_move(prev + [90, config.CLAW_OPEN], READY, zone) is not None)

print("\n--- a small nudge near a piece is still allowed ---")
near = pose_at(0, 180, config.DEFAULT_BOARD_Z + 20)
nudge = pose_at(0, 180, config.DEFAULT_BOARD_Z + 28)
check("8 mm straight up from a piece is allowed",
      safety.check_transition(near, nudge, zone)[0])

print("\n--- the zone follows a taught board ---")
frame = BoardFrame.fit({"a1": (-77.0, 126.0, -13.0), "h1": (77.0, 126.0, -13.0),
                        "a6": (-77.0, 236.0, -13.0)})
board_zone = safety.Zone.from_board(frame)
check("centre of the board is inside", board_zone.over_board(0, 180))
check("12 mm past the a1 corner is inside (margin)",
      board_zone.over_board(-77 - 11 - 12, 126 - 11 - 12))
check("behind the base is outside", not board_zone.over_board(0, -60))
check("far to the side is outside", not board_zone.over_board(250, 180))

print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("All checks passed.")
