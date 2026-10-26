#!/usr/bin/env python3
"""
Measure the electronics box -- hands-free at the arm.

The arm's safety model needs to know where the black electronics box (Jetson
+ driver board) is. config.ELECTRONICS is a guess from the product photo;
this replaces it with the real thing.

At the arm: after the long beep (power off), touch the CLOSED claw tip on
the top of the electronics box at three corners, in this order, holding
still 2 s at each (beeps = how many are done):

    1. front-left   (nearest the arm, on your left as you stand behind it)
    2. front-right  (nearest the arm, on your right)
    3. back         (the far end, either side)

Long beep = done, power back on. Saved to calibration/electronics.json; the
3D view shows the new box.
"""
import config
from robot import world
from sessions import teach

CORNERS = ["front-left corner", "front-right corner", "back corner"]


def box_from(points):
    """Box ranges from the three touched corners (claw tip on its top)."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return {"x": (min(xs), max(xs)), "y": (min(ys), max(ys)), "z": (0.0, max(zs))}


def run(arm):
    print("\n--- Measure the electronics box (hands-free at the arm) ---")
    print("At the arm:")
    print("  * wait for the LONG beep -- be holding the arm by then (power goes off)")
    print("  * touch the CLOSED claw tip on the TOP of the black electronics box, at:")
    for i, name in enumerate(CORNERS, start=1):
        print("      %d. the %s" % (i, name))
    print("    holding still 2 s at each (beeps = how many are done)")
    print("  * long beep at the end = power is back on, you can let go")
    teach.countdown(arm, config.TEACH_COUNTDOWN_S)
    taught, how = teach.collect(arm, CORNERS)
    teach._finish_at_arm(arm, how)
    if len(taught) < len(CORNERS):
        print("  Only %d of %d corners -- nothing saved." % (len(taught), len(CORNERS)))
        return None
    ranges = box_from([taught[c]["xyz"] for c in CORNERS])
    if ranges["y"][1] > -20.0 or ranges["z"][1] < 10.0:
        print("  Those touches don't look like a box behind the arm (y %.0f..%.0f, top %.0f) "
              "-- nothing saved." % (ranges["y"][0], ranges["y"][1], ranges["z"][1]))
        return None
    world.save_electronics(ranges)
    arm.zone.obstacles = world.solid_obstacles()
    print("  Saved: electronics box x %.0f..%.0f, y %.0f..%.0f, top %.0f mm."
          % (ranges["x"][0], ranges["x"][1], ranges["y"][0], ranges["y"][1], ranges["z"][1]))
    return ranges
