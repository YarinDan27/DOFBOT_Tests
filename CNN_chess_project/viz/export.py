#!/usr/bin/env python3
"""
Writes the 3D view as one file you can open anywhere, no arm needed:

    python3 -m viz.export                 -> arm_view.html
    python3 -m viz.export my_view.html

It shows your saved calibration (board, home pose, measured electronics, the
latest self-test colours) and lets you dial in any pose with sliders and see
whether the arm's shape would hit something.
"""
import sys

import config
from robot import world
from robot.board_frame import BoardFrame
from sessions import look_pose, self_test
from viz import live, scene


def presets(home):
    out = [{"name": "Resting (folded back)", "angles": list(config.POSE_REST),
            "note": "Shoulder leaned back over the electronics, elbow and wrist at 0: "
                    "the claw faces forward over the board."},
           {"name": "Ready", "angles": list(config.POSE_READY),
            "note": "High over the board: moves to the board start here."},
           {"name": "Upright", "angles": [90, 90, 90, 90, 90, 150], "note": "All servos at 90."}]
    if home:
        out.insert(0, {"name": "Your home pose", "angles": home, "note": "Taught by hand."})
    return out


def main(path="arm_view.html"):
    frame = BoardFrame.load()
    zone = world.World.for_board(frame)
    home = look_pose.load()
    data = scene.build(zone, frame, results=self_test.latest_results(frame),
                       extra={"presets": presets(home), "start": home or list(config.POSE_REST),
                              "notes": "Built from this Jetson's saved calibration by "
                                       "viz/export.py (v%s)." % config.VERSION})
    with open(path, "w") as fh:
        fh.write(live.page(data))
    print("Wrote %s -- open it in a browser." % path)


if __name__ == "__main__":
    main(*sys.argv[1:2])
