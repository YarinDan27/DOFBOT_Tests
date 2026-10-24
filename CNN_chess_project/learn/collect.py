#!/usr/bin/env python3
"""
Collects training photos. Runs on the Jetson. (Milestone M3 -- not built yet.)

Set up a known position, the robot photographs it from the scan pose, and
saves:  data/raw/<session>/0001.jpg  +  0001.json  {"fen": ..., "scan_pose": ...}
The FEN says what's on every square, so all 64 squares are labelled for free.
"""


def collect(camera, fen):
    raise NotImplementedError("data collection not built yet (milestone M3)")
