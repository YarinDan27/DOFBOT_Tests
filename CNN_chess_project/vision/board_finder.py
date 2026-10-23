#!/usr/bin/env python3
"""
Finds the board in a photo. (Milestone M0/M2 -- not built yet.)

Plan: the printed board mat has 4 ArUco markers in its margins, outside the
playing area, so pieces never cover them. Finding them gives the board's
4 corners in the picture, with pieces on the board, in every photo.

Input:  a photo (numpy image from vision/camera.py)
Output: the board's 4 corners in pixels (a1 side first), or None if the
        markers aren't all visible.

First check that OpenCV on the Jetson has the marker module:
    python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
"""


def find_board(photo):
    raise NotImplementedError("board finder not built yet (milestone M2)")
