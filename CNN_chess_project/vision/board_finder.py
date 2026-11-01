#!/usr/bin/env python3
"""
Finds the board in a photo taken from the home pose.

No printing needed: an EMPTY board shows a 7 x 7 grid of inner corners
(where four squares meet), which OpenCV finds to a fraction of a pixel
(cv2.findChessboardCorners). A perspective map (homography) fitted to those
49 points gives the board's 4 outer corners. The camera always looks from
the same home pose, so this is done once -- calibration/board_view.json --
and holds for every photo from there, pieces or not. (Move the home pose or
the board: do it again.)

Which corner is a1: standing behind the arm, rank 1 is nearest and file a is
on the left, so in an upright picture a1 is the bottom-left corner.
config.VIEW_ROTATE turns that round (0 / 90 / 180 / 270) if the camera's
picture is turned; the preview (http://<jetson>:8080/preview) shows the
square names on the photo, so you can check once.

Input:  a photo (BGR image from vision/camera.py)
Output: the 4 outer corners in pixels, [a1, h1, h8, a8], or None.
"""
import json
import os

import cv2
import numpy as np

import config

INNER = (7, 7)


def _name_corners(outer, rotate=0):
    """[a1, h1, h8, a8] from 4 unordered corners, by where they are in the picture."""
    pts = [tuple(map(float, p)) for p in outer]
    tl = min(pts, key=lambda p: p[0] + p[1])
    br = max(pts, key=lambda p: p[0] + p[1])
    tr = max(pts, key=lambda p: p[0] - p[1])
    bl = min(pts, key=lambda p: p[0] - p[1])
    clockwise = [bl, tl, tr, br]          # going round the picture, starting bottom-left
    k = int(round(rotate / 90.0)) % 4
    return [list(clockwise[k]), list(clockwise[k - 1]), list(clockwise[k - 2]),
            list(clockwise[k - 3])]


def find_board(photo, rotate=None):
    """The board's outer corners [a1, h1, h8, a8] (pixels), or None if it isn't found."""
    rotate = config.VIEW_ROTATE if rotate is None else rotate
    gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY) if photo.ndim == 3 else photo
    pts = None
    if hasattr(cv2, "findChessboardCornersSB"):          # OpenCV 4: the sturdier finder
        found, sb = cv2.findChessboardCornersSB(gray, INNER)
        if found:
            pts = sb.reshape(-1, 2)
    if pts is None:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, classic = cv2.findChessboardCorners(gray, INNER, flags=flags)
        if not found:
            return None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        pts = cv2.cornerSubPix(gray, classic, (5, 5), (-1, -1), criteria).reshape(-1, 2)
    # OpenCV lists them row by row from one corner: board units 1..7 each way
    grid = np.array([[i + 1.0, j + 1.0] for j in range(INNER[1]) for i in range(INNER[0])])
    h, _ = cv2.findHomography(grid, pts.astype(np.float64))
    if h is None:
        return None
    outer = cv2.perspectiveTransform(
        np.array([[[0.0, 0.0], [8.0, 0.0], [8.0, 8.0], [0.0, 8.0]]]), h)[0]
    return _name_corners(outer, rotate)


# ---------------- saved once, for the home pose ----------------
def save(corners, pose, size, path=None):
    path = path or config.BOARD_VIEW_FILE
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w") as fh:
        json.dump({"corners": [[round(c, 2) for c in p] for p in corners],
                   "pose": [None if a is None else round(float(a), 1) for a in (pose or [])],
                   "image_size": list(size), "rotate": config.VIEW_ROTATE}, fh, indent=2)


def load(path=None):
    """{'corners', 'pose', 'image_size'} or None."""
    path = path or config.BOARD_VIEW_FILE
    if not os.path.exists(path):
        return None
    with open(path) as fh:
        return json.load(fh)
