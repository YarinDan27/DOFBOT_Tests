#!/usr/bin/env python3
"""
Turns a photo into 64 square pictures.

Input:  a photo + the board's 4 corners [a1, h1, h8, a8] (board_finder.py)
Output: {'a1': image, ..., 'h8': image}, each CROP_H x CROP_W (96 x 64).

A perspective warp (cv2.getPerspectiveTransform + cv2.warpPerspective)
turns the slanted board into a flat top-down one, rank 8 at the top. Each
crop is its square plus half a square above it: seen from the arm's side,
a piece's top leans toward the far ranks, into the square behind.
"""
import cv2
import numpy as np

FILES = "abcdefgh"       # (vision/ doesn't import robot/: see the README's rule)

PX = 64                  # warped pixels per square
LEAN = 0.5               # extra square-heights above each square
CROP_W, CROP_H = PX, int(PX * (1 + LEAN))
TOP = int(PX * LEAN)     # margin above rank 8, so its crops fit too


def warp(photo, corners):
    """Top-down board: 8*PX wide, 8*PX + TOP tall (rank 8 at the top)."""
    src = np.array(corners, dtype=np.float32)
    w = 8 * PX
    dst = np.array([[0, TOP + w], [w, TOP + w], [w, TOP], [0, TOP]], dtype=np.float32)
    m = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(photo, m, (w, TOP + w), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REPLICATE)


def box(square):
    """(x0, y0, x1, y1) of a square's crop in the warped board."""
    f, r = FILES.index(square[0]), int(square[1]) - 1
    x0 = f * PX
    y1 = TOP + (8 - r) * PX
    return x0, y1 - CROP_H, x0 + PX, y1


def square_crops(photo, corners):
    flat = warp(photo, corners)
    out = {}
    for r in range(1, 9):
        for f in FILES:
            sq = f + str(r)
            x0, y0, x1, y1 = box(sq)
            out[sq] = flat[y0:y1, x0:x1].copy()
    return out


def preview(photo, corners, labels=None):
    """
    The photo with the board outline and every square's name (or `labels`:
    {square: text}) drawn where the program thinks it is -- to check once
    that a1 really is a1.
    """
    img = photo.copy()
    src = np.array(corners, dtype=np.float32)
    w = 8.0
    dst = np.array([[0, w], [w, w], [w, 0], [0, 0]], dtype=np.float32)
    back = cv2.getPerspectiveTransform(dst, src)       # board units -> photo pixels
    cv2.polylines(img, [src.astype(np.int32).reshape(-1, 1, 2)], True, (0, 220, 255), 2)
    for r in range(1, 9):
        for i, f in enumerate(FILES):
            p = cv2.perspectiveTransform(np.array([[[i + 0.5, 8 - r + 0.5]]], np.float32), back)
            x, y = int(p[0][0][0]), int(p[0][0][1])
            text = (labels or {}).get(f + str(r), f + str(r))
            cv2.putText(img, text, (x - 9, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 0, 0), 3)
            cv2.putText(img, text, (x - 9, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                        (255, 255, 255), 1)
    a1 = tuple(int(c) for c in corners[0])
    cv2.circle(img, a1, 6, (0, 0, 255), -1)
    return img
