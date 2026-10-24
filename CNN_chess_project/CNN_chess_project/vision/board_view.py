#!/usr/bin/env python3
"""
Turns a photo into 64 square pictures. (Milestone M2 -- not built yet.)

Input:  a photo + the board's 4 corners (from board_finder.py)
Output: {'a1': image, ..., 'h8': image} -- each crop a fixed size, and TALLER
        than the square: seen at an angle, a piece's top leans into the next
        square, so the crop includes the space above it.

How: a perspective warp (cv2.getPerspectiveTransform + cv2.warpPerspective)
turns the slanted board into a flat top-down one; then it's just slicing.
"""


def square_crops(photo, corners):
    raise NotImplementedError("board view not built yet (milestone M2)")
