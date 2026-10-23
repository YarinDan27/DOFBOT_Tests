#!/usr/bin/env python3
"""
Camera check for the self-test: is the claw really over the square?

Input:  a self-test photo of square X (the robot put the claw there itself),
        and, if it exists, the teach photo of X (you put it there by hand:
        the "this is exactly right" reference).
Output: how far off the claw is, in mm.

NOT BUILT YET. The method depends on where the camera sits:
  * on the arm  -> the claw is always in the same spot in the picture, so
                   compare where the square's centre falls relative to it
  * fixed       -> the claw moves around the picture, so find the claw and
                   the square and measure the gap
It needs one real photo from the camera, with the claw over a square, to design.
"""


def claw_offset_mm(photo_path, reference_path=None):
    raise NotImplementedError("camera check not built yet -- needs a sample photo")
