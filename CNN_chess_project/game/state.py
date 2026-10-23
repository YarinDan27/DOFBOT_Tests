#!/usr/bin/env python3
"""
The game: the position, and working out what the human played. (Milestone M5.)

Input:  the current position + a new 8x8 grid of empty/white/black from the CNN
Output: the human's move (e.g. 'e2e4'), only if exactly ONE legal move
        explains the change. Otherwise None, and the robot stops and asks.

Built on python-chess (rules, legal moves, FEN). Check which version still
supports Python 3.6 before installing it on the Jetson.
"""


def detect_move(position, occupancy):
    raise NotImplementedError("move detection not built yet (milestone M5)")
