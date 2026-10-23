#!/usr/bin/env python3
"""
A full game. (Milestone M6 -- not built yet.)

One turn:
    scan pose -> photo -> find board -> 64 crops -> CNN -> occupancy grid
    -> human's move (must be legal) -> Stockfish -> arm moves the piece
    -> photo again to check it landed. Anything unexpected: stop and ask.
"""


def run(arm, frame, camera):
    raise NotImplementedError("game loop not built yet (milestone M6)")
