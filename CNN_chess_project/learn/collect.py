#!/usr/bin/env python3
"""
Training photos: one photo of the board from the home pose + what's on
every square. Runs on the Jetson (menu 9, sessions/photos.py).

Each photo is saved as   data/raw/<session>/0001.jpg
with its labels in       data/raw/<session>/0001.json
    {"position": {"a1": "R", "e4": ".", ...}, "corners": [...], "pose": [...]}
Pieces are FEN letters: white = capitals (PNBRQK), black = small (pnbrqk),
"." = empty. Knowing the position labels all 64 squares of the photo for
free -- one photo = 64 training examples.
"""
import json
import os
import time

import config

FILES = "abcdefgh"
START_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"
EMPTY_FEN = "8/8/8/8/8/8/8/8"


def squares():
    return [f + str(r) for r in range(1, 9) for f in FILES]


def position_from_fen(fen):
    """{'a1': 'R', ..., 'h8': 'r'} from the board part of a FEN (rank 8 first)."""
    board = fen.strip().split()[0]
    rows = board.split("/")
    if len(rows) != 8:
        raise ValueError("a FEN board has 8 ranks separated by /")
    out = {}
    for i, row in enumerate(rows):
        rank = 8 - i
        f = 0
        for ch in row:
            if ch.isdigit():
                for _ in range(int(ch)):
                    if f > 7:
                        raise ValueError("rank %d is too long" % rank)
                    out[FILES[f] + str(rank)] = "."
                    f += 1
            elif ch in "pnbrqkPNBRQK":
                if f > 7:
                    raise ValueError("rank %d is too long" % rank)
                out[FILES[f] + str(rank)] = ch
                f += 1
            else:
                raise ValueError("%r isn't a piece letter" % ch)
        if f != 8:
            raise ValueError("rank %d doesn't have 8 squares" % rank)
    return out


def fen_from_position(position):
    rows = []
    for rank in range(8, 0, -1):
        row, gap = "", 0
        for f in FILES:
            p = position.get(f + str(rank), ".")
            if p == ".":
                gap += 1
            else:
                row += (str(gap) if gap else "") + p
                gap = 0
        rows.append(row + (str(gap) if gap else ""))
    return "/".join(rows)


def colour(piece):
    """'empty' / 'white' / 'black' -- the 3 labels of round 1."""
    if piece == ".":
        return "empty"
    return "white" if piece.isupper() else "black"


def new_session():
    return time.strftime("%Y%m%d-%H%M%S")


def save_photo(photo, position, corners, pose, session, raw_dir=None):
    """Writes the next NNNN.jpg + NNNN.json in the session folder. Returns the jpg path."""
    import cv2
    folder = os.path.join(raw_dir or os.path.join(config.DATA_DIR, "raw"), session)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    n = len([f for f in os.listdir(folder) if f.endswith(".jpg")]) + 1
    base = os.path.join(folder, "%04d" % n)
    cv2.imwrite(base + ".jpg", photo, [cv2.IMWRITE_JPEG_QUALITY, 95])
    with open(base + ".json", "w") as fh:
        json.dump({"position": position, "fen": fen_from_position(position),
                   "corners": corners, "pose": pose, "time": time.time()}, fh, indent=1)
    return base + ".jpg"
