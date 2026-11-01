#!/usr/bin/env python3
"""
Cuts every collected photo into 64 labelled square pictures.

Input:  data/raw/<session>/0001.jpg + 0001.json   (learn/collect.py)
Output: data/squares/<session>/<label>/0001_e4_P.png
        label = empty / white / black (round 1); the piece letter stays in
        the file name for later (13 labels: which piece).

    python3 -m learn.make_squares            # all sessions not cut yet
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2                                   # noqa: E402

import config                                # noqa: E402
from learn.collect import colour             # noqa: E402
from vision import board_view                # noqa: E402

PIECE_NAME = {".": "e"}                      # file names can't hold "." nicely


def cut_session(session_dir, out_dir):
    """Cuts one session. Returns the number of square pictures written."""
    session = os.path.basename(os.path.normpath(session_dir))
    written = 0
    for name in sorted(os.listdir(session_dir)):
        if not name.endswith(".json"):
            continue
        with open(os.path.join(session_dir, name)) as fh:
            meta = json.load(fh)
        photo = cv2.imread(os.path.join(session_dir, name[:-5] + ".jpg"))
        if photo is None or not meta.get("corners"):
            continue
        crops = board_view.square_crops(photo, meta["corners"])
        for sq, img in crops.items():
            piece = meta["position"].get(sq, ".")
            folder = os.path.join(out_dir, session, colour(piece))
            if not os.path.isdir(folder):
                os.makedirs(folder)
            cv2.imwrite(os.path.join(folder, "%s_%s_%s.png" % (
                name[:-5], sq, PIECE_NAME.get(piece, piece))), img)
            written += 1
    return written


def main(raw_dir=None, out_dir=None, again=False):
    raw_dir = raw_dir or os.path.join(config.DATA_DIR, "raw")
    out_dir = out_dir or os.path.join(config.DATA_DIR, "squares")
    if not os.path.isdir(raw_dir):
        print("No photos yet (%s). Collect some first: menu 9 on the arm." % raw_dir)
        return 0
    total = 0
    for session in sorted(os.listdir(raw_dir)):
        src = os.path.join(raw_dir, session)
        if not os.path.isdir(src):
            continue
        if os.path.isdir(os.path.join(out_dir, session)) and not again:
            continue
        n = cut_session(src, out_dir)
        print("  %s: %d square pictures" % (session, n))
        total += n
    print("Done: %d new square pictures in %s" % (total, out_dir))
    return total


if __name__ == "__main__":
    main(again="--again" in sys.argv)
