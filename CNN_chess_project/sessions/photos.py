#!/usr/bin/env python3
"""
Training photos -- how the camera learns the pieces (menu 9).

The arm waits at home (the pose you taught with menu 7, camera on the
board) and photographs the board. You say what's on it -- Enter = the
starting position -- so every photo labels all 64 squares. Or the arm
shuffles pieces itself: it moves a random piece to a random empty square,
goes home, takes a photo, and knows the new position because it made the
move (s 20 = 20 photos, hands-free).

Once, first: the camera finds the board -- on an EMPTY board, it finds the
grid of squares (vision/board_finder.py) and remembers where the board is in
the home photo. Check the names once at http://<jetson>:8080/preview.

Then on the PC with the NVIDIA GPU (README, "Training the camera"):
copy data/raw over, python3 -m learn.make_squares, python3 -m learn.train,
and copy the new models/vNNN folder back. r here reads the board with it.
"""
import os
import random
import time

import config
from learn import collect
from robot.arm import ArmBlocked, OutOfReach, UnsafeMove
from robot.board_frame import format_grid
from vision import board_finder, board_view

PREVIEW = "board_preview.jpg"


def _photo(arm, camera):
    """
    Home -- exactly (v2.7.2: within HOME_TOL_DEG, the base always arriving
    from the same side), so the board is where the camera found it -- then a
    fresh frame. None if it isn't exactly home or the camera gave nothing
    (says which).
    """
    off = arm.go_home()
    if off is None or off > 2 * config.HOME_TOL_DEG:
        print("  Not exactly home (%s) -- no photo: the board would be shifted in it."
              % ("a joint didn't answer" if off is None else "%.1f degrees off" % off))
        return None
    time.sleep(0.5)
    photo = camera.fresh_frame(timeout=3.0)
    if photo is None:
        print("  The camera gave no picture.")
    return photo


def _find_board(arm, camera):
    """Finds the empty board in the home photo, once. Returns the corners or None."""
    print("  First, the camera finds the board: take every piece OFF, then press Enter.")
    input("  (empty board) Enter: ")
    photo = _photo(arm, camera)
    if photo is None:
        return None
    corners = board_finder.find_board(photo)
    if corners is None:
        print("  Couldn't find the board in the picture. The whole board must be in view at")
        print("  home (live: %s) -- teach a home pose that shows it (menu 7), then try again."
              % camera.url())
        return None
    pose = arm.read_angles(samples=2, ids=(1, 2, 3, 4))[:4]
    board_finder.save(corners, pose, photo.shape[1::-1])
    import cv2
    cv2.imwrite(os.path.join(config.DATA_DIR, PREVIEW), board_view.preview(photo, corners))
    print("  Found it. Check the square names once: %s/preview" % camera.url())
    print("  (a1 in the wrong corner? set VIEW_ROTATE in config.py and do this again)")
    return corners


def _view(arm, camera):
    view = board_finder.load()
    if view is not None:
        here = arm.home_pose[:4]
        if all(a is not None for a in view.get("pose", [])) and len(view["pose"]) == 4 and max(
                abs(view["pose"][i] - here[i]) for i in range(4)) <= 3.0:
            return view["corners"]
        print("  The home pose changed since the camera found the board -- finding it again.")
    return _find_board(arm, camera)


def _random_move(position, reach, rng):
    pieces = [sq for sq, p in position.items() if p != "." and reach.get(sq) == "ok"]
    empty = [sq for sq, p in position.items() if p == "." and reach.get(sq) == "ok"]
    if not pieces or not empty:
        return None
    return rng.choice(pieces), rng.choice(empty)


def _shuffle(arm, camera, frame, corners, position, n, session, saved):
    rng = random.Random()
    reach = frame.reach_map(arm.zone)
    grab = config.GRAB_BELOW_TOP_MM

    def lower(sq):
        x, y, z = frame.square_xyz(sq)
        return (x, y, z - grab)

    for k in range(1, n + 1):
        move = _random_move(position, reach, rng)
        if move is None:
            print("  No piece the arm can reach has an empty square to go to.")
            break
        src, dst = move
        try:
            gripped = arm.move_piece(lower(src), lower(dst))
        except (ArmBlocked, UnsafeMove, OutOfReach) as e:
            print("  Stopped: %s." % e)
            break
        if not gripped:
            print("  The claw closed on nothing at %s -- stopping (the photo would be "
                  "labelled wrong). Put the piece back where it was." % src)
            break
        position[dst], position[src] = position[src], "."
        photo = _photo(arm, camera)
        if photo is None:
            print("  Stopping.")
            break
        collect.save_photo(photo, position, corners, arm.home_pose[:4], session)
        saved[0] += 1
        arm.beep(1)
        print("  [%d/%d] %s -> %s, photo %d" % (k, n, src, dst, saved[0]))
    return position


def _read(arm, camera, corners):
    from vision.classifier import Classifier
    try:
        clf = Classifier()
    except Exception as e:
        print("  %s" % e)
        return
    photo = _photo(arm, camera)
    if photo is None:
        return
    result = clf.classify(board_view.square_crops(photo, corners))
    sign = {"empty": ".", "white": "W", "black": "B"}
    print(format_grid({sq: sign.get(lab, "?") + ("?" if conf < 0.8 else "")
                       for sq, (lab, conf) in result.items()}, width=3))
    unsure = clf.unsure(result)
    print("  Model %s%s" % (os.path.basename(clf.dir),
                            (" -- not sure about " + " ".join(unsure)) if unsure else ", sure of all 64"))


def run(arm, camera, frame):
    print("\n--- Training photos: the camera learns the pieces ---")
    if camera is None:
        print("  The camera isn't working (menu 6 tests it).")
        return
    corners = _view(arm, camera)
    if corners is None:
        return
    session = collect.new_session()
    saved = [0]
    position = collect.position_from_fen(collect.START_FEN)
    while True:
        print("\n  Enter = photo of the starting position   s = the arm shuffles pieces, 20 photos")
        print("  r = read the board (trained model)   q = done   (e = empty board, or a FEN)")
        raw = input("  Photos so far: %d. Choose: " % saved[0]).strip()
        if raw.lower() == "q":
            break
        if raw.lower() == "r":
            _read(arm, camera, corners)
            continue
        if raw.lower().startswith("s"):
            if frame is None:
                print("  Teach the board first (menu 1): the arm needs it to move pieces.")
                continue
            parts = raw.split()
            n = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 20
            print("  Shuffling from the last position you told me (%d pieces)."
                  % sum(p != "." for p in position.values()))
            position = _shuffle(arm, camera, frame, corners, position, n, session, saved)
            continue
        try:
            fen = collect.START_FEN if raw == "" else (collect.EMPTY_FEN if raw.lower() == "e"
                                                       else raw)
            position = collect.position_from_fen(fen)
        except ValueError as e:
            print("  That FEN doesn't work: %s." % e)
            continue
        photo = _photo(arm, camera)
        if photo is None:
            continue
        collect.save_photo(photo, position, corners, arm.home_pose[:4], session)
        saved[0] += 1
        arm.beep(1)
    if saved[0]:
        print("\n  %d photos in data/raw/%s. Train on the PC with the GPU: see the README,"
              % (saved[0], session))
        print("  \"Training the camera\".")
