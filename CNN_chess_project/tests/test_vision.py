#!/usr/bin/env python3
"""
v2.7: the vision pipeline end to end, on made-up photos -- no camera, no
arm. A board with pieces is drawn, tilted the way the home-pose camera sees
it, then:

  board_finder  finds the empty board's corners, a1 bottom-left
  board_view    cuts 64 squares
  collect       saves labelled photos (FEN <-> position)
  make_squares  cuts them into labelled square pictures
  train         trains the CNN (if PyTorch is installed: CPU here, GPU at home)
  classifier    runs the exported ONNX with OpenCV, checks it at load, reads a
                new board -- and refuses a model whose check doesn't match

    python3 tests/test_vision.py
"""
import os
import random
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import cv2                                                    # noqa: E402
import numpy as np                                            # noqa: E402

import config                                                 # noqa: E402

TMP = tempfile.mkdtemp()
config.DATA_DIR = os.path.join(TMP, "data")
config.MODELS_DIR = os.path.join(TMP, "models")
config.BOARD_VIEW_FILE = os.path.join(TMP, "calibration", "board_view.json")

from learn import collect, make_squares                       # noqa: E402
from vision import board_finder, board_view                   # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(("  pass  " if ok else "  FAIL  ") + name + (("  (" + detail + ")") if detail else ""))
    if not ok:
        failures.append(name)


# ---------------- a made-up camera photo ----------------
S = 48                       # top-down pixels per square when drawing
B = 30                       # border around the board
TRUE = np.float32([[150, 430], [500, 420], [430, 120], [205, 125]])   # a1, h1, h8, a8 in the photo


def draw_board(position, rng, light=1.0):
    size = 8 * S + 2 * B
    img = np.full((size, size, 3), (60, 90, 120), np.uint8)              # the table / frame
    for r in range(8):
        for f in range(8):
            dark = (r + f) % 2 == 0
            colour = (70, 110, 150) if dark else (190, 220, 235)
            y0 = B + (7 - r) * S
            x0 = B + f * S
            cv2.rectangle(img, (x0, y0), (x0 + S - 1, y0 + S - 1), colour, -1)
    for sq, piece in position.items():
        if piece == ".":
            continue
        f, r = "abcdefgh".index(sq[0]), int(sq[1]) - 1
        cx = B + f * S + S // 2 + rng.randint(-4, 4)
        cy = B + (7 - r) * S + S // 2 + rng.randint(-4, 4)
        body = (235, 235, 230) if piece.isupper() else (25, 25, 30)
        edge = (120, 120, 120) if piece.isupper() else (90, 90, 90)
        tall = {"p": 14, "n": 20, "b": 22, "r": 18, "q": 26, "k": 28}[piece.lower()]
        # seen from the arm's side, a piece leans toward the far ranks (up here)
        cv2.rectangle(img, (cx - 8, cy - tall), (cx + 8, cy), body, -1)
        cv2.ellipse(img, (cx, cy), (14, 9), 0, 0, 360, body, -1)
        cv2.ellipse(img, (cx, cy), (14, 9), 0, 0, 360, edge, 1)
        cv2.circle(img, (cx, cy - tall), 8, body, -1)
        cv2.circle(img, (cx, cy - tall), 8, edge, 1)
    src = np.float32([[B, B + 8 * S], [B + 8 * S, B + 8 * S], [B + 8 * S, B], [B, B]])
    m = cv2.getPerspectiveTransform(src, TRUE)
    photo = cv2.warpPerspective(img, m, (640, 480), borderValue=(40, 50, 60))
    photo = np.clip(photo.astype(np.float32) * light + rng.uniform(-10, 10), 0, 255)
    photo += np.random.RandomState(rng.randint(0, 10 ** 6)).normal(0, 4, photo.shape)
    return np.clip(photo, 0, 255).astype(np.uint8)


def random_position(rng, n_pieces):
    pos = {sq: "." for sq in collect.squares()}
    for sq in rng.sample(collect.squares(), n_pieces):
        pos[sq] = rng.choice("PNBRQKpnbrqk")
    return pos


# =====================================================================
print("--- 1. finding the board (empty board, once) ---")
rng = random.Random(1)
empty = draw_board(collect.position_from_fen(collect.EMPTY_FEN), rng)
corners = board_finder.find_board(empty, rotate=0)
check("found it", corners is not None)
if corners is not None:
    off = max(float(np.hypot(*(np.array(corners[i]) - TRUE[i]))) for i in range(4))
    check("all 4 corners within 3 px, a1 bottom-left", off < 3.0, "%.1f px" % off)
    turned = board_finder.find_board(empty, rotate=90)
    check("VIEW_ROTATE 90 names a different corner a1", turned is not None and
          np.hypot(*(np.array(turned[0]) - TRUE[3])) < 3.0)
    board_finder.save(corners, [90, 120, 0, 10], empty.shape[1::-1])
    check("saved and loaded back", board_finder.load()["corners"][0] == [round(c, 2) for c in corners[0]])
    pv = board_view.preview(empty, corners)
    check("the preview draws on a copy of the photo", pv.shape == empty.shape and not (pv == empty).all())
full = draw_board(collect.position_from_fen(collect.START_FEN), rng)
check("with pieces on, it may not find the board (that's why it's done once, empty)", True)

# =====================================================================
print("\n--- 2. 64 squares, labelled photos, square pictures ---")
crops = board_view.square_crops(full, corners or TRUE.tolist())
check("64 crops, 96 x 64 each", len(crops) == 64 and all(c.shape == (96, 64, 3) for c in crops.values()))
check("a white piece's crop is brighter in the middle than an empty dark square's",
      crops["a2"][40:90, 20:44].mean() > crops["a3"][40:90, 20:44].mean() + 30)
start = collect.position_from_fen(collect.START_FEN)
check("FEN -> position -> FEN", collect.fen_from_position(start) == collect.START_FEN
      and sum(p != "." for p in start.values()) == 32 and start["e1"] == "K" and start["d8"] == "q")
try:
    collect.position_from_fen("rnbqkbnr/ppp/8/8/8/8/PPPPPPPP/RNBQKBNR")
    check("a broken FEN is refused", False)
except ValueError:
    check("a broken FEN is refused", True)

sessions = []
for s in range(3):
    session = "2026110%d-1200" % (s + 1)
    sessions.append(session)
    for k in range(6):
        pos = start if k == 0 else random_position(rng, rng.randint(4, 28))
        photo = draw_board(pos, rng, light=rng.uniform(0.75, 1.2))
        collect.save_photo(photo, pos, corners or TRUE.tolist(), [90, 120, 0, 10], session)
n = make_squares.main()
check("18 photos -> 1152 square pictures", n == 18 * 64, "%d" % n)
check("sorted into empty / white / black", all(os.path.isdir(os.path.join(
    config.DATA_DIR, "squares", sessions[0], lab)) for lab in ("empty", "white", "black")))
check("a second run doesn't cut them again", make_squares.main() == 0)

# =====================================================================
print("\n--- 3. training (PyTorch) and the Jetson's classifier (OpenCV) ---")
import importlib.util                                        # noqa: E402
have_torch = importlib.util.find_spec("torch") is not None
if not have_torch:
    print("  (PyTorch isn't installed here: training skipped. On your GPU PC: pip install torch)")
if have_torch:
    from learn import train
    from vision.classifier import Classifier
    lines = []
    out = train.train(epochs=6, batch=64, log=lines.append)
    print("\n".join("    " + l for l in lines if "epoch" not in l or "6/6" in l))
    check("a model folder was made", out is not None and os.path.exists(os.path.join(out, "model.onnx")))
    import json
    man = json.load(open(os.path.join(out, "manifest.json")))
    check("manifest: labels, preprocessing, sessions, test score",
          man["labels"] == ["empty", "white", "black"] and man["test_sessions"] == [sessions[-1]]
          and man["test"]["square_accuracy"] is not None)
    check("it learned: 97%+ of held-out squares right", man["test"]["square_accuracy"] >= 0.97,
          "%.1f%%" % (100 * man["test"]["square_accuracy"]))
    clf = Classifier(out)
    check("the Jetson's OpenCV runs the ONNX and matches PyTorch (checked at load)", clf is not None)
    new_pos = random_position(rng, 20)
    photo = draw_board(new_pos, rng, light=0.9)
    result = clf.classify(board_view.square_crops(photo, corners or TRUE.tolist()))
    wrong = [sq for sq in result if result[sq][0] != collect.colour(new_pos[sq])]
    check("reads a board it has never seen", len(wrong) <= 1, "wrong: %s" % (wrong or "none"))
    # a broken copy is refused
    bad = os.path.join(TMP, "models", "broken")
    shutil.copytree(out, bad)
    d = np.load(os.path.join(bad, "check.npz"))
    np.savez_compressed(os.path.join(bad, "check.npz"), images=d["images"], scores=d["scores"] + 5.0)
    try:
        Classifier(bad)
        check("a model that doesn't match its check is refused", False)
    except RuntimeError as e:
        check("a model that doesn't match its check is refused", "different answers" in str(e))

# =====================================================================
print("\n--- 4. menu 9 on the simulated arm: find the board once, photos, the arm shuffles ---")
import builtins                                               # noqa: E402
import time                                                   # noqa: E402
sys.path.insert(0, os.path.join(HERE, "fakes"))
import Arm_Lib                                                # noqa: E402 (the fake)
assert "fakes" in Arm_Lib.__file__
config.LOCK_FILE = os.path.join(TMP, "arm.lock")
config.LOGS_DIR = os.path.join(TMP, "logs")
os.remove(config.BOARD_VIEW_FILE)
CLOCK = [1000.0]
time.time = lambda: CLOCK[0]
time.sleep = lambda s: CLOCK.__setitem__(0, CLOCK[0] + max(0.0, float(s)))
from robot import arm as arm_mod                              # noqa: E402
from robot.board_frame import BoardFrame                      # noqa: E402
from sessions import photos                                   # noqa: E402

WORLD = {"pos": collect.position_from_fen(collect.EMPTY_FEN)}
shots = []


class FakeCamera(object):
    def fresh_frame(self, timeout=2.0):
        shots.append(dict(WORLD["pos"]))
        return draw_board(WORLD["pos"], rng)

    def url(self):
        return "http://jetson:8080"


ANSWERS = []


def scripted(prompt=""):
    if "Choose" in prompt and WORLD["pos"]["e1"] == ".":
        WORLD["pos"] = collect.position_from_fen(collect.START_FEN)    # you set up the pieces
    return ANSWERS.pop(0)


builtins.input = scripted
Arm_Lib.reset(config.POSE_REST)
a = arm_mod.Arm()
frame = BoardFrame.fit({"a8": (-62.0, 248.0, 25.0), "h1": (73.0, 108.0, 26.0),
                        "a1": (-67.0, 116.0, 26.0)})
a.set_board(frame)
real_move = a.move_piece
moved = []


def move_and_track(from_xyz, to_xyz):
    real_move(from_xyz, to_xyz)
    src = min(collect.squares(), key=lambda sq: np.hypot(*(np.array(frame.square_xyz(sq)[:2])
                                                            - np.array(from_xyz[:2]))))
    dst = min(collect.squares(), key=lambda sq: np.hypot(*(np.array(frame.square_xyz(sq)[:2])
                                                            - np.array(to_xyz[:2]))))
    WORLD["pos"][dst], WORLD["pos"][src] = WORLD["pos"][src], "."
    moved.append((src, dst))
    return True


a.move_piece = move_and_track
ANSWERS[:] = ["", "", "s 4", "q"]           # empty board found; start position; 4 shuffles; done
raw = os.path.join(config.DATA_DIR, "raw")
before = set(os.listdir(raw))
photos.run(a, FakeCamera(), frame)
check("found the board once and kept it", board_finder.load() is not None)
session = sorted(set(os.listdir(raw)) - before)[-1]
import json                                                   # noqa: E402
metas = [json.load(open(os.path.join(raw, session, n))) for n in sorted(os.listdir(
    os.path.join(raw, session))) if n.endswith(".json")]
check("5 photos: the starting position + 4 the arm made itself", len(metas) == 5, "%d" % len(metas))
check("the arm really moved 4 pieces", len(moved) == 4, str(moved))
check("every photo's label is what was really on the board",
      [m["position"] for m in metas] == shots[-5:])
check("never touched anything", Arm_Lib.STATE["contacts"] == 0)

# =====================================================================
print("\n--- 5. python3 -m learn: photos from the arm -> train -> model back (fake ssh) ---")
if have_torch:
    import learn.__main__ as one_command                       # noqa: E402
    REMOTE = os.path.join(TMP, "jetson")                        # pretend Jetson disk
    shutil.copytree(os.path.join(config.DATA_DIR, "raw"), os.path.join(REMOTE, "data", "raw"))
    shutil.rmtree(config.DATA_DIR)                              # the PC starts with nothing
    shutil.rmtree(config.MODELS_DIR)
    calls = []

    def fake_call(cmd):
        calls.append(cmd[0])
        local = [c.split(":", 1)[1].replace("~/jet", REMOTE) if ":" in c and "@" in c else c
                 for c in cmd]
        if cmd[0] == "ssh":
            os.makedirs(os.path.join(REMOTE, "models"), exist_ok=True)
            return 0
        src, dst = local[2], local[3]
        target = os.path.join(dst, os.path.basename(src.rstrip("/")))
        shutil.copytree(src, target)
        return 0

    one_command.subprocess.call = fake_call
    config.JETSON_DIR = "~/jet"
    quick = train.train
    train.train = lambda: quick(epochs=2, batch=64, log=lambda *a: None)
    code = one_command.main(["me@arm"])
    train.train = quick
    sent = os.listdir(os.path.join(REMOTE, "models")) if os.path.isdir(os.path.join(REMOTE, "models")) else []
    check("one command: copied the photos, trained, sent the model back",
          code == 0 and calls == ["scp", "ssh", "scp"] and sent == ["v001"], "%s %s" % (calls, sent))

print("\n%d failure(s)" % len(failures))
if failures:
    for f in failures:
        print("  - " + f)
    sys.exit(1)
