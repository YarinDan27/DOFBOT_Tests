# CNN chess project

A DOFBOT Pro that plays chess: a camera and a convolutional neural network (CNN) read the board, Stockfish picks the move, and the arm moves the piece.

## Start here

```bash
cd ~/YDscripts/CNN_chess_project && python3 run.py
```

On start, the arm goes **home**: the look-at-board pose you taught (option 2), or the ready pose until then. If it's low over the board (for example where it fell when the power went off), it rises straight up first.

1. **Teach the board by hand.** The servos switch off and you push the claw onto 5 squares (default b3 g3 b6 g6 e5 -- rank 1 and 2 are too close to the base to teach with the claw straight down). At each one the program reads the joint angles and takes a photo, so the camera sees the claw in the right place. It fits the board to your touches and flags any touch that disagrees with the rest, so you redo just that one.
2. **Teach the look-at-board pose.** The servos switch off; hold the arm where the camera sees the whole board (watch the stream), press Enter. The arm starts here and returns here after every job.
3. **Self-test.** The robot hovers over squares by itself, reads its joints back, and reports how far off it is on each one, as a map of the board. Every photo is saved for the camera check.
4. **Reach map / hover / move one piece** (4–6) to check it with your own eyes.

No-hardware checks (laptop or Jetson):

```bash
python3 tests/test_geometry.py && python3 tests/test_safety.py && python3 tests/test_arm.py
```

**Camera:** on start it stops any other program holding the camera (what `fuser` shows), then tries each `/dev/video*` until one gives a real picture, and says which one it used. Menu 7 shows the camera and its frame rate.

## Layout

| Folder | Job | Status |
|---|---|---|
| `run.py` | the menu; with `sessions/`, the only code that connects arm, camera and game | working |
| `config.py` | every fixed number | working |
| `robot/` | moves things: `kinematics.py`, `arm.py`, `board_frame.py`, `safety.py` (Jetson only) | working |
| `sessions/` | things the robot does with you: `teach.py`, `look_pose.py`, `self_test.py`, `play.py` | all working except play (milestone M6) |
| `vision/` | sees things: `camera.py` working; `claw_check.py`, `board_finder.py`, `board_view.py`, `square_cnn.py`, `classifier.py` to build | partly |
| `game/` | thinks: `state.py` (python-chess), `engine.py` (Stockfish) | milestone M5 |
| `learn/` | the CNN workshop: collect, make squares, dataset, train, evaluate | milestones M3–M4 |
| `calibration/` | machine-written: `board_arm.json`, `look_pose.json` | written by options 1 and 2 |
| `data/`, `logs/` | photos and reports (git-ignored) | written by sessions |
| `tests/` | no-hardware checks | working |

**Rule:** `robot/`, `vision/` and `game/` never import each other. That keeps vision testable on saved photos and chess logic testable without the arm.

## Safety

**v0.2.2, after the arm drove into the board on 2026-10-24:**
- **Start-up says where it thinks the claw is and waits for your Enter.** Standing upright, z should be about 423 mm. If what it says doesn't match what you see, press Ctrl-C.
- **Every move is checked against the servos' read-back**, in small sub-steps (5° for joint moves). If a joint falls more than 6° short of where it was sent, because something is in the way or the arm isn't where the program thought, it **stops and holds still**. It won't move again until you say the way is clear.
- **No power-on and no move if a servo doesn't answer** — you move the arm by hand instead. (v0.2.1's fallback read is gone; a bad reading from it is the likely cause of the crash.) A joint past the robot's *move* limits is fine: power comes on holding it exactly there, and it comes back inside the limits in small checked steps (v0.2.4).
- **Right after power-on, every servo must be where it said it was**, and the readings must make physical sense (no part of the arm under the table). Otherwise a servo is misreporting: it stops.
- **Only one program on the arm.** At start it looks for Yahboom's ROS arm driver (`arm.launch`, started at boot) and for another copy of this program, which would send their own commands to the same servos and hold the camera. Stop the ROS driver with `sudo pkill -f arm.launch`.
- **Ctrl-C never moves the arm.** Option 8 parks and quits.
- Every command and read-back is logged in `logs/motion/`.
- `python3 tests/test_arm.py` runs the real arm code against a simulated arm that stalls on the board and can misreport a servo. Nothing moves.

**From v0.2.0:**

- Every angle is checked against `config.SERVO_LIMITS` before anything moves.
- Moves near the board are straight lines in 10 mm steps, and the **whole route is solved before the arm moves**. If any point is unreachable, nothing moves.
- **Every joint swing is simulated first** (`robot/safety.py`): going home, parking, the jump onto a straight-line path. If any part of the arm would pass lower than 60 mm over the board (or below the table), that swing is refused, and the planner tries one servo at a time in every order, or via standing upright. Nothing safe? It asks you to lift the arm by hand.
- If the arm is low over the board, it **rises straight up first**, in 5 mm steps that keep the claw on its line.
- Power comes on **aimed at where the arm already is**, at start-up and after teaching, so it shouldn't jump.
- Teaching: **hold the arm** before servo power goes off and before it comes back on. Watch it closely the first time.

The danger zone is the taught board plus 25 mm; before a board is taught, a large area in front of the arm (`config.DEFAULT_ZONE`).

## Coordinates

Standing behind the arm, looking at the board: X = right, Y = toward the board, Z = up, in mm, with the origin at the centre of the base on the table.
