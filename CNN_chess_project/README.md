# CNN chess project

A DOFBOT Pro that plays chess: a camera and a convolutional neural network (CNN) read the board, Stockfish picks the move, and the arm moves the piece.

## Start here

```bash
cd ~/YDscripts/CNN_chess_project && python3 run.py
```

On start, the arm goes **home**: the look-at-board pose you taught (option 2), or the ready pose until then. If it's low over the board (for example where it fell when the power went off), it rises straight up first.

1. **Teach the board by hand.** The servos switch off and you push the claw onto 5 squares. At each one the program reads the joint angles and takes a photo, so the camera sees the claw in the right place. It fits the board to your touches and flags any touch that disagrees with the rest, so you redo just that one.
2. **Teach the look-at-board pose.** The servos switch off; hold the arm where the camera sees the whole board (watch the stream), press Enter. The arm starts here and returns here after every job.
3. **Self-test.** The robot hovers over squares by itself, reads its joints back, and reports how far off it is on each one, as a map of the board. Every photo is saved for the camera check.
4. **Reach map / hover / move one piece** (4–6) to check it with your own eyes.

No-hardware checks (laptop or Jetson):

```bash
python3 tests/test_geometry.py && python3 tests/test_safety.py
```

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

- Every angle is checked against `config.SERVO_LIMITS` before anything moves.
- Moves near the board are straight lines in 10 mm steps, and the **whole route is solved before the arm moves**. If any point is unreachable, nothing moves.
- **Every joint swing is simulated first** (`robot/safety.py`): going home, parking, the jump onto a straight-line path. If any part of the arm would pass lower than 60 mm over the board (or below the table), that swing is refused, and the planner tries one servo at a time in every order, or via standing upright. Nothing safe? It asks you to lift the arm by hand.
- If the arm is low over the board, it **rises straight up first**, in 5 mm steps that keep the claw on its line.
- Power comes on **aimed at where the arm already is**, at start-up and after teaching, so it shouldn't jump.
- Teaching: **hold the arm** before servo power goes off and before it comes back on. Watch it closely the first time.

The danger zone is the taught board plus 25 mm; before a board is taught, a large area in front of the arm (`config.DEFAULT_ZONE`).

## Coordinates

Standing behind the arm, looking at the board: X = right, Y = toward the board, Z = up, in mm, with the origin at the centre of the base on the table.
