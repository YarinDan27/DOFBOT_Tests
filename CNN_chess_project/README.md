# CNN chess project

A DOFBOT Pro that plays chess: a camera and a convolutional neural network (CNN) read the board, Stockfish picks the move, and the arm moves the piece.

## Start here

```bash
cd ~/YDscripts/CNN_chess_project && python3 run.py
```

1. **Teach the board by hand.** The servos switch off and you push the claw onto 5 squares. At each one the program reads the joint angles and takes a photo, so the camera sees the claw in the right place. It fits the board to your touches and flags any touch that disagrees with the rest, so you redo just that one.
2. **Self-test.** The robot hovers over squares by itself, reads its joints back, and reports how far off it is on each one, as a map of the board. Every photo is saved for the camera check.
3. **Reach map / hover / move one piece** to check it with your own eyes.

No-hardware checks (laptop or Jetson):

```bash
python3 tests/test_geometry.py
```

## Layout

| Folder | Job | Status |
|---|---|---|
| `run.py` | the menu; with `sessions/`, the only code that connects arm, camera and game | working |
| `config.py` | every fixed number | working |
| `robot/` | moves things: `kinematics.py`, `arm.py`, `board_frame.py` (Jetson only) | working |
| `sessions/` | things the robot does with you: `teach.py`, `self_test.py`, `play.py` | teach + self-test working; play is milestone M6 |
| `vision/` | sees things: `camera.py` working; `claw_check.py`, `board_finder.py`, `board_view.py`, `square_cnn.py`, `classifier.py` to build | partly |
| `game/` | thinks: `state.py` (python-chess), `engine.py` (Stockfish) | milestone M5 |
| `learn/` | the CNN workshop: collect, make squares, dataset, train, evaluate | milestones M3–M4 |
| `calibration/` | machine-written: `board_arm.json` | written by Teach |
| `data/`, `logs/` | photos and reports (git-ignored) | written by sessions |
| `tests/` | no-hardware checks | working |

**Rule:** `robot/`, `vision/` and `game/` never import each other. That keeps vision testable on saved photos and chess logic testable without the arm.

## Safety (carried over from `old_chess_project`)

- Every angle is checked against `config.SERVO_LIMITS` before anything moves.
- Every move is a straight line in 10 mm steps, and the **whole route is solved before the arm moves**. If any point is unreachable, nothing moves.
- Teaching: **hold the arm** before servo power goes off and before it comes back on. It comes back on aimed at where it already is, so it shouldn't jump. Watch it closely the first time.
- After a pick or a hover, the arm rises straight up before swinging to the ready pose, so it never sweeps low over pieces.

## Coordinates

Standing behind the arm, looking at the board: X = right, Y = toward the board, Z = up, in mm, with the origin at the centre of the base on the table.
