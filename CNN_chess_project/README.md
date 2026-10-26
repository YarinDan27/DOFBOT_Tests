# CNN chess project

A DOFBOT Pro that plays chess: a camera and a convolutional neural network (CNN) read the board, Stockfish picks the move, and the arm moves the piece.

## Start here

```bash
cd ~/YDscripts/CNN_chess_project && python3 run.py
```

**v2.4.3.** No questions at start-up. The banner shows the version and what was last fixed. Power comes on right where the arm is, it says where the claw is, and after 3 s (Ctrl-C = stay) it goes **home**: up first, then across. Home is the camera view you taught (option 6), or the ready pose until then.

| Menu | What it does |
|---|---|
| 1 Teach the board | **Hands-free at the arm**, see below |
| 2 Go home | |
| 3 Board map | Coloured 8×8 map of how sure the arm is about each square: green = within 2 mm, yellow = within 5 mm, red = not trusted, grey = out of reach, `?` = not tested, `*` = taught. Type a square to hover over it, or two (`e2 e4`) to move that piece |
| 4 Self-test | Hovers over every reachable square, no touching, reads its joints back and fills in the map. Ctrl-C stops; results are kept and added to |
| 5 Camera test | Frees the camera, then shows frame rate and brightness, saves a photo and gives the live-view link |
| 6 Teach the camera view | Hands-free: hold the arm high, camera on the board, still for 3 s |
| 0 Quit | The arm stays where it is, holding |
| c Carry on | Only shown after a stopped move, once you've looked at the arm |

### Teaching the board, hands-free

Put a **pawn** on each square to teach (default b3 g3 b6 g6 e5). Choose 1 and press Enter. The arm closes its claw and comes down low over the board. Then walk over:

1. **Countdown:** beeps for the last 5 s, then a **long beep**. Power is off, so be holding the arm by then.
2. For each square in order, put the **closed claw tip on top of the pawn**, with the claw pointing down or tilted a little, and **hold still 2 s**. Beeps = how many squares are done (1, 2, 3…).
3. Light: **blue** = waiting, **red** = a joint can't report (tilt the claw a bit more), **green** = recorded.
4. After the last square: a **long beep**. Power is back on and the arm holds itself. Let go.

To stop early, lift the claw up high and hold still 4 s (two beeps). If it can't switch power back on, you get 3 beeps and a red light. If all touches agree, it saves by itself. If one is off, the computer asks you to redo just that one. Grabbing goes 15 mm below the taught pawn tops (`GRAB_BELOW_TOP_MM`).

No-hardware checks (laptop or Jetson):

```bash
python3 tests/test_geometry.py && python3 tests/test_safety.py && python3 tests/test_arm.py && python3 tests/test_teach.py
```

`test_teach.py` covers hands-free teaching with a pretend hand and clock: a silent wrist, stopping early, walking away, redoing a bad square, the camera view, and the whole menu. It also checks that no command ever turned a joint more than 9° at once.

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
| `calibration/` | machine-written: `board_arm.json`, `look_pose.json` | written by options 1 and 6 |
| `data/`, `logs/` | photos and reports (git-ignored) | written by sessions |
| `tests/` | no-hardware checks | working |

**Rule:** `robot/`, `vision/` and `game/` never import each other. That keeps vision testable on saved photos and chess logic testable without the arm.

## Safety

**v0.2.2, after the arm drove into the board on 2026-10-24:**
- **Start-up says where it thinks the claw is**, then waits 3 s before going home (Ctrl-C = stay). If power had to wait for a joint you were fixing by hand, it doesn't move at all until you choose 2.
- **Every move is checked against the servos' read-back**, in small sub-steps (5° for joint moves). If a joint falls more than 6° short of where it was sent, because something is in the way or the arm isn't where the program thought, it **stops and holds still**. It won't move again until you say the way is clear.
- **No power-on and no move if a servo doesn't answer** — you move the arm by hand instead. (v0.2.1's fallback read is gone; a bad reading from it is the likely cause of the crash.) A joint past the robot's *move* limits is fine: power comes on holding it exactly there, and it comes back inside the limits in small checked steps (v0.2.4).
- **Right after power-on, every servo must be where it said it was**, and the readings must make physical sense (no part of the arm under the table). Otherwise a servo is misreporting: it stops.
- **Only one program on the arm.** At start it looks for Yahboom's ROS arm driver (`arm.launch`, started at boot) and for another copy of this program, which would send their own commands to the same servos and hold the camera. Stop the ROS driver with `sudo pkill -f arm.launch`.
- **Ctrl-C never moves the arm**, and neither does quitting.
- **(2.4.3) Joints a little past the end of their scale still report.** Yahboom's `Arm_serial_servo_read` returns nothing outside 0–180°, so the folded pose (shoulder ~182°) or a vertical claw on a piece (wrist ~−5°) looked like a dead servo. `robot/arm.py` reads the same register with the same maths, up to 20° past either end. It does *not* use `Arm_serial_servo_read_any`: that one skips the flip servos 2–4 need, so a wrist at −6° reads as 186°, the likely cause of the 2026-10-24 crash.
- **(2.4.2) Straight lines never flip the elbow halfway.** Every 10 mm step must follow on from the one before (no joint turns more than 6°). Near the base the step is split finer. Before, a line could jump between two elbow solutions in one step.
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
