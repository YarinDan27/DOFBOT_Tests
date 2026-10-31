# CNN chess project

A DOFBOT Pro that plays chess: a camera and a convolutional neural network (CNN) read the board, Stockfish picks the move, and the arm moves the piece.

## Start here

```bash
cd ~/YDscripts/CNN_chess_project && python3 run.py
```

```bash
python3 run.py --sim      # the same program on a simulated arm: no arm needed (try it on your PC)
python3 run.py --check    # reads the servos and says what it sees; moves nothing
python3 -m viz.export     # writes arm_view.html: the 3D view of your calibration, to open anywhere
python3 -m viz.urdf       # writes dofbot_chess.urdf: the same arm model, for RViz or MuJoCo
```

**v2.6.0.** The banner shows the version and what was last fixed. No questions at start-up. It stops Yahboom's ROS arm driver if it's running, which may ask for your password. Power comes on right where the arm is. It says where the claw is, and after 3 s (Ctrl-C = stay) it goes **home**: resting, folded back over the electronics, or the home pose you taught.

**Live 3D view:** while it runs, open the address it prints (`http://<jetson>:8081`) in a browser on your laptop. You see the arm as the program sees it, with its pose, the move it's making, the board, the electronics box, and every part's distance to them. A **motor loop** chip shows the loop rate and any late ticks, missed reads or refused commands. The same line is printed when you quit.

| Menu | What it does |
|---|---|
| 1 Teach the board | **Hands-free at the arm**, see below |
| 2 Go home | Resting (folded back), or the home pose you taught |
| 3 Board map | Coloured 8×8 map of how sure the arm is about each square: green = within 2 mm, yellow = within 5 mm, red = not trusted, grey = out of reach, `?` = not tested, `*` = taught. Type a square to hover over it, or two (`e2 e4`) to move that piece |
| 4 Self-test | Hovers over every reachable square, no touching, reads its joints back and fills in the map. Ctrl-C stops; results are kept and added to |
| 5 Keyboard control | Move each joint (q/a w/s e/d r/f) or the claw tip (arrows, u/o) a step per key press; **m** marks the square under the claw (teaching by keyboard) |
| 6 Camera test | Frees the camera, then shows frame rate and brightness, saves a photo and gives the live camera link |
| 7 Teach the home pose | Hands-free: hold the arm where it should wait, still for 3 s |
| 8 Measure the electronics | Hands-free: touch the closed claw on the top of the black box at three corners |
| 0 Rest and quit | Folds back to rest, then quits (**q** quits leaving it where it is) |
| c Carry on | Only shown after a stopped move, once you've looked at the arm |

### Teaching the board, hands-free

Put a **pawn** on each square to teach (default b3 g3 b6 g6 e5). Choose 1 and press Enter. The arm closes its claw and comes down low over the board. Then walk over:

1. **Countdown:** beeps for the last 5 s, then a **long beep**. Power is off, so be holding the arm by then.
2. For each square in order, put the **closed claw tip on top of the pawn**, with the claw pointing down or tilted a little, and **hold still 2 s**. Beeps = how many squares are done (1, 2, 3…).
3. Light: **blue** = waiting, **red** = a joint can't report (tilt the claw a bit more), **green** = recorded.
4. After the last square: a **long beep**. Power is back on and the arm holds itself. Let go.

To stop early, lift the claw up high and hold still 4 s (two beeps). If it can't switch power back on, you get 3 beeps and a red light. If all touches agree, it saves by itself. If one is off, the computer asks you to redo just that one. Grabbing goes 15 mm below the taught pawn tops (`GRAB_BELOW_TOP_MM`).

The same flow (countdown, hold still, beeps) teaches the home pose (7) and measures the electronics (8).

## Safety: the arm's real shape (v2.5)

Up to 2.4, safety was a list of per-servo limits (shoulder 10–140 and so on). That refused good poses, like resting with the shoulder leaned back to ~185–190 and the elbow and wrist folded at 0. It still let a wrong reading drive the arm into the board, because it never knew where anything actually was.

Now there are no per-servo safety limits, only what each servo can physically do and report (`config.JOINT_RANGE`). Safety comes from geometry:

- **The arm is five solid parts** (`robot/body.py`): upper arm, forearm, wrist, gripper and the camera bar, each a capsule with its real thickness.
- **What's around it** (`robot/world.py`): the table, the base plate, the turning base, and the **electronics box** behind the arm. Every part keeps `HARD_CLEAR_MM` (6 mm) away from them in every pose and during every move. So shoulder 190 + elbow 0 + wrist 0 is fine, and shoulder 190 + elbow 180 + wrist 180 is refused, because the arm would be in the electronics.
- **The board** works as before: joint swings stay 60 mm above it, and only planned straight lines go down to a piece.
- **The electronics box is an estimate** from the product photo and your poses (`config.ELECTRONICS`). Measure it with menu 8. Teaching a home pose by hand also lowers the box if you held the arm somewhere the estimate said was taken, because you held it there, so there's room.
- **Every move is simulated before it runs** (`robot/safety.py`): every 2°, the whole arm against everything. If moving all joints at once isn't safe, it tries one joint at a time in every order, then going via upright, ready or shoulder-first. As a last resort it searches joint space (RRT-connect, `SEARCH_TIME_S`).
- **Straight lines** (going down to pieces) keep every pose clear too, and never flip the elbow halfway (2.4.2).
- **Smooth motion** (`robot/motion.py`, `SMOOTH_MOTION`): joints move together on a speed profile (40°/s, gentle start and stop). A new target goes out every 0.1 s and every one is **read back**. If a joint stops following while its commands keep going, something is in the way: it **stops and holds still** until you choose c (2.6, below). Set `SMOOTH_MOTION = False` to go back to the 2.4 start-stop sub-steps.
- **Readings past the scale** (2.4.3): Yahboom's `Arm_serial_servo_read` says nothing outside 0–180°, so the program reads the same register itself, up to 20° past. It never uses `Arm_serial_servo_read_any`, which forgets that servos 2–4 are flipped: a wrist at −6° came back as 186°, the likely cause of the 24 Oct crash.
- **Impossible readings** (part of the arm under the table or deep inside the electronics) mean a servo is misreporting: it won't move.
- **Power comes on aimed at where the arm already is.** A joint that can't report means a countdown, power off so you can move it, then power on by itself (long beep).
- **Only one program drives the arm.** It stops Yahboom's ROS driver and other copies of itself at start, and (2.6) holds a lock while it runs.
- **Ctrl-C never moves the arm.** q quits without moving; 0 rests first.
- Every command and read-back is in `logs/motion/`, and everything printed is in `logs/session/`. Send me the newest file if something goes wrong.

## Motor control (v2.6, from Microduck)

Pollen Robotics' Microduck (shown at NVIDIA GTC Berlin) publishes all its robot code. The motor rules it follows apply to our arm too, so 2.6 adopts them. The full list and reasons are in `docs/design.md`.

- **One safety gate.** Every command, from any part of the program, is checked just before it's sent. It refuses angles that aren't numbers, angles past the servo's range, jumps of more than 15° in one command, and poses closer to the electronics, base or table than where the arm was found. A refused command sends nothing.
- **A loop that keeps time.** Targets follow the clock, not a tick count, so slow I²C reads don't make moves run in slow motion. A late tick is skipped, never made up with a burst.
- **Missed reads pause the move** for a tick. Only 3 in a row stop it, and then it names the servo.
- **Better stall detection.** A slow or loaded servo lags and sags but keeps moving. A blocked one stops while its commands go on, and that's what stops the arm now. The few degrees a joint already sagged before the move don't count. Between moves it waits for the joints to come to rest (`SETTLE_CLOSE_DEG`, `SETTLE_STILL_DEG`).
- **One program on the arm** (`LOCK_FILE`). A second copy says who has it and doesn't touch the bus.
- **Servos that behave like servos in `--sim`** (`tests/fakes/servo_model.py`): command delay, top speed, gear play, sag under load, noisy whole-degree readings and missed reads. `tests/test_motion.py` runs a full tour against 12 random servo sets and the worst case. That's Microduck's "domain randomisation": if it works for all of them, it doesn't depend on one lucky guess about our servos.
- **The arm model as a URDF** (`python3 -m viz.urdf`), checked pose by pose against the model the safety checks use (`tests/test_urdf.py`). It's for RViz or MuJoCo later, and hasn't been tried in RViz yet. `urdf.joint_values(angles)` turns servo angles into the joint values RViz expects.

### Things to measure when you're back at the arm

- **The electronics box:** menu 8, or its x/y/z ranges in `config.ELECTRONICS`.
- **The gripper length:** wrist servo axis to where the closed fingers meet (`config.GRIPPER_LEN`, now 76 mm, approximate). Hand-taught heights read ~20 mm higher than the old calibration, and this is the likely reason.
- **The camera bar's position** on the wrist (`CAMERA_ALONG_MM`, `CAMERA_UP_MM`).
- **How your servos really move:** after a few moves, send me the newest `logs/motion/` file. Sent vs read-back shows the real delay and sag, and I'll narrow `servo_model.RANGES` to match.

## No-hardware checks (laptop, PC or Jetson)

```bash
for t in geometry safety arm teach world motion urdf; do python3 tests/test_$t.py || break; done
```

- `test_world.py` (v2.5) covers the arm's shape against the electronics, base and table. It checks your fold, your 190/0/0 rest and the 190/180/180 crash pose. It re-checks planned moves every 0.5°, runs the search around an obstacle, and covers smooth motion, stopping on an unknown obstacle, your board (from the squares you taught), keyboard control, the electronics model learning from a held pose, and that the 3D page draws the arm exactly where Python puts it.
- `test_teach.py` covers hands-free teaching with a pretend hand and clock, and the whole menu.
- `test_arm.py` covers start-up, misreading servos, unknown obstacles and power-on in place.
- `test_motion.py` (2.6) covers the servo model, the tour against random servo sets, stopping within 0.1 s of touching something the model doesn't know about, the gate catching a planner bug before the box, the loop surviving missed and slow reads, and the lock.
- `test_urdf.py` (2.6) covers the URDF matching `robot/body.py` in 66 poses.

The simulated arm (`tests/fakes/Arm_Lib.py`) treats the board, table, electronics and base as solid, and stalls on contact like a real servo. With the servo model on (always in `--sim`), its servos also take time, sag and miss the odd read. `run.py --sim` uses it, with its own `sim/` folder, so it never touches the real calibration.

## Layout

| Folder | Job | Status |
|---|---|---|
| `run.py` | the menu; with `sessions/`, the only code that connects arm, camera and game | working |
| `config.py` | every fixed number | working |
| `robot/` | moves things: `kinematics.py`, `body.py` (arm shape), `world.py` (what's around it), `safety.py` (simulation + planning), `motion.py` (speed profiles), `arm.py`, `board_frame.py` | working |
| `sessions/` | things the robot does with you: `teach.py`, `look_pose.py` (home pose), `electronics.py`, `jog.py` (keyboard), `self_test.py`, `play.py` | all working except play (M6) |
| `viz/` | the 3D view: `viewer.html`, `live.py` (served while run.py runs), `export.py`, `scene.py`; `urdf.py` (the model for RViz/MuJoCo) | working |
| `docs/` | `design.md`: the rules the arm code keeps, and why | |
| `vision/` | sees things: `camera.py` working; `claw_check.py`, `board_finder.py`, `board_view.py`, `square_cnn.py`, `classifier.py` to build | partly |
| `game/` | thinks: `state.py` (python-chess), `engine.py` (Stockfish) | milestone M5 |
| `learn/` | the CNN workshop: collect, make squares, dataset, train, evaluate | milestones M3–M4 |
| `calibration/` | machine-written: `board_arm.json`, `look_pose.json` (home), `electronics.json` | written by options 1, 7, 8 |
| `data/`, `logs/`, `sim/` | photos, reports, session logs, the simulated run's own folders (git-ignored) | written by sessions |
| `tests/` | no-hardware checks | working |

**Rule:** `robot/`, `vision/` and `game/` never import each other. That keeps vision testable on saved photos and chess logic testable without the arm.

## Coordinates

Standing behind the arm, looking at the board: X = right, Y = toward the board, Z = up, in mm. The origin is the centre of the turning base, z = 0 on top of the base plate. The table and board are ~15 mm lower, and the electronics are at negative Y, behind the arm.
