# How the arm code is built, and why (v2.6)

These are the rules the code keeps. If you change something and a rule breaks, a test in `tests/` should fail. If none does, add one.

## The rules

1. **One program drives the arm.** `robot/arm.py` takes a lock (`/tmp/dofbot_arm.lock`) before it opens the bus. A second copy gets a clear "another program is driving the arm (pid …)" and touches nothing. `run.py` also stops Yahboom's ROS arm driver at start.
2. **Nothing moves at start.** Power comes on aimed at where each joint already is. The first move is one you can see coming: a countdown, then home.
3. **Every command goes through one gate** (`Arm._gate`), whoever sent it: the planner, keyboard control, teaching, a test. It refuses:
   - an angle that isn't a number;
   - an angle past what the servo can physically do (`JOINT_RANGE`), unless it's coming back in from there;
   - a jump of more than `GATE_STEP_DEG` (15°) from the last command on servos 1–4 (moves go in steps of a few degrees, so a big jump is a bug);
   - a pose that puts any part of the arm too close to the electronics, base or table. A pose that is closer than where the arm was *found* is refused; one that's no closer is allowed, so the arm can always back out.

   A refused command sends nothing and is counted (`health["refused"]`).
4. **Every move is simulated before it's sent** (`robot/safety.py`): the whole arm, every 2°, against everything. The gate is the second check, not the only one.
5. **The motor loop keeps time** (`Arm._stream`). Every `CONTROL_DT` (0.1 s) the target is where the speed profile says the joints should be *at that moment*. A late tick doesn't make the move run in slow motion. A tick that's very late is skipped, not made up for with a burst.
6. **A missed read pauses, it doesn't panic.** If a servo doesn't answer, the move holds that target for a tick. Three misses in a row (`READ_MISS_LIMIT`) means hold still and say which servo went quiet.
7. **A joint that stops following its commands means something is in the way** (`motion.StallWatch`). Being a few degrees behind isn't enough, because a slow or loaded servo always is. What counts is the commands moving on while the joint doesn't. The sag a joint already had when the move began doesn't count either. Then it holds still right there (`ArmBlocked`) until you choose **c**.
8. **Ctrl-C never moves the arm.** Quitting with q never moves it either.
9. **Every command and read-back is logged** (`logs/motion/`), and the loop's health (rate, late ticks, missed reads, refused commands) is printed on quit and shown in the 3D view.

## Lessons from Microduck

Microduck is Pollen Robotics' and Hugging Face's open-source walking robot, shown in Jensen Huang's NVIDIA GTC Berlin keynote (October 2026). Its code is public: `pollen-robotics/microduck` (the robot's software) and `pollen-robotics/microduck_rl` (training in simulation). It's a different robot, a walker trained with reinforcement learning, but the way it treats motors applies to any arm. What we took:

| Microduck | Here |
|---|---|
| One daemon owns the motor bus, with a lock file | `take_bus_lock()` in `robot/arm.py` |
| "Safety owns the only write handle": every command is checked (finite, in range) before it reaches a motor | `Arm._gate`, used by `set_servo` and `move_servos` |
| Never moves on start; holds the pose it found | power-on in place (since 2.4), hold pose taken from the reading |
| Fixed-rate loop: on overrun, skip the tick, don't burst | `_stream` keeps a deadline and skips |
| A failed read keeps the last good sample and is counted | pause one tick, `READ_MISS_LIMIT`, `health["read_misses"]` |
| Health published while running | `arm.health`, `health_line()`, the 3D view's motor-loop chip |
| "Actuator fidelity is most of the sim-to-real gap": an actuator model, with delay, friction and backlash, and an encoder on the output side | `tests/fakes/servo_model.py`: delay, top speed, gear play, sag under load, noisy whole-degree readings, missed reads |
| Domain randomisation: train against a range of motors, not one | `tests/test_motion.py` runs the same tour against 12 random servo sets plus the worst case |
| A fake hardware layer so the whole stack runs without the robot | `tests/fakes/Arm_Lib.py`, `run.py --sim` |
| One robot description shared by the simulator and the robot (MJCF from the CAD) | `config.py` → `robot/body.py`, exported as `dofbot_chess.urdf` (`viz/urdf.py`) and checked pose by pose (`tests/test_urdf.py`) |
| Policy exported to ONNX with its preprocessing baked in, a manifest next to it, checked on load | the plan for the square classifier (milestone M4), below |

What we didn't take: their reinforcement-learning walking controller. Our arm's moves are planned (inverse kinematics plus collision checks), and that's the right tool for picking up pieces.

## For the CNN (M3–M4)

Same idea as Microduck's policy files. Export the trained classifier to **ONNX** with the image preprocessing (resize, normalise) inside the model. Next to it, write a small **manifest**: the class list in order, input size, colour order, training date, accuracy on the held-out squares. At start-up, load it with OpenCV's `dnn` module (already on the Jetson), check it against the manifest, and run it on one saved test square with a known answer. Then a model trained on the PC can't silently disagree with the one on the Jetson.

## Where the numbers come from

- Lengths: Yahboom's URDF (`config.py`, top).
- Thicknesses, camera bar, electronics box: the product photo and your poses. These are estimates, so measure them (README, *Things to measure*).
- Servo model ranges: typical for 15 kg bus servos, not measured. When you're back at the arm, the motion logs (`logs/motion/`) show the real delay and sag, and the ranges in `servo_model.RANGES` can be narrowed to match.
