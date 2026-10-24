#!/usr/bin/env python3
"""
Every fixed number lives here.

Nothing about WHERE the board is: that is taught by hand (sessions/teach.py)
and saved to calibration/board_arm.json.
"""
import os

VERSION = "0.2.0"

# ---------- folders ----------
ROOT = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIR = os.path.join(ROOT, "calibration")
DATA_DIR = os.path.join(ROOT, "data")      # photos (git-ignored)
LOGS_DIR = os.path.join(ROOT, "logs")      # session reports (git-ignored)
BOARD_FILE = os.path.join(CALIBRATION_DIR, "board_arm.json")
LOOK_POSE_FILE = os.path.join(CALIBRATION_DIR, "look_pose.json")

# ---------- arm hardware (from the manufacturer's URDF; never changes) ----------
H = 107.5          # table surface to shoulder pivot (servo 2), mm
L1 = 82.85         # servo 2 -> servo 3
L2 = 82.85         # servo 3 -> servo 4
L3_WRIST = 73.85   # servo 4 -> servo 5
GRIPPER_LEN = 76.0 # servo 5 -> where the fingers close (measured, approximate)
L3 = L3_WRIST + GRIPPER_LEN

# ---------- servo behaviour (verified on the real arm) ----------
# All servos at 90 = arm straight up.
# Servo 1 up = turns left. Servos 2,3,4 up = lean AWAY from the board.
CENTER = 90.0

# Safety limits. "Away" is the dangerous side: the arm hits the Jetson
# electronics behind it. Tighten these if it ever gets close.
SERVO_LIMITS = {
    1: (10.0, 170.0),
    2: (10.0, 140.0),
    3: (10.0, 140.0),
    4: (10.0, 170.0),
    5: (0.0, 180.0),
    6: (0.0, 180.0),
}

# ---------- speeds (bigger = faster) ----------
SPEED_SLOW = 300
SPEED_NORMAL = 600
SPEED_HOME = 1200

# ---------- gripper ----------
CLAW_OPEN = 150
CLAW_MAX = 175
CLAW_STEP = 3
CLAW_STEP_SPEED = 200
CLAW_STALL_GAP = 12

# ---------- poses ----------
# Straight up: the safe place to park between sessions.
POSE_REST = [90, 90, 90, 90, 90, CLAW_OPEN]
# Hovering high over the board. Straight-line moves have to start from a pose
# the "gripper pointing down" math can express; standing upright is not one.
POSE_READY = [90, 90, 25, 25, 90, CLAW_OPEN]

# ---------- collision check for joint moves (robot/safety.py) ----------
CLEAR_MM = 60.0            # over the board, every part of the arm stays this high
TABLE_CLEAR_MM = 5.0       # anywhere else, this high above the board surface
ZONE_MARGIN_MM = 25.0      # the "over the board" area reaches this far past its edge
NUDGE_MM = 15.0            # a part already low may still move this much (letting go)
SIM_SAMPLES = 20           # points checked along each simulated joint move
CONTINUOUS_DEG = 8.0       # bigger first step than this = a swing: check it
LINE_TOL_MM = 5.0          # rising straight up: the claw may wander this far off the line
# Before the board is taught: assume it could be anywhere in this area
# (x from, x to, y from, y to, mm), at the height measured in the old project.
DEFAULT_ZONE = (-170.0, 170.0, 70.0, 330.0)
DEFAULT_BOARD_Z = -13.0
JOINT_MS_PER_DEG = 14      # joint move speed: ~1.3 s for a 90 degree turn
JOINT_MIN_MS = 400

# ---------- motion planning ----------
HOVER_MM = 45.0            # how high above a piece to hover before descending
TRAVEL_MM = 80.0           # height above the board used for travelling across it
STEP_MM = 10.0             # straight-line moves are cut into steps this long
STEP_MS = 140              # milliseconds allowed per step
LIFT_STEP_MM = 5.0         # rising straight up from wherever: smaller steps, so
                           # even a stretched-out arm turns only a few degrees each
APPROACH_STRAIGHT = 180.0  # hand pointing straight down
APPROACH_MIN = 120.0       # most we'll let the hand tilt to reach far squares

# ---------- teaching by hand (sessions/teach.py) ----------
# Spread out, all inside the reach ring. 4+ squares lets the program check
# your touches: with exactly 3 the fit is perfect by definition.
TEACH_SQUARES = ["b2", "g2", "b6", "g6", "e4"]
TEACH_READS = 5             # servo readings averaged per taught square
TEACH_MAX_RESIDUAL_MM = 4.0 # a touch further than this from the fitted board is probably off

# ---------- self-test (sessions/self_test.py) ----------
SETTLE_S = 0.6              # wait after arriving before measuring
SELF_TEST_GOOD_MM = 2.0     # within this = confident
SELF_TEST_OK_MM = 5.0       # within this = usable; beyond = not trusted
QUICK_SQUARES = ["a1", "d1", "h1", "a4", "e4", "h4", "a6", "d6", "h6",
                 "b8", "g8"]

# ---------- camera ----------
CAMERA_INDEX = 0
WEB_PORT = 8080
CAM_WIDTH = 640     # smaller frames compress faster; try 320 if it's still slow
CAM_HEIGHT = 480
JPEG_QUALITY = 50   # stream quality, 1-100. lower = blurrier but faster
PHOTO_QUALITY = 95  # saved photos are training data: keep them sharp
