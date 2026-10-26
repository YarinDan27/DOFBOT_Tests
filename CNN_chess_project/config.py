#!/usr/bin/env python3
"""
Every fixed number lives here.

Nothing about WHERE the board is: that is taught by hand (sessions/teach.py)
and saved to calibration/board_arm.json.
"""
import os

VERSION = "2.4.3"
VERSION_NOTE = ("stops Yahboom's driver itself; joints a little past the end of their "
                "scale now report (the folded pose); if one still can't, power goes off "
                "after a countdown so you can move it")

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
JOINT_MS_PER_DEG = 22      # joint move speed: ~2 s for a 90 degree turn (slow on purpose)
JOINT_MIN_MS = 110         # shortest sub-step
JOINT_SUBSTEP_DEG = 5.0    # joint moves are cut into sub-steps this big, each checked

# ---------- read-back checks (robot/arm.py) ----------
STALL_DEG = 6.0            # a servo this far short of where it was sent = blocked: stop
IMPOSSIBLE_BELOW_MM = 40.0 # readings that put the arm this far under the board = a servo
                           # is misreporting: don't move at all
LIMIT_SLACK_DEG = 3.0      # a reading this far past a limit is still trusted (and nudged
                           # back onto it); further than that = don't power on or move
WIDE_READ_DEG = 20.0       # a joint up to this far past the end of its 0-180 scale still
                           # reports (Arm_Lib alone says nothing past the end)

# ---------- motion planning ----------
HOVER_MM = 45.0            # how high above a piece to hover before descending
TRAVEL_MM = 80.0           # height above the board used for travelling across it
STEP_MM = 10.0             # straight-line moves are cut into steps this long
STEP_MS = 140              # milliseconds allowed per step
LINE_MAX_TURN_DEG = 6.0    # no joint turns more than this in one 10 mm step (no
                           # sudden elbow flip halfway along a line)
LIFT_STEP_MM = 5.0         # rising straight up from wherever: smaller steps, so
                           # even a stretched-out arm turns only a few degrees each
APPROACH_STRAIGHT = 180.0  # hand pointing straight down
APPROACH_MIN = 120.0       # most we'll let the hand tilt to reach far squares

# ---------- teaching by hand (sessions/teach.py) ----------
# Spread out, all inside the reach ring. 4+ squares lets the program check
# your touches. Avoid the far corners (a8/h8) and the squares right in front
# of the base: pushing the claw there bends a joint past its range, and a
# servo past its range can't report its angle.
TEACH_SQUARES = ["b3", "g3", "b6", "g6", "e5"]
TEACH_READS = 5             # servo readings averaged per taught square
TEACH_MAX_RESIDUAL_MM = 4.0 # a touch further than this from the fitted board is probably off

# Hands-free at the arm: you put the CLOSED claw tip on top of a piece and hold still.
TEACH_COUNTDOWN_S = 15      # time to walk from the computer to the arm before power goes off
TEACH_TICK_S = 0.25         # how often it reads the arm while you teach
STILL_S = 2.0               # hold the claw still this long to record a square
STILL_MM = 8.0              # "still" = the tip stays within this of one spot (a
                            # 1-degree reading wobble alone moves it ~4 mm)
AUTO_MIN_MOVE_MM = 20.0     # after a record, move at least this far before the next one
AUTO_MAX_Z_MM = 150.0       # higher than this = you've lifted the claw away, not teaching
PAUSE_HOLD_S = 4.0          # lift high and hold still this long = stop teaching (power on)
TEACH_IDLE_S = 120.0        # nothing moves for this long = you left: stop, power on in place
FINISH_POWER_ON_S = 20.0    # at the end, wait up to this long for every joint to report
LOOK_HOLD_S = 3.0           # look-at-board view: hold still this long to save it
TEACH_START_SQUARE = "e5"   # before power goes off, the arm comes down low over this square...
TEACH_START_ABOVE_MM = 40.0 # ...this far above it, so it can't fall far
TEACH_START_XYZ = (0.0, 190.0, 90.0)   # (same, before any board is known)
GRAB_BELOW_TOP_MM = 15.0    # taught heights are piece TOPS; grabbing goes this far below

# ---------- self-test (sessions/self_test.py) ----------
SETTLE_S = 0.6              # wait after arriving before measuring
SELF_TEST_GOOD_MM = 2.0     # within this = confident
SELF_TEST_OK_MM = 5.0       # within this = usable; beyond = not trusted
QUICK_SQUARES = ["a1", "d1", "h1", "a4", "e4", "h4", "a6", "d6", "h6",
                 "b8", "g8"]

# ---------- camera ----------
CAMERA_INDEX = 0           # tried first; every other /dev/video* is tried after it
CAMERA_FREE_DEVICE = True  # on start, stop other programs holding the camera (fuser)
CAMERA_RELEASE_S = 2.0     # wait after freeing it (and between tries) for the device to let go
WEB_PORT = 8080
CAM_WIDTH = 640     # smaller frames compress faster; try 320 if it's still slow
CAM_HEIGHT = 480
JPEG_QUALITY = 50   # stream quality, 1-100. lower = blurrier but faster
PHOTO_QUALITY = 95  # saved photos are training data: keep them sharp
