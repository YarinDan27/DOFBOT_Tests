#!/usr/bin/env python3
"""
Every fixed number lives here. Nothing about the board's position --
that gets discovered by calibration and saved to board_calibration.json.
"""

VERSION = "1.0.2"

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
# Ready pose: hovering high over the middle of the board. Straight-line moves
# have to start from a pose the "gripper pointing down" math can express, and
# standing perfectly upright is not one of them -- so the arm comes here first.
POSE_READY = [90, 90, 25, 25, 90, CLAW_OPEN]

# ---------- motion planning ----------
HOVER_MM = 45.0            # how high above a piece to hover before descending
TRAVEL_MM = 80.0           # height above the board used for travelling across it
STEP_MM = 10.0             # straight-line moves are cut into steps this long
STEP_MS = 140              # milliseconds allowed per step
APPROACH_STRAIGHT = 180.0  # hand pointing straight down
APPROACH_MIN = 120.0       # most we'll let the hand tilt to reach far squares

# ---------- camera stream ----------
WEB_PORT = 8080
CAM_WIDTH = 640     # smaller frames compress faster; try 320 if it's still slow
CAM_HEIGHT = 480
JPEG_QUALITY = 50   # 1-100. lower = blurrier but much faster
