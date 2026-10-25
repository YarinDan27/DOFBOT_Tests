"""
A pretend Arm_Lib for tests -- no hardware, nothing moves.

It behaves like the real arm in the ways that matter for safety:
  * servos move to where they're sent ... unless the arm would hit the board
    or the table on the way: then that servo STOPS at the point of contact,
    exactly like a real servo pushing against the board
  * readings come back as whole degrees with a little noise; None outside
    0-180 (servo 5: 0-270), like the real library
  * READ_BIAS makes a servo report the wrong angle (the suspected cause of
    the 2026-10-24 crash); SILENT makes a servo never answer
  * with power off, servos stay wherever the test puts them (set_pose)
"""
import random

from robot import safety

STATE = {"pos": {i: 90.0 for i in range(1, 7)}, "torque": True, "contacts": 0,
         "beeps": [], "lights": []}   # beeps: 100 ms units per beep; lights: (r, g, b)
READ_BIAS = {}          # servo id -> degrees added to every reading
SILENT = set()          # servo ids that never answer
BOARD = {"z": -13.0, "x": (-120.0, 120.0), "y": (90.0, 290.0)}   # the real board
TABLE_Z = -16.0


def reset(pose=(90, 90, 90, 90, 90, 150)):
    set_pose(pose)
    STATE["torque"] = True
    STATE["contacts"] = 0
    STATE["beeps"] = []
    STATE["lights"] = []
    READ_BIAS.clear()
    SILENT.clear()


def set_pose(pose):
    for i in range(1, 7):
        STATE["pos"][i] = float(pose[i - 1])


def touching(angles):
    """Would any part of the arm be inside the board or the table?"""
    for x, y, z in safety.link_points(angles):
        if z < TABLE_Z:
            return True
        if (z < BOARD["z"] and BOARD["x"][0] <= x <= BOARD["x"][1]
                and BOARD["y"][0] <= y <= BOARD["y"][1]):
            return True
    return False


class Arm_Device:
    def Arm_serial_set_torque(self, onoff):
        STATE["torque"] = bool(onoff)

    def Arm_serial_servo_write(self, servo_id, angle, duration_ms):
        if not STATE["torque"]:
            return
        if servo_id > 4:
            STATE["pos"][servo_id] = float(angle)
            return
        start = STATE["pos"][servo_id]
        angles = [STATE["pos"][i] for i in range(1, 5)]
        steps = 40
        for k in range(1, steps + 1):
            trial = list(angles)
            trial[servo_id - 1] = start + (float(angle) - start) * k / steps
            if touching(trial) and not touching(angles):
                STATE["contacts"] += 1
                return                      # stalled against the board
            angles = trial
        STATE["pos"][servo_id] = float(angle)

    def Arm_serial_servo_read(self, servo_id):
        if servo_id in SILENT:
            return None
        v = int(round(STATE["pos"][servo_id] + READ_BIAS.get(servo_id, 0.0)
                      + random.gauss(0, 0.3)))
        top = 270 if servo_id == 5 else 180
        return v if 0 <= v <= top else None

    def Arm_Buzzer_On(self, delay=0xff):
        STATE["beeps"].append(delay)

    def Arm_Buzzer_Off(self):
        pass

    def Arm_RGB_set(self, red, green, blue):
        STATE["lights"].append((red, green, blue))
