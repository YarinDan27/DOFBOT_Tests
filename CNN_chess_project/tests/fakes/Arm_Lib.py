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
  * (v2.5) the electronics box and turning base are solid too
  * (v2.6) use_servo_model() makes the servos move like real ones, over time:
    command delay, the move spread over its duration, top speed, gear play,
    sag under load, noisy whole-degree readings and the odd missed read
    (servo_model.py, after Microduck's actuator models). Off by default:
    then a servo jumps to its target the moment it's told.
  * run.py --sim uses this file as the arm, so you can try the program without one
"""
import random
import time

import config
from robot import body, safety

STATE = {"pos": {i: 90.0 for i in range(1, 7)}, "torque": True, "contacts": 0,
         "beeps": [], "lights": []}   # beeps: 100 ms units per beep; lights: (r, g, b)
READ_BIAS = {}          # servo id -> degrees added to every reading
SILENT = set()          # servo ids that never answer
BOARD = {"z": -13.0, "x": (-120.0, 120.0), "y": (90.0, 290.0)}   # the real board
TABLE_Z = -16.0
MODEL = {"servos": None}  # servo_model.ServoModel, or None = servos teleport


def use_servo_model(params=None, seed=0):
    """Real-servo behaviour from now on (None: back to teleporting servos)."""
    from servo_model import ServoModel
    MODEL["servos"] = ServoModel(params, seed) if params is not False else None
    return MODEL["servos"]


def _advance():
    """Moves the joints to where the servos have got to by now (servo model only)."""
    m = MODEL["servos"]
    if m is None or not STATE["torque"]:
        return
    now = time.time()
    old = [STATE["pos"][i] for i in range(1, 5)]
    for i in range(1, 5):
        if not m.known(i):          # power just came on here: the motor is where the joint is
            m.hold(i, old[i - 1])
    motor = [m.motor(i, old[i - 1], now) for i in range(1, 5)]
    droop = m.droop(motor)
    new = [motor[i] + droop[i] for i in range(4)]
    if max(abs(new[i] - old[i]) for i in range(4)) < 1e-9:
        return
    steps = 40
    current = list(old)
    for k in range(1, steps + 1):
        trial = [old[i] + (new[i] - old[i]) * k / steps for i in range(4)]
        if touching(trial) and not touching(current):
            STATE["contacts"] += 1          # stalled: pushing against something
            STATE.setdefault("first_contact_t", now)
            STATE.setdefault("first_contact", (now, list(trial), _what_touches(trial)))
            break
        current = trial
    for i in range(4):
        STATE["pos"][i + 1] = current[i]


def _reading(servo_id):
    """The angle a read reports: the real angle + bias + noise (None = no answer)."""
    _advance()
    true = STATE["pos"][servo_id] + READ_BIAS.get(servo_id, 0.0)
    m = MODEL["servos"]
    if m is not None:
        return m.read(true)
    return true + random.gauss(0, 0.3)


def reset(pose=(90, 90, 90, 90, 90, 150)):
    set_pose(pose)
    STATE["torque"] = True
    STATE["contacts"] = 0
    STATE.pop("first_contact_t", None)
    STATE.pop("first_contact", None)
    STATE["beeps"] = []
    STATE["lights"] = []
    READ_BIAS.clear()
    SILENT.clear()


def set_pose(pose):
    if MODEL["servos"] is not None:
        for i in range(1, 7):
            MODEL["servos"].hold(i, float(pose[i - 1]))
    for i in range(1, 7):
        STATE["pos"][i] = float(pose[i - 1])


# The real electronics box and turning base (the program only has a model of
# them; here they are "real": a link's centre line entering one is a contact).
ELECTRONICS = dict(config.ELECTRONICS)


def _inside_solid(link, p):
    e = ELECTRONICS
    if (e["x"][0] <= p[0] <= e["x"][1] and e["y"][0] <= p[1] <= e["y"][1]
            and e["z"][0] <= p[2] <= e["z"][1]):
        return True
    if link != "upper arm":
        for part in config.TURRET:
            if (part["z"][0] <= p[2] <= part["z"][1]
                    and (p[0] ** 2 + p[1] ** 2) ** 0.5 <= part["radius"]):
                return True
    return False


def _what_touches(angles):
    for x, y, z in safety.link_points(angles):
        if z < TABLE_Z:
            return "table at (%.0f, %.0f, %.0f)" % (x, y, z)
        if (z < BOARD["z"] and BOARD["x"][0] <= x <= BOARD["x"][1]
                and BOARD["y"][0] <= y <= BOARD["y"][1]):
            return "board at (%.0f, %.0f, %.0f)" % (x, y, z)
    for link, p, _ in body.sample_points(angles, 8.0):
        if _inside_solid(link, p):
            return "%s in a solid at (%.0f, %.0f, %.0f)" % ((link,) + tuple(p))
    return None


def touching(angles):
    """Would any part of the arm be inside the board, the table, the electronics or the base?"""
    for x, y, z in safety.link_points(angles):
        if z < TABLE_Z:
            return True
        if (z < BOARD["z"] and BOARD["x"][0] <= x <= BOARD["x"][1]
                and BOARD["y"][0] <= y <= BOARD["y"][1]):
            return True
    for link, p, _ in body.sample_points(angles, 8.0):
        if _inside_solid(link, p):
            return True
    return False


def _raw(servo_id, angle):
    """The servo's own number for an angle, the way the real board reports it."""
    if servo_id in (2, 3, 4):
        angle = 180.0 - angle
    if servo_id == 5:
        return int((3700 - 380) * angle / 270.0 + 380)
    return int((3100 - 900) * angle / 180.0 + 900)


class FakeSMBus:
    """The I2C bus underneath: what Arm_Lib (and robot/arm.py's wide read) talk to."""

    def __init__(self):
        self.asked = None

    def write_byte_data(self, addr, register, value):
        self.asked = register

    def read_word_data(self, addr, register):
        servo_id = register - 0x30
        if servo_id in SILENT or not 1 <= servo_id <= 6:
            return 0
        angle = _reading(servo_id)
        if angle is None:
            return 0
        pos = max(1, _raw(servo_id, angle))
        return (pos >> 8 & 0xff) | (pos << 8 & 0xff00)     # bytes swapped, like the real one


class Arm_Device:
    def __init__(self):
        self.addr = 0x15
        self.bus = FakeSMBus()

    def Arm_serial_set_torque(self, onoff):
        STATE["torque"] = bool(onoff)

    def Arm_serial_servo_write(self, servo_id, angle, duration_ms):
        if not STATE["torque"]:
            return
        if servo_id > 4:
            STATE["pos"][servo_id] = float(angle)
            return
        m = MODEL["servos"]
        if m is not None:
            _advance()
            now = time.time()
            m.command(servo_id, m.motor(servo_id, STATE["pos"][servo_id], now), float(angle),
                      duration_ms, now)
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
        angle = _reading(servo_id)
        if angle is None:
            return None
        v = int(round(angle))
        top = 270 if servo_id == 5 else 180
        return v if 0 <= v <= top else None

    def Arm_Buzzer_On(self, delay=0xff):
        STATE["beeps"].append(delay)

    def Arm_Buzzer_Off(self):
        pass

    def Arm_RGB_set(self, red, green, blue):
        STATE["lights"].append((red, green, blue))
