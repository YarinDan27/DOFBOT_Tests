"""
How a real bus servo moves -- for the simulated arm (Arm_Lib.py next to this).

Borrowed from Microduck (Pollen Robotics): their sim2real notes say actuator
fidelity is most of the gap between simulation and the real robot, so they
model each servo down to its control law and randomise its parameters in
training. Ours is much simpler, but it has the effects that matter for
checking our motion code against a servo that doesn't teleport:

  * delay      the command reaches the servo a little late (I2C -> the
               expansion board -> the servo bus)
  * duration   the servo spreads the move over the time it was given, and
               never goes faster than its top speed
  * backlash   a little gear play: under gravity the output sits at one end
  * sag        a loaded joint droops a little (shoulder most, wrist least)
  * noise      readings are whole degrees, a bit noisy
  * misses     now and then a read gets no answer

Every number can be randomised (randomised()), the way Microduck trains
against a range of servos rather than one perfect one -- if our motion code
works across the whole range, it doesn't depend on one lucky guess.
"""
import math
import random

import config

# typical values for the DOFBOT's 15 kg bus servos (not measured: ranges below)
DEFAULTS = {"delay_s": 0.015, "max_speed": 250.0, "backlash": 1.0, "sag": 1.5,
            "noise": 0.3, "miss_rate": 0.0}
RANGES = {"delay_s": (0.005, 0.05), "max_speed": (90.0, 400.0), "backlash": (0.0, 2.0),
          "sag": (0.0, 4.0), "noise": (0.0, 0.6), "miss_rate": (0.0, 0.03)}
SAG_SHARE = {2: 1.0, 3: 0.6, 4: 0.25}     # how much of the droop each joint gets


def randomised(rng):
    """A servo set drawn from RANGES (domain randomisation)."""
    return {k: rng.uniform(*v) for k, v in RANGES.items()}


class ServoModel:
    def __init__(self, params=None, seed=0):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.rng = random.Random(seed)
        # per servo: where the move starts (time, motor angle), target, how long
        self.cmd = {}

    def command(self, sid, start_angle, target, duration_ms, now):
        speed_time = abs(target - start_angle) / self.p["max_speed"]
        self.cmd[sid] = {"t0": now + self.p["delay_s"], "a0": start_angle, "a1": float(target),
                         "dur": max(duration_ms / 1000.0, speed_time)}

    def motor(self, sid, fallback, now):
        """Where the servo's motor is now (before gear play and sag)."""
        c = self.cmd.get(sid)
        if c is None:
            return fallback
        if now <= c["t0"]:
            return c["a0"]
        f = 1.0 if c["dur"] <= 0 else min(1.0, (now - c["t0"]) / c["dur"])
        return c["a0"] + (c["a1"] - c["a0"]) * f

    def hold(self, sid, angle):
        """The joint was put here (by hand, or at start): the motor stays here until told."""
        self.cmd[sid] = {"t0": float("-inf"), "a0": float(angle), "a1": float(angle), "dur": 0.0}

    def known(self, sid):
        return sid in self.cmd

    def droop(self, angles):
        """
        Servo degrees each joint 1-4 sits off its motor position: gravity
        pulls a joint toward the side the arm hangs out on, through the gear
        play, then a little more under load.
        """
        s2, s3, s4 = angles[1], angles[2], angles[3]
        a1 = math.radians(90.0 - s2)
        a2 = a1 + math.radians(90.0 - s3)
        a3 = a2 + math.radians(90.0 - s4)
        # horizontal lever arm of what each joint holds up, as a fraction of full reach
        reach = config.L1 + config.L2 + config.L3
        lever = {2: (config.L1 * math.sin(a1) + config.L2 * math.sin(a2)
                     + config.L3 * math.sin(a3)) / reach,
                 3: (config.L2 * math.sin(a2) + config.L3 * math.sin(a3)) / reach,
                 4: config.L3 * math.sin(a3) / reach}
        out = [0.0, 0.0, 0.0, 0.0]
        for sid, share in SAG_SHARE.items():
            load = lever[sid]
            # leaning forward (load > 0) = tilt grows = servo value goes DOWN
            side = -1.0 if load > 0 else 1.0
            out[sid - 1] = side * (self.p["backlash"] / 2.0 * min(1.0, abs(load) * 4.0)
                                   + self.p["sag"] * share * abs(load))
        return out

    def read(self, true_angle):
        if self.rng.random() < self.p["miss_rate"]:
            return None
        return true_angle + self.rng.gauss(0.0, self.p["noise"])
