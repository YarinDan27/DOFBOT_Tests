#!/usr/bin/env python3
"""
Teach the look-at-board view -- hands-free at the arm.

This is where the arm waits ("home"), with the camera seeing the whole board.
The arm comes back here after every job.

At the arm: after the long beep (power off), hold the arm up HIGH with the
camera looking down at the whole board, and hold still for LOOK_HOLD_S
seconds. Blue light = this pose is usable, keep still; red = not usable
(too low over the board, or a joint past its range) -- move it.
Long beep = saved, and the arm holds itself there.

Saved to calibration/look_pose.json, plus a photo of the view in data/look/
so you can check it afterwards.
"""
import json
import math
import os
import time

import config
from robot import kinematics, safety
from robot.arm import ArmBlocked, FIX_BY_HAND, SERVO_NAMES, pose_problem


def load():
    """The saved pose (6 servo angles), or None if there isn't a usable one."""
    if not os.path.exists(config.LOOK_POSE_FILE):
        return None
    try:
        with open(config.LOOK_POSE_FILE) as fh:
            pose = [float(a) for a in json.load(fh)["servos"]]
    except (ValueError, KeyError, TypeError):
        print("  look_pose.json is unreadable -- ignoring it.")
        return None
    if len(pose) != 6 or _outside_limits(pose):
        print("  look_pose.json is outside the servo limits -- ignoring it.")
        return None
    return pose


def _outside_limits(angles):
    for servo_id, angle in enumerate(angles, start=1):
        low, high = config.SERVO_LIMITS[servo_id]
        if not (low <= angle <= high):
            return "servo %d is at %.0f, outside its safe range %.0f-%.0f" % (
                servo_id, angle, low, high)
    return None


def _problem(angles, zone):
    """Why this pose can't be the look pose, or None if it's fine."""
    outside = _outside_limits(angles)
    if outside:
        return outside + ". Move it back a little."
    if not safety.is_clear(angles, zone):
        return ("Too low: every part of the arm must stay %.0f mm above the board. "
                "Raise it." % config.CLEAR_MM)
    return None


def check_pose(angles, zone):
    """Why these angles can't be the look pose, or None if they can."""
    return pose_problem(angles) or _problem(angles, zone)


def save_pose(pose, camera=None):
    """Writes calibration/look_pose.json (+ a photo of the view). Returns the photo path."""
    photo = None
    if camera is not None:
        session = time.strftime("%Y%m%d-%H%M%S")
        path = camera.save_photo(os.path.join(config.DATA_DIR, "look", session + ".jpg"))
        if path:
            photo = os.path.relpath(path, config.ROOT)
    if not os.path.isdir(config.CALIBRATION_DIR):
        os.makedirs(config.CALIBRATION_DIR)
    x, y, z, _ = kinematics.forward(*pose[:4])
    with open(config.LOOK_POSE_FILE, "w") as fh:
        json.dump({"servos": [round(float(a), 1) for a in pose],
                   "claw_tip_mm": [round(x, 1), round(y, 1), round(z, 1)],
                   "photo": photo,
                   "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, fh, indent=2)
    return photo


def run(arm, camera=None):
    """Hands-free. Returns the saved pose, or None."""
    from sessions.teach import BLUE, OFF, RED, _Light, _Stillness, _end_line, _show, countdown

    print("\n--- Teach the look-at-board view (hands-free at the arm) ---")
    print("At the arm:")
    print("  * wait for the LONG beep -- be holding the arm by then (power goes off)")
    print("  * move it up HIGH, camera looking down at the whole board, and hold still %.0f s"
          % config.LOOK_HOLD_S)
    print("  * blue light = usable, keep still; red = not usable here (too low, or a joint")
    print("    past its range) -- move it")
    print("  * long beep = saved, the arm holds itself there")
    if camera is None:
        print("  (camera off: no photo of the view this time)")
    countdown(arm, config.TEACH_COUNTDOWN_S)

    still = _Stillness()
    light = _Light(arm)
    screen = {}
    last_motion = time.time()
    pose = None
    # It only saves once you've moved the arm away from where power went off
    # (so a pause before you start isn't saved as the view).
    first = arm.read_angles(samples=1, ids=(1, 2, 3, 4))
    start_tip = (None if any(a is None for a in first[:4])
                 else list(kinematics.forward(*first[:4])[:3]))
    while True:
        now = time.time()
        angles = arm.read_angles(samples=1)
        silent = [s for s in (1, 2, 3, 4) if angles[s - 1] is None]
        if silent:
            light.set(RED)
            still.reset()
            _show("  %s can't report -- %s" % (SERVO_NAMES[silent[0]], FIX_BY_HAND[silent[0]]),
                  screen)
        else:
            candidate = list(angles)
            if candidate[4] is None:
                candidate[4] = config.CENTER
            candidate[5] = config.CLAW_OPEN
            problem = check_pose(candidate, arm.zone)
            tip = list(kinematics.forward(*angles[:4])[:3])
            held = still.update(tip, now)
            if held == 0.0:
                last_motion = now
            if start_tip is not None and math.sqrt(sum(
                    (tip[k] - start_tip[k]) ** 2 for k in range(3))) < config.AUTO_MIN_MOVE_MM:
                light.set(BLUE)
                _show("  move the arm to where the camera sees the whole board", screen)
            elif problem:
                light.set(RED)
                _show("  not usable: %s" % problem, screen)
            else:
                light.set(BLUE)
                _show("  usable -- hold still %.1f/%.0f s" % (min(held, config.LOOK_HOLD_S),
                                                             config.LOOK_HOLD_S), screen)
                if held >= config.LOOK_HOLD_S:
                    pose = [round(a, 1) for a in candidate]
                    break
        if now - last_motion > config.TEACH_IDLE_S:
            _end_line(screen)
            print("  Nothing moved for %.0f s -- stopping, nothing saved." % config.TEACH_IDLE_S)
            break
        time.sleep(config.TEACH_TICK_S)
    _end_line(screen)
    light.set(BLUE)
    try:
        powered = arm.power_on_when_readable(timeout=config.FINISH_POWER_ON_S, show=True)
    except ArmBlocked as e:
        print("  Power is on, but: %s. It's holding still." % e)
        powered = True
    if not powered:
        light.set(RED)
        arm.beep(3)
        print("  Couldn't switch power back on -- the arm is still floppy. Lay it down gently.")
        if pose is not None:
            save_pose(pose, camera)
            arm.home_pose = pose
            print("  (The view was saved anyway.)")
        return pose
    light.set(OFF)
    if pose is None:
        arm.beep(2)
        return None
    photo = save_pose(pose, camera)
    arm.home_pose = pose
    arm.beep(long=True)
    x, y, z, _ = kinematics.forward(*pose[:4])
    print("  Saved the look-at-board view (claw at x=%.0f y=%.0f z=%.0f mm). The arm holds "
          "itself there." % (x, y, z))
    if photo:
        print("  What the camera sees from there: %s" % photo)
    return pose
