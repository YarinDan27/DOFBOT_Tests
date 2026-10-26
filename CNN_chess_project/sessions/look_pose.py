#!/usr/bin/env python3
"""
Teach the home pose -- hands-free at the arm.

Home is where the arm waits between jobs and where it starts and ends every
move. Before you teach one it's POSE_REST: folded back over the electronics,
claw facing forward over the board. Teach your own: resting like that, or
higher up with the camera looking down at the whole board.

At the arm: after the long beep (power off), move the arm where you want it
and hold still for LOOK_HOLD_S seconds. Blue light = this pose is usable,
keep still; red = not usable (a joint can't report, or the arm would be low
over the board) -- move it. Long beep = saved, and the arm holds itself there.

No per-servo limits (v2.5): any pose is fine as long as the arm's shape is
clear. If the electronics model (a guess until measured) says your pose is
too close to the electronics, your pose wins -- you're holding the arm
there, so there's room -- and the model's electronics box is lowered.

Saved to calibration/look_pose.json, plus a photo of the view in data/look/.
"""
import json
import math
import os
import time

import config
from robot import kinematics, safety, world
from robot.arm import ArmBlocked, FIX_BY_HAND, SERVO_NAMES, limit_problem, silent_servos


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
    if len(pose) != 6 or limit_problem(pose):
        print("  look_pose.json is outside what the servos can do -- ignoring it.")
        return None
    return pose


def check_pose(angles, zone):
    """
    Why these angles can't be the home pose, or None if they can. Being too
    close to the (guessed) electronics is NOT a reason: see make_room_for().
    """
    silent = silent_servos(angles)
    if silent:
        return silent + " can't report its angle"
    problem = limit_problem(angles)
    if problem:
        return problem
    if any(zone.danger(p) for p in safety.link_points(angles)):
        return ("too low over the board: every part of the arm must stay %.0f mm above "
                "it. Raise it." % config.CLEAR_MM)
    if hasattr(zone, "margin"):
        m, link, ob, _ = zone.margin(angles)
        if m < 0.0 and ob != "electronics":
            return zone.body_problem(angles)
    return None


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


def accept(arm, pose):
    """Makes `pose` the home pose, making room in the electronics model if needed."""
    if hasattr(arm.zone, "margin"):
        top = world.make_room_for(arm.zone, pose)
        if top is not None:
            print("  The electronics model said this was too close to the electronics, but")
            print("  you're holding the arm there, so there's room: its top is now %.0f mm "
                  "(saved)." % top)
    arm.home_pose = list(pose)


def run(arm, camera=None):
    """Hands-free. Returns the saved pose, or None."""
    from sessions.teach import BLUE, OFF, RED, _Light, _Stillness, _end_line, _show, countdown

    print("\n--- Teach the home pose (hands-free at the arm) ---")
    print("At the arm:")
    print("  * wait for the LONG beep -- be holding the arm by then (power goes off)")
    print("  * move it where it should wait: folded back resting, or up high with the camera")
    print("    on the whole board -- and hold still %.0f s" % config.LOOK_HOLD_S)
    print("  * blue light = usable, keep still; red = not usable here (a joint can't report,")
    print("    or it's low over the board) -- move it")
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
    # (so a pause before you start isn't saved as the pose).
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
                _show("  move the arm to where it should wait", screen)
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
        powered = arm.power_on_when_readable(timeout=config.FINISH_POWER_ON_S, show=True,
                                             light=True)
    except ArmBlocked as e:
        print("  Power is on, but: %s. It's holding still." % e)
        powered = True
    if not powered:
        light.set(RED)
        arm.beep(3)
        print("  Couldn't switch power back on -- the arm is still floppy. Lay it down gently.")
        if pose is not None:
            save_pose(pose, camera)
            accept(arm, pose)
            print("  (The pose was saved anyway.)")
        return pose
    light.set(OFF)
    if pose is None:
        arm.beep(2)
        return None
    photo = save_pose(pose, camera)
    accept(arm, pose)
    arm.beep(long=True)
    x, y, z, _ = kinematics.forward(*pose[:4])
    print("  Saved the home pose (claw at x=%.0f y=%.0f z=%.0f mm). The arm holds itself "
          "there." % (x, y, z))
    if photo:
        print("  What the camera sees from there: %s" % photo)
    return pose
