#!/usr/bin/env python3
"""
DOFBOT Chess.

  1 Calibrate : teach it where the board is
  2 Test      : prove it can reach and grab
  3 Play      : move pieces
  4 Camera    : live view on / off, and how fast it's actually running

Nothing about the board is hardcoded -- calibration writes board_calibration.json.
"""
import sys
import time
import select
import termios
import threading
import tty

import config
import chessboard
import kinematics
from arm import Arm, OutOfReach, UnsafeAngle

try:
    from camera import Camera
except Exception as exc:          # opencv missing, camera busy, etc.
    Camera = None
    CAMERA_ERROR = str(exc)

DEFAULT_CAL_SQUARES = ["a1", "h1", "a8"]


# ============================ keyboard jogger ============================

def jog(arm, start_angles, label, camera=None):
    print(f"\nJog: {label}")
    if camera is not None:
        print(f"  live view: {camera.url()}")
    print("  1/2 base   3/4 shoulder   5/6 elbow   7/8 servo4   9/0 wrist   -/= claw")
    print("  [space] confirm    [x] cancel")

    angles = dict(start_angles)
    step_map = {
        '1': (1, +2), '2': (1, -2),
        '3': (2, +2), '4': (2, -2),
        '5': (3, +2), '6': (3, -2),
        '7': (4, +2), '8': (4, -2),
        '9': (5, +2), '0': (5, -2),
        '-': (6, +2), '=': (6, -2),
    }
    running = True
    confirmed = False
    pressed = None
    lock = threading.Lock()

    def reader():
        nonlocal running, confirmed, pressed
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.01)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch == ' ':
                        confirmed, running = True, False
                    elif ch.lower() == 'x' or ch == '\x03':
                        confirmed, running = False, False
                    else:
                        with lock:
                            pressed = ch
                else:
                    with lock:
                        pressed = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def driver():
        while running:
            start = time.time()
            with lock:
                key = pressed
            if key in step_map:
                servo_id, delta = step_map[key]
                target = angles[servo_id] + delta
                low, high = config.SERVO_LIMITS[servo_id]
                if low <= target <= high:
                    angles[servo_id] = target
                    try:
                        arm.set_servo(servo_id, target, 60)
                    except UnsafeAngle:
                        pass
                else:
                    sys.stdout.write("\a")
            x, y, z, _ = kinematics.forward(angles[1], angles[2], angles[3], angles[4])
            sys.stdout.write(
                f"\r servos {[int(angles[i]) for i in range(1, 7)]}  "
                f"gripper x={x:7.1f} y={y:7.1f} z={z:7.1f}   ")
            sys.stdout.flush()
            time.sleep(max(0.03 - (time.time() - start), 0))

    threads = [threading.Thread(target=reader, daemon=True),
               threading.Thread(target=driver, daemon=True)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass
    print()
    return angles, confirmed


# ============================ calibrate ============================

def calibrate_board(arm, camera):
    print("\n--- Board calibration ---")
    print("Pick three squares to touch. They must not be in a straight line.")
    print(f"Default is {', '.join(DEFAULT_CAL_SQUARES)}; if a corner is hard to reach,")
    print("use a nearer one instead, e.g. a1 h1 a6.")
    raw = input(f"Squares [{' '.join(DEFAULT_CAL_SQUARES)}]: ").strip().lower()
    squares = raw.split() if raw else list(DEFAULT_CAL_SQUARES)
    if len(squares) != 3:
        print("  Need exactly three.")
        return None

    print("\nPut a piece on each of those squares (the same kind of piece each time).")
    print("For each one, jog the OPEN claw around the piece at grabbing height,")
    print("then press space.")
    input("Press [Enter] when the pieces are on the board...")

    arm.open_claw()
    points = {}
    angles = {i + 1: v for i, v in enumerate(arm.read_all())}
    for square in squares:
        angles, ok = jog(arm, angles, f"open claw around the piece on {square}", camera)
        if not ok:
            print("Cancelled. Nothing saved.")
            return None
        x, y, z, _ = kinematics.forward(angles[1], angles[2], angles[3], angles[4])
        points[square] = (x, y, z)
        print(f"  {square} recorded at x={x:.1f} y={y:.1f} z={z:.1f}")

    try:
        frame = chessboard.BoardFrame.from_points(points)
    except ValueError as exc:
        print(f"  {exc}")
        return None

    file_mm, rank_mm = frame.square_size_mm()
    print(f"\nSquare size measured: {file_mm:.1f}mm across files, {rank_mm:.1f}mm across ranks")
    if abs(file_mm - rank_mm) > 3.0:
        print("  WARNING: those should match. One of the three points was probably")
        print("  not right over the piece -- often the far square, because the arm")
        print("  can barely stretch there. Worth redoing before you trust it.")
    frame.save()
    print(f"Saved to {chessboard.CALIBRATION_PATH}")
    arm.park()
    return frame


def show_calibration(frame):
    if frame is None:
        print("\nNo calibration yet.")
        return
    file_mm, rank_mm = frame.square_size_mm()
    print(f"\nSquare size: {file_mm:.1f} x {rank_mm:.1f} mm")
    for square in ("a1", "h1", "a8", "h8", "e4"):
        x, y, z = frame.square_xyz(square)
        ok = kinematics.solve_reachable(x, y, z)[0] is not None
        print(f"  {square}: x={x:7.1f} y={y:7.1f} z={z:7.1f}  "
              f"{'reachable' if ok else 'OUT OF REACH'}")


def reachability_report(frame):
    if frame is None:
        print("\nCalibrate first.")
        return
    bad = [f"{f}{r}" for f in chessboard.FILES for r in range(1, 9)
           if kinematics.solve_reachable(*frame.square_xyz(f"{f}{r}"))[0] is None]
    if bad:
        print(f"\n{len(bad)} of 64 squares out of reach:\n  " + " ".join(bad))
        print("  Move the board closer, or lower the arm, then recalibrate.")
    else:
        print("\nAll 64 squares are reachable.")


def calibrate_menu(arm, state, camera):
    while True:
        print("\n-- Calibrate --")
        print("1. Board calibration (touch three squares)")
        print("2. Show current calibration")
        print("3. Which squares can it reach?")
        print("4. Back")
        choice = input("Select: ").strip()
        if choice == '1':
            frame = calibrate_board(arm, camera)
            if frame:
                state["frame"] = frame
        elif choice == '2':
            show_calibration(state["frame"])
        elif choice == '3':
            reachability_report(state["frame"])
        elif choice == '4':
            return


# ============================ test ============================

def need_frame(state):
    if state["frame"] is None:
        print("\nCalibrate the board first.")
        return False
    return True


def ask_square(prompt, frame):
    raw = input(prompt).strip().lower()
    try:
        return frame.square_xyz(raw), raw
    except ValueError as exc:
        print(f"  {exc}")
        return None, raw


def test_hover(arm, state):
    if not need_frame(state):
        return
    xyz, name = ask_square("Square to hover over (e.g. e4): ", state["frame"])
    if xyz is None:
        return
    try:
        arm.goto(xyz[0], xyz[1], xyz[2] + config.HOVER_MM)
        print(f"  hovering over {name}")
    except OutOfReach as exc:
        print(f"  {exc}")


def test_corners(arm, state):
    if not need_frame(state):
        return
    print("\nVisiting the four corners at hover height.")
    for square in ("a1", "h1", "h8", "a8"):
        x, y, z = state["frame"].square_xyz(square)
        try:
            arm.goto(x, y, z + config.HOVER_MM)
            print(f"  {square}: ok")
        except OutOfReach as exc:
            print(f"  {square}: {exc}")
        input("  [Enter] for the next corner...")
    arm.park()


def test_grab(arm, state):
    if not need_frame(state):
        return
    xyz, name = ask_square("Square with a piece on it (e.g. e2): ", state["frame"])
    if xyz is None:
        return
    x, y, z = xyz
    try:
        arm.open_claw()
        arm.goto(x, y, z + config.HOVER_MM)
        arm.move_line((x, y, z))
        print("  gripped something" if arm.close_claw() else "  closed on nothing")
        arm.move_line((x, y, z + config.HOVER_MM))
        input("  [Enter] to put it back...")
        arm.move_line((x, y, z))
        arm.open_claw()
        arm.move_line((x, y, z + config.HOVER_MM))
        arm.park()
    except OutOfReach as exc:
        print(f"  {exc}")


def test_menu(arm, state):
    while True:
        print("\n-- Test --")
        print("1. Hover over a square")
        print("2. Visit the four corners")
        print("3. Grab a piece and put it back")
        print("4. Back")
        choice = input("Select: ").strip()
        if choice == '1':
            test_hover(arm, state)
        elif choice == '2':
            test_corners(arm, state)
        elif choice == '3':
            test_grab(arm, state)
        elif choice == '4':
            return


# ============================ play ============================

def play_move(arm, state):
    if not need_frame(state):
        return
    src, from_name = ask_square("From square: ", state["frame"])
    if src is None:
        return
    dst, to_name = ask_square("To square: ", state["frame"])
    if dst is None:
        return
    try:
        gripped = arm.move_piece(src, dst)
        print(f"  moved {from_name} -> {to_name}" +
              ("" if gripped else "   (warning: the claw never felt a piece)"))
        arm.park()
    except OutOfReach as exc:
        print(f"  {exc}")


def play_menu(arm, state):
    while True:
        print("\n-- Play --")
        print("1. Move one piece")
        print("2. Full game (not built yet: needs the engine and vision)")
        print("3. Back")
        choice = input("Select: ").strip()
        if choice == '1':
            play_move(arm, state)
        elif choice == '2':
            print("  Not built yet.")
        elif choice == '3':
            return


# ============================ camera ============================

def camera_menu(state):
    while True:
        camera = state["camera"]
        status = "off" if camera is None else f"on -- {camera.url()}  ({camera.stats()})"
        print(f"\n-- Camera --  currently {status}")
        print("1. Start" if camera is None else "1. Stop")
        print("2. Refresh speed reading")
        print("3. Back")
        choice = input("Select: ").strip()
        if choice == '1':
            if camera is None:
                if Camera is None:
                    print(f"  camera unavailable: {CAMERA_ERROR}")
                    continue
                try:
                    cam = Camera()
                    cam.start()
                    state["camera"] = cam
                    print(f"  streaming at {cam.url()}")
                except Exception as exc:
                    print(f"  {exc}")
            else:
                camera.stop()
                state["camera"] = None
                print("  stopped")
        elif choice == '2':
            if camera:
                time.sleep(1.0)
                print(f"  {camera.stats()}")
        elif choice == '3':
            return


# ============================ main ============================

def main():
    print(f"DOFBOT Chess v{config.VERSION}")
    state = {"frame": chessboard.BoardFrame.load(), "camera": None}
    print("Board calibration: " + ("loaded" if state["frame"] else "none yet"))

    if Camera is not None:
        try:
            cam = Camera()
            cam.start()
            state["camera"] = cam
            print(f"Camera: {cam.url()}")
        except Exception as exc:
            print(f"Camera off: {exc}")
    else:
        print(f"Camera off: {CAMERA_ERROR}")

    with Arm() as arm:
        print(f"Servos: {[int(v) if v is not None else None for v in arm.read_all()]}")
        try:
            while True:
                print("\n=== DOFBOT Chess ===")
                print("1. Calibrate")
                print("2. Test")
                print("3. Play")
                print("4. Camera")
                print("5. Quit")
                choice = input("Select: ").strip()
                if choice == '1':
                    calibrate_menu(arm, state, state["camera"])
                elif choice == '2':
                    test_menu(arm, state)
                elif choice == '3':
                    play_menu(arm, state)
                elif choice == '4':
                    camera_menu(state)
                elif choice == '5':
                    return
        finally:
            if state["camera"]:
                state["camera"].stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped. Arm parking...")
