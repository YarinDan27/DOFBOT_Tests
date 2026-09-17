#!/usr/bin/env python3
import sys
import time
import socket
import tty
import termios
import select
import threading
import config
from arm_control import ArmControl
from board_mapper import BoardMapper
from arm_kinematics import ArmKinematics
from chess_engine import ChessEngineInterface
from web_stream import WebStreamServer

def update_config_file(key, value):
    config_path = "/home/jetson/YDscripts/chess_project/config.py"
    try:
        with open(config_path, "r") as f:
            lines = f.readlines()
        with open(config_path, "w") as f:
            for line in lines:
                if line.strip().startswith(key):
                    f.write(f"{key} = {value}\n")
                else:
                    f.write(line)
        print(f"[Config] Saved {key} to disk.")
    except Exception as e:
        print(f"[Config] Failed to save {key}: {e}")

def get_jetson_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def flush_terminal_buffer():
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass

def interactive_jogger(arm, baseline_array):
    print("\nManual jog. Hold a key to sweep that motor.")
    print("  1/2 base   3/4 shoulder   5/6 elbow   7/8 wrist-roll   9/0 wrist-tilt   -/= claw")
    print("  [space] confirm position    [x] cancel")

    shared_joints = list(baseline_array)
    running = True
    confirmed = True
    active_key = None
    key_lock = threading.Lock()

    def input_reader():
        nonlocal running, active_key, confirmed
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while running:
                r, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == ' ':
                        confirmed = True
                        running = False
                        break
                    elif ch.lower() == 'x':
                        confirmed = False
                        running = False
                        break
                    with key_lock:
                        active_key = ch
                else:
                    with key_lock:
                        active_key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def motor_driver():
        nonlocal running, active_key
        step_map = {
            '1': (0, 3),  '2': (0, -3),
            '3': (1, 2),  '4': (1, -2),
            '5': (2, 2),  '6': (2, -2),
            '7': (3, 3),  '8': (3, -3),
            '9': (4, 3),  '0': (4, -3),
            '-': (5, 3),  '=': (5, -3)
        }
        while running:
            loop_start = time.time()
            with key_lock:
                current_key = active_key
            if current_key in step_map:
                idx, delta = step_map[current_key]
                new_angle = shared_joints[idx] + delta
                if 0 <= new_angle <= 180:
                    shared_joints[idx] = new_angle
                    if arm:
                        arm.set_servo(idx + 1, new_angle, 20)
            sys.stdout.write(f"\r[Position] {shared_joints}    ")
            sys.stdout.flush()
            time.sleep(max(0.025 - (time.time() - loop_start), 0))

    t1 = threading.Thread(target=input_reader, daemon=True)
    t2 = threading.Thread(target=motor_driver, daemon=True)
    t1.start(); t2.start()
    t1.join(); t2.join()
    flush_terminal_buffer()
    print()
    return shared_joints, confirmed

def run_hardware_flash(arm):
    print("\nFactory servo reset. This re-zeros all 6 motors to 90 degrees.")
    if arm:
        arm.move_to_joints([90, 90, 90, 90, 90, 90], config.HOME_SPEED)
        time.sleep(2.0)
        for i in range(1, 7):
            try:
                arm.bus.Arm_serial_servo_write_offset_switch(i)
                time.sleep(0.05)
            except Exception as e:
                print(f"[Warning] Servo {i} offset-clear failed: {e}")
        print("Hold the arm perfectly straight and vertical.")
        input("Press [Enter] to release motor torque...")
        arm.bus.Arm_serial_set_torque(0)
        time.sleep(0.5)
    print("Adjust the arm manually to dead-center vertical.")
    input("Once aligned, press [Enter] to lock it in as the new zero...")
    if arm:
        arm.bus.Arm_serial_set_torque(1)
        time.sleep(0.5)
        arm.bus.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
        time.sleep(1.0)
        arm.bus.Arm_serial_servo_write_offset_state()
        time.sleep(0.5)
    print("Done. Servos re-zeroed.")

def quick_relock(arm, mapper, server):
    """
    Mandatory every boot: since these servos lose position reference once
    powered off, we re-verify the two reference points math depends on
    (observe angle, e4 grab point) before anything else is allowed to run.
    """
    print("\n=== Startup relock (required) ===")
    jetson_ip = get_jetson_ip()
    print(f"Live view: http://{jetson_ip}:{config.WEB_PORT}")

    if arm:
        arm.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)
    print("\nStep 1/2: confirm the observation angle still frames the board.")
    new_obs, valid = interactive_jogger(arm, config.POS_CHESS_OBSERVE)
    if not valid:
        return False
    config.POS_CHESS_OBSERVE = new_obs
    update_config_file("POS_CHESS_OBSERVE", str(new_obs))

    if arm:
        e4_joints = mapper.get_square_joints("e4", config.POS_CHESS_GRAB_BASE)
        arm.move_to_joints(e4_joints, config.ACTION_SPEED)
    print("\nStep 2/2: confirm the gripper is centered over square e4.")
    new_base, valid = interactive_jogger(arm, config.POS_CHESS_GRAB_BASE)
    if not valid:
        return False
    config.POS_CHESS_GRAB_BASE = new_base
    update_config_file("POS_CHESS_GRAB_BASE", str(new_base))

    if arm:
        arm.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)
    print("\nRelock complete.")
    return True

def run_full_recalibration(arm, mapper):
    """Optional, deeper check: run this if the board itself moved, not just the arm."""
    print("\n=== Full board recalibration ===")
    for corner in ["a1", "h1", "a8", "h8"]:
        print(f"\nChecking corner: {corner}")
        if arm:
            c_joints = mapper.get_square_joints(corner, config.POS_CHESS_GRAB_BASE)
            arm.move_to_joints(c_joints, config.ACTION_SPEED)
        _, valid = interactive_jogger(arm, config.POS_CHESS_GRAB_BASE)
        if not valid:
            return False
    print("\nAll 4 corners verified.")
    return True

def main():
    print (f"DOFBOT Chess v{config.VERSION}")
    arm = ArmControl()
    mapper = BoardMapper()
    kinematics = ArmKinematics(arm)
    stockfish = ChessEngineInterface()
    server = WebStreamServer()
    server.start_stream()

    # Mandatory every run -- not a menu option, can't be skipped.
    while not quick_relock(arm, mapper, server):
        print("\nRelock was cancelled. It must complete before continuing.")
        retry = input("Press [Enter] to retry, or 'q' to quit: ").strip().lower()
        if retry == 'q':
            server.stop_stream()
            sys.exit()

    while True:
        print("\n=== DOFBOT Chess ===")
        print("1. Full board recalibration (only if the board itself moved)")
        print("2. Factory servo reset")
        print("3. Play")
        print("4. Quit")
        choice = input("Select: ").strip()

        if choice == '4':
            server.stop_stream()
            sys.exit()
        elif choice == '1':
            run_full_recalibration(arm, mapper)
        elif choice == '2':
            run_hardware_flash(arm)
        elif choice == '3':
            print("\n[Game Initialized] Active. Robot playing as Black.")
            break

if __name__ == "__main__":
    main()