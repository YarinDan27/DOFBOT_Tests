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
    config_path = "/home/jetson/YDscripts/2026/chess_project/config.py"
    try:
        with open(config_path, "r") as f:
            lines = f.readlines()
        with open(config_path, "w") as f:
            for line in lines:
                if line.strip().startswith(key):
                    f.write(f"{key} = {value}\n")
                else:
                    f.write(line)
        print(f"\n[Config Save] Successfully wrote {key} to disk.")
    except Exception as e:
        print(f"\n[Config Error] Failed auto-save: {e}")

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

# ====================================================================
# HIGH-SPEED ASYNCHRONOUS FLUID JOGGER ENGINE
# ====================================================================
def interactive_jogger(arm, baseline_array):
    print("\n==============================================")
    print("     HIGH-SPEED FLUID MECHATRONIC JOGGER      ")
    print("==============================================")
    print("  Hold keys down for fluid continuous sweep.  ")
    print("  [1] / [2] : Motor 1 (Base)")
    print("  [3] / [4] : Motor 2 (Shoulder)")
    print("  [5] / [6] : Motor 3 (Elbow)")
    print("  [7] / [8] : Motor 4 (Wrist Roll)")
    print("  [9] / [0] : Motor 5 (Wrist Tilt)")
    print("  [-] / [=] : Motor 6 (Claw)")
    print("  [Spacebar]: Lock Position and Confirm Phase")
    print("  [x]       : Quit Calibration Back to Menu")
    print("----------------------------------------------")

    shared_joints = list(baseline_array)
    running = True
    confirmed = True
    active_key = None
    key_lock = threading.Lock()

    # THREAD 1: Instantaneous Non-Blocking Input Reader
    def input_reader():
        nonlocal running, active_key, confirmed
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while running:
                # High frequency poll (100Hz) for raw key status
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
                        active_key = None  # No key held down down-cycles movement
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # THREAD 2: Smooth Hardware Vector Actuator
    def motor_driver():
        nonlocal running, active_key
        # High speed velocity steps
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
                # Enforce physical hardware geometric boundaries
                new_angle = shared_joints[idx] + delta
                if 0 <= new_angle <= 180:
                    shared_joints[idx] = new_angle
                    if arm:
                        # Direct register update with ultra-low execution time duration
                        arm.set_servo(idx + 1, new_angle, 20)
            
            # Print state out at a regulated rate without bottlenecking local bus operations
            sys.stdout.write(f"\r[Live Hardware Vector] {shared_joints}    ")
            sys.stdout.flush()
            
            # Sleep precise remnant to maintain rock-solid 40Hz output update stream
            time.sleep(max(0.025 - (time.time() - loop_start), 0))

    # Spawn both parallel worker processes
    t1 = threading.Thread(target=input_reader, daemon=True)
    t2 = threading.Thread(target=motor_driver, daemon=True)
    t1.start()
    t2.start()
    
    t1.join()
    t2.join()
    
    flush_terminal_buffer()
    return shared_joints, confirmed

def run_hardware_flash(arm):
    print("\n==============================================")
    print("  RAW FROM-SCRATCH SERVO CENTER-POINT FLASH   ")
    print("==============================================")
    
    if arm:
        print("[Step 1] Moving arm to where it CURRENTLY thinks center is...")
        arm.move_to_joints([90, 90, 90, 90, 90, 90], config.HOME_SPEED)
        time.sleep(2.0)
        
        print("\n[Step 2] Initializing factory hardware reset switch...")
        for i in range(1, 7):
            try:
                arm.bus.Arm_serial_servo_write_offset_switch(i)
                time.sleep(0.05)
            except Exception as e:
                print(f"[Warning] Index {i} register clear bypass: {e}")
                
        print("\n[Step 3] Releasing motor torque completely.")
        print("-> PLEASE PHYSICALLY HOLD THE ARM PERFECTLY STRAIGHT UP & CENTERED.")
        input("Press [Enter] to make the arm go completely limp...")
        
        arm.bus.Arm_serial_set_torque(0)
        time.sleep(0.5)
    
    print("\n[Step 4] Arm is limp. Adjust it manually to absolute vertical center.")
    input("Once aligned perfectly straight, press [Enter] to write permanent 90° EEPROM flash...")
    
    if arm:
        print("[Hardware] Locking torque back down and flashing EEPROM register...")
        arm.bus.Arm_serial_set_torque(1)
        time.sleep(0.5)
            
        arm.bus.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
        time.sleep(1.0)
        
        arm.bus.Arm_serial_servo_write_offset_state()
        time.sleep(0.5)
            
    print("[Success] Hardware servo brains completely factory-reset and centered!")

def run_serial_calibration(arm, mapper, server):
    print("\n==============================================")
    print("    STARTING 4-PHASE SERIAL CALIBRATION       ")
    print("==============================================")
    
    # PHASE 1
    print("\n--- Phase 1: Verify Observation Space ---")
    jetson_ip = get_jetson_ip()
    print(f"\n>>> LIVE STREAM RUNNING AT: http://{jetson_ip}:{config.WEB_PORT} <<<")
    if arm: arm.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)
    new_obs, valid = interactive_jogger(arm, config.POS_CHESS_OBSERVE)
    if not valid: return False
    
    config.POS_CHESS_OBSERVE = new_obs
    update_config_file("POS_CHESS_OBSERVE", str(new_obs))
    
    # PHASE 2
    print("\n--- Phase 2: Centering e4 Touch Benchmark ---")
    if arm:
        e4_joints = mapper.get_square_joints("e4", config.POS_CHESS_GRAB_BASE)
        arm.move_to_joints(e4_joints, config.ACTION_SPEED)
    new_base, valid = interactive_jogger(arm, config.POS_CHESS_GRAB_BASE)
    if not valid: return False
    
    config.POS_CHESS_GRAB_BASE = new_base
    update_config_file("POS_CHESS_GRAB_BASE", str(new_base))
    
    # PHASE 3
    print("\n--- Phase 3: Verification of 4 Corners ---")
    for corner in ["a1", "h1", "a8", "h8"]:
        print(f"\nChecking Boundary: Square {corner}")
        if arm:
            c_joints = mapper.get_square_joints(corner, config.POS_CHESS_GRAB_BASE)
            arm.move_to_joints(c_joints, config.ACTION_SPEED)
        _, valid = interactive_jogger(arm, config.POS_CHESS_GRAB_BASE)
        if not valid: return False
        
    # PHASE 4
    print("\n--- Phase 4: Static Piece State Audit ---")
    print("Set up layout. Press [Spacebar] to verify.")
    _, valid = interactive_jogger(arm, config.POS_CHESS_OBSERVE)
    if not valid: return False
    
    print("\n>>> SUCCESS: ALL CALIBRATION PHASES COMPLETED VERBALLY! <<<")
    if arm: arm.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)
    return True

def main():
    arm = ArmControl()
    mapper = BoardMapper()
    kinematics = ArmKinematics(arm)
    stockfish = ChessEngineInterface()
    
    server = WebStreamServer()
    server.start_stream()
    calibration_verified = False
    
    while True:
        print("\n==============================================")
        print("          DOFBOT CENTRAL CONTROLLER           ")
        print("==============================================")
        print("1. Raw Hardware Servo Center-Point Flash (From Scratch Reset)")
        print("2. Run 4-Step Serial System Calibration")
        print("3. Launch Stockfish Battle Loop")
        print("4. Shutdown Application")
        choice = input("Select operation: ").strip()
        
        if choice == '4':
            server.stop_stream()
            sys.exit()
        elif choice == '1':
            run_hardware_flash(arm)
        elif choice == '2':
            calibration_verified = run_serial_calibration(arm, mapper, server)
        elif choice == '3':
            if not calibration_verified:
                print("[Access Denied] Complete option 2 calibration first!")
                continue
            print("\n[Game Initialized] Active. Robot playing as Black.")
            break

if __name__ == "__main__":
    main()
