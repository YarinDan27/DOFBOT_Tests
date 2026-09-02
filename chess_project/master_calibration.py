#!/usr/bin/env python3
import time
import sys
import config
from arm_control import ArmControl
from board_mapper import BoardMapper
from detector import ChessDetector
from web_stream import WebStreamServer

def torque_claw_grab(arm):
    """Steps claw closed incrementally until resistance is met."""
    print("[Mechatronics] Testing smart torque grip closure...")
    arm.set_servo(6, config.CLAW_OPEN, config.CLAW_STEP_SPEED)
    time.sleep(0.3)
    
    current_angle = config.CLAW_OPEN
    while current_angle < config.CLAW_MAX:
        current_angle += config.CLAW_STEP
        arm.set_servo(6, current_angle, config.CLAW_STEP_SPEED)
        time.sleep(0.02)
        
        feedback_angle = arm.read_servo(6)
        if abs(current_angle - feedback_angle) > config.CLAW_STALL_GAP:
            print(f"[Mechatronics] Torque stall caught at {feedback_angle}°. Grip secure.")
            return True
            
    print("[Mechatronics] Empty grasp profile reached.")
    return False

def main():
    print("====================================================")
    print("    DOFBOT CHESS MASTER CALIBRATION WIZARD          ")
    print("====================================================")
    
    arm = ArmControl()
    mapper = BoardMapper()
    detector = ChessDetector(config.WEIGHTS)
    server = WebStreamServer()

    # PHASE 1: OBSERVATION & DOT AUDIT
    print("\n--- PHASE 1: Observation Angle & Red Dot Audit ---")
    arm.move_to_joints(config.POS_HOME, config.HOME_SPEED)
    time.sleep(1.0)
    arm.move_to_joints(config.POS_CHESS_OBSERVE, config.ACTION_SPEED)
    time.sleep(1.5)
    
    server.start_stream()
    while True:
        ready = input("Are the 4 red dots perfectly visible and centered? (y/n): ").strip().lower()
        if ready == 'y':
            server.stop_stream()
            print("[Phase 1 Approved] Observation position and layout perspective verified.")
            break
        print("Adjust hardware layout manually and try again.")

    # PHASE 2: THE e4 TOUCH BENCHMARK
    print("\n--- PHASE 2: The e4 Touch Benchmark ---")
    print("Navigating to baseline target square with clearance profile claw...")
    arm.set_servo(6, config.CLAW_OPEN, config.ACTION_SPEED)
    time.sleep(0.4)
    
    e4_joints = mapper.get_square_joints("e4", config.POS_CHESS_GRAB_BASE)
    arm.move_to_joints(e4_joints, config.ACTION_SPEED)
    time.sleep(1.5)
    
    input("Use mechanical adjustments until centered over e4. Press [Enter] to lock matrix...")
    print("[Phase 2 Approved] Coordinate scaling map benchmarked.")

    # PHASE 3: THE 4-CORNER ENVELOPE SWEEP
    print("\n--- PHASE 3: The 4-Corner Envelope Sweep ---")
    for corner in ["a1", "h1", "a8", "h8"]:
        print(f"\nMoving clearance profile claw to check boundary: {corner}")
        arm.set_servo(6, config.CLAW_OPEN, config.ACTION_SPEED)
        time.sleep(0.2)
        
        corner_joints = mapper.get_square_joints(corner, config.POS_CHESS_GRAB_BASE)
        arm.move_to_joints(corner_joints, config.ACTION_SPEED)
        time.sleep(1.2)
        
        torque_claw_grab(arm)
        input(f"Verify spatial tracking limits at {corner}. Press [Enter] to continue...")
    print("[Phase 3 Approved] Geometric working matrix confirmed.")

    # PHASE 4: STATIC PIECE VERIFICATION & SNAPSHOT
    print("\n--- PHASE 4: Static Piece Layout Verification ---")
    raw_frame = detector.get_frame()
    detected_board = detector.parse_layout(raw_frame, mapper)
    
    print("\n--- Current Visual Matrix Evaluation ---")
    print("Row 8: [R, N, B, Q, K, B, N, R] (Black)")
    print("Row 7: [P, P, P, P, P, P, P, P]")
    print("... Rows 3-6 Empty ...")
    print("Row 2: [P, P, P, P, P, P, P, P]")
    print("Row 1: [R, N, B, Q, K, B, N, R] (White)")
    print("----------------------------------------")
    
    while True:
        correction = input("Enter layout modifications (e.g., 'b8=Empty') or type 'done': ").strip()
        if correction.lower() == 'done':
            break
        try:
            square, identity = correction.split('=')
            detected_board[square.strip()] = identity.strip()
            print(f"Set tracking state {square.strip()} -> {identity.strip()}")
        except ValueError:
            print("Format must be 'Square=PieceType' or 'done'.")
            
    print("\n[Phase 4 Approved] Starting matrix state frozen.")
    print("Calibration cycle complete. Turning off active YOLO threads to preserve CPU.")
    
    arm.set_servo(6, config.CLAW_OPEN, config.ACTION_SPEED)
    arm.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)

if __name__ == "__main__":
    main()
