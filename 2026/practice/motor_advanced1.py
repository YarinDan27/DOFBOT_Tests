#!/usr/bin/env python3
"""
DOFBOT Advanced Control Script
=================================
This script demonstrates good coding practices while controlling the robot arm.

CODING LESSONS IN THIS FILE:
1. Constants at the top (makes it easy to change settings)
2. Functions for reusable code (DRY - Don't Repeat Yourself)
3. Clear naming conventions (you know what everything does)
4. Comments that explain WHY, not just WHAT
5. Error handling (try/except blocks)
6. Dictionary-based configurations (easy to add new poses)
7. Sensor feedback (reading servo positions for smart gripper)
"""

import time
import sys

# ============================================================================
# LESSON 1: CONSTANTS - Define all your "magic numbers" at the top
# This makes it easy to tweak settings without hunting through code
# ============================================================================

# Speed settings (in milliseconds - higher = slower)
SPEED_FAST = 500
SPEED_NORMAL = 1000
SPEED_SLOW = 1500

# Gripper positions
GRIPPER_OPEN = 60
GRIPPER_CLOSED = 120
GRIPPER_NEUTRAL = 90

# Smart gripper settings
GRIPPER_STEP_SIZE = 5          # How many degrees to close per step
GRIPPER_MAX_ATTEMPTS = 20      # Max steps before giving up
GRIPPER_POSITION_TOLERANCE = 3 # Degrees of tolerance for "reached position"

# Servo limits (safety!)
SERVO_MIN = 0
SERVO_MAX = 180

# ============================================================================
# LESSON 2: IMPORT AND INITIALIZATION
# Check if libraries exist before using them (defensive programming)
# ============================================================================

try:
    from Arm_Lib import Arm_Device
except ImportError:
    print("ERROR: Arm_Lib not found!")
    print("Make sure you're running on the DOFBOT/Jetson system.")
    sys.exit(1)

# ============================================================================
# LESSON 3: DICTIONARIES FOR CONFIGURATION
# Use dictionaries to store related data - makes code cleaner and scalable
# ============================================================================

# Motor name to ID mapping
MOTOR_MAP = {
    'base': 1,
    'shoulder': 2,
    'elbow': 3,
    'wrist_pitch': 4,
    'wrist_roll': 5,
    'gripper': 6
}

# ============================================================================
# POSES - Static positions the robot can hold
# ============================================================================
POSES = {
    'home': {
        1: 90,   # base
        2: 90,   # shoulder
        3: 90,   # elbow
        4: 90,   # wrist_pitch
        5: 90,   # wrist_roll
        6: 90    # gripper
    },
    
    'stare': {
        1: 90,   # base forward
        2: 180,  # shoulder all the way up
        3: 0,    # elbow straight
        4: 0,    # wrist pitch up
        5: 90,   # wrist roll neutral
        6: 90    # gripper neutral
    },
    
    'pickup': {
        1: 90,   # base forward
        2: 120,  # shoulder lowered
        3: 120,  # elbow bent down
        4: 150,  # wrist pointing down
        5: 90,   # wrist straight
        6: 60    # gripper open and ready
    },
    
    'rest': {
        1: 90,
        2: 150,  # shoulder down
        3: 150,  # elbow tucked
        4: 90,
        5: 90,
        6: 90
    }
}

# ============================================================================
# MOVES - Multi-step sequences (animated movements)
# ============================================================================
MOVES = {
    'wave': [
        {'pose': {1: 45, 2: 60, 3: 60, 4: 90, 5: 90, 6: 90}, 'duration': 1.5},
        {'pose': {4: 60}, 'duration': 0.5},   # Wrist up
        {'pose': {4: 120}, 'duration': 0.5},  # Wrist down
        {'pose': {4: 60}, 'duration': 0.5},   # Wrist up
        {'pose': {4: 120}, 'duration': 0.5},  # Wrist down
        {'pose': {4: 60}, 'duration': 0.5},   # Wrist up
        {'pose': {4: 90}, 'duration': 0.5},   # Wrist neutral
    ],
    
    'dance': [
        {'pose': {1: 45, 2: 60, 3: 60}, 'duration': 1.0},   # Left lean
        {'pose': {1: 135, 2: 60, 3: 60}, 'duration': 1.0},  # Right lean
        {'pose': {1: 90, 2: 45, 3: 45}, 'duration': 1.0},   # Center up
        {'pose': {1: 90, 2: 120, 3: 120}, 'duration': 1.0}, # Center down
        {'pose': {1: 45, 2: 60, 5: 45}, 'duration': 0.8},   # Twist left
        {'pose': {1: 135, 2: 60, 5: 135}, 'duration': 0.8}, # Twist right
        {'pose': {1: 90, 5: 90}, 'duration': 1.0},          # Center
    ]
}

# ============================================================================
# LESSON 4: CLASSES FOR ORGANIZATION
# Group related functions together - keeps code organized
# ============================================================================

class RobotController:
    """
    Main robot controller class
    
    CODING TIP: Classes help organize code into logical units.
    All robot control functions live here.
    """
    
    def __init__(self):
        """
        Initialize the robot arm
        
        CODING TIP: __init__ is called when you create the object.
        Use it to set up initial state.
        """
        print("Initializing DOFBOT...")
        self.arm = Arm_Device()
        time.sleep(1)
        print("✓ Arm initialized!")
        
    # ========================================================================
    # LESSON 5: HELPER FUNCTIONS - Small, reusable building blocks
    # ========================================================================
    
    def _validate_servo(self, servo_id):
        """
        Validate servo ID is in valid range
        
        CODING TIP: Functions starting with _ are "private" - 
        meant for internal use only
        """
        if not (1 <= servo_id <= 6):
            raise ValueError(f"Invalid servo ID: {servo_id}. Must be 1-6.")
        return True
        
    def _validate_angle(self, angle):
        """Validate angle is in safe range"""
        if not (SERVO_MIN <= angle <= SERVO_MAX):
            raise ValueError(f"Angle {angle}° out of range ({SERVO_MIN}-{SERVO_MAX})")
        return True
        
    def _clamp_angle(self, angle):
        """
        Clamp angle to safe limits
        
        CODING TIP: Instead of throwing errors, sometimes it's better
        to automatically fix the input (but tell the user!)
        """
        if angle < SERVO_MIN:
            print(f"Warning: Angle {angle}° too low, clamping to {SERVO_MIN}°")
            return SERVO_MIN
        if angle > SERVO_MAX:
            print(f"Warning: Angle {angle}° too high, clamping to {SERVO_MAX}°")
            return SERVO_MAX
        return angle
    
    def read_servo_position(self, servo_id):
        """
        Read current position of a servo
        
        Args:
            servo_id (int): Servo number (1-6)
            
        Returns:
            int: Current angle of servo, or None if read fails
            
        CODING TIP: Reading servo position allows for feedback!
        We can detect if an object is blocking the gripper.
        """
        try:
            position = self.arm.Arm_serial_servo_read(servo_id)
            return position
        except Exception as e:
            print(f"Warning: Could not read servo {servo_id}: {e}")
            return None
    
    # ========================================================================
    # CORE MOVEMENT FUNCTIONS
    # ========================================================================
    
    def move_servo(self, servo_id, angle, speed=SPEED_NORMAL):
        """
        Move a single servo motor
        
        Args:
            servo_id (int): Servo number (1-6)
            angle (int): Target angle (0-180)
            speed (int): Movement speed in ms
        """
        # Validate inputs
        self._validate_servo(servo_id)
        angle = self._clamp_angle(angle)
        
        # Execute movement
        print(f"Moving servo {servo_id} to {angle}° (speed: {speed}ms)")
        self.arm.Arm_serial_servo_write(servo_id, angle, speed)
        
    def move_multiple(self, servo_angles, speed=SPEED_NORMAL):
        """
        Move multiple servos at once
        
        Args:
            servo_angles (dict): Dictionary of {servo_id: angle}
            speed (int): Movement speed in ms
            
        CODING TIP: This function accepts a dictionary, making it
        flexible to move any combination of servos
        """
        print(f"Moving {len(servo_angles)} servos simultaneously...")
        for servo_id, angle in servo_angles.items():
            # Convert string keys to int if needed
            servo_id = int(servo_id)
            self.move_servo(servo_id, angle, speed)
        
        # Wait for movement to complete
        time.sleep(speed / 1000 + 0.2)
    
    # ========================================================================
    # SMART GRIPPER CONTROL - With resistance detection!
    # ========================================================================
    
    def smart_grip(self):
        """
        Intelligent gripper that closes slowly and detects objects
        
        CODING TIP: This is a great example of using sensor feedback!
        We command a position, then read back to see if we got there.
        If we didn't, something is blocking the gripper (the object!)
        
        Returns:
            bool: True if object detected, False if gripper closed fully
        """
        print("\n=== Smart Grip Engaged ===")
        print("Slowly closing gripper until object detected...")
        
        # Start from open position
        current_angle = GRIPPER_OPEN
        self.arm.Arm_serial_servo_write(6, current_angle, SPEED_FAST)
        time.sleep(0.5)
        
        attempts = 0
        object_detected = False
        
        # LESSON: While loop with exit conditions
        while attempts < GRIPPER_MAX_ATTEMPTS and current_angle < GRIPPER_CLOSED:
            attempts += 1
            
            # Close gripper by one step
            target_angle = current_angle + GRIPPER_STEP_SIZE
            target_angle = min(target_angle, GRIPPER_CLOSED)  # Don't exceed max
            
            print(f"  Step {attempts}: Commanding {target_angle}°...", end=" ")
            self.arm.Arm_serial_servo_write(6, target_angle, SPEED_SLOW)
            
            # Wait for movement to complete
            time.sleep(SPEED_SLOW / 1000 + 0.15)
            
            # Read actual position
            actual_position = self.read_servo_position(6)
            
            if actual_position is None:
                print("(can't read position)")
                current_angle = target_angle  # Assume it worked
                continue
            
            print(f"actual: {actual_position}°")
            
            # LESSON: Detect resistance by comparing commanded vs actual position
            position_error = abs(target_angle - actual_position)
            
            if position_error > GRIPPER_POSITION_TOLERANCE:
                # Gripper couldn't reach target = object detected!
                print(f"  ✓ OBJECT DETECTED! (Error: {position_error}°)")
                print(f"  Gripper stopped at {actual_position}°")
                object_detected = True
                break
            else:
                # Gripper reached target, continue closing
                current_angle = target_angle
        
        if not object_detected:
            if current_angle >= GRIPPER_CLOSED:
                print("  ✗ Gripper fully closed, no object detected")
            else:
                print("  ? Max attempts reached")
        
        print("=== Smart Grip Complete ===\n")
        return object_detected
    
    def gripper_open(self):
        """Open gripper"""
        print("Opening gripper...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_OPEN, SPEED_FAST)
        time.sleep(0.8)
        
    def gripper_close(self):
        """Close gripper (simple, no detection)"""
        print("Closing gripper...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_CLOSED, SPEED_FAST)
        time.sleep(0.8)
    
    # ========================================================================
    # POSE EXECUTION
    # ========================================================================
    
    def execute_pose(self, pose_name, speed=SPEED_NORMAL):
        """
        Execute a predefined pose (static position)
        
        Args:
            pose_name (str): Name of pose from POSES dictionary
            speed (int): Movement speed
            
        CODING TIP: By storing poses in a dictionary at the top,
        we can easily add new poses without changing this function
        """
        if pose_name not in POSES:
            print(f"Unknown pose: {pose_name}")
            print(f"Available poses: {', '.join(POSES.keys())}")
            return False
            
        print(f"Executing POSE: {pose_name}")
        pose = POSES[pose_name]
        self.move_multiple(pose, speed)
        return True
    
    # ========================================================================
    # MOVE EXECUTION
    # ========================================================================
    
    def execute_move(self, move_name):
        """
        Execute a multi-step move (animated sequence)
        
        Args:
            move_name (str): Name from MOVES dictionary
            
        CODING TIP: Moves are lists of steps. We iterate through
        each step and execute it. This pattern works for any length sequence!
        """
        if move_name not in MOVES:
            print(f"Unknown move: {move_name}")
            print(f"Available moves: {', '.join(MOVES.keys())}")
            return False
            
        print(f"Starting MOVE: {move_name}")
        move = MOVES[move_name]
        
        # LESSON: Iterate through a list of steps
        for i, step in enumerate(move, 1):
            print(f"  Step {i}/{len(move)}")
            pose = step['pose']
            duration = step['duration']
            
            # Move to this step's position
            self.move_multiple(pose, int(duration * 500))
            time.sleep(duration)
        
        print(f"✓ Move '{move_name}' complete!")
        return True
    
    # ========================================================================
    # INTERACTIVE MENU
    # ========================================================================
    
    def run_menu(self):
        """
        Main interactive menu
        
        CODING TIP: Keep the menu loop simple. Complex logic
        should be in separate functions.
        """
        print("\n" + "="*60)
        print("DOFBOT Advanced Control System")
        print("="*60)
        
        while True:
            print("\n--- MAIN MENU ---")
            print("\n[Basic Controls]")
            print("  1. Move individual servo (constant speed)")
            print("  2. Open gripper")
            print("  3. Close gripper (simple)")
            print("  4. SMART GRIP (detect object)")
            
            print("\n[POSES - Static Positions]")
            print("  h. HOME position")
            print("  s. STARE position") 
            print("  p. PICKUP position")
            print("  r. REST position")
            
            print("\n[MOVES - Animated Sequences]")
            print("  w. WAVE move")
            print("  d. DANCE move")
            
            print("\n[Exit]")
            print("  q. QUIT")
            
            choice = input("\nChoice: ").strip().lower()
            
            # ================================================================
            # LESSON 6: CONTROL FLOW - if/elif/else chains
            # ================================================================
            
            if choice == 'q':
                print("Returning to home position...")
                self.execute_pose('home')
                print("Goodbye!")
                break
                
            # ---- BASIC CONTROLS ----
            elif choice == '1':
                self._menu_move_servo()
                
            elif choice == '2':
                self.gripper_open()
                
            elif choice == '3':
                self.gripper_close()
                
            elif choice == '4':
                detected = self.smart_grip()
                if detected:
                    print("✓ Successfully gripped object!")
                else:
                    print("No object detected")
            
            # ---- POSES ----
            elif choice == 'h':
                self.execute_pose('home')
                
            elif choice == 's':
                self.execute_pose('stare')
                
            elif choice == 'p':
                self.execute_pose('pickup')
                
            elif choice == 'r':
                self.execute_pose('rest')
            
            # ---- MOVES ----
            elif choice == 'w':
                self.execute_move('wave')
                
            elif choice == 'd':
                self.execute_move('dance')
            
            else:
                print("Invalid choice! Try again.")
    
    def _menu_move_servo(self):
        """
        Sub-menu for moving individual servos
        
        CODING TIP: Breaking menu options into separate functions
        keeps the main menu clean and readable
        """
        print("\n--- Move Individual Servo ---")
        print("Motor IDs:")
        for name, motor_id in MOTOR_MAP.items():
            print(f"  {motor_id} = {name}")
        
        # LESSON 7: INPUT VALIDATION with try/except
        try:
            servo = int(input("\nServo ID (1-6): "))
            angle = int(input("Angle (0-180): "))
            
            # Use constant speed for simplicity
            self.move_servo(servo, angle, SPEED_NORMAL)
            time.sleep(SPEED_NORMAL / 1000 + 0.5)
            
        except ValueError as e:
            print(f"Invalid input: {e}")
        except Exception as e:
            print(f"Error: {e}")

# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    """
    Main entry point
    
    CODING TIP: Always use try/except at the top level to catch
    unexpected errors and clean up gracefully
    """
    try:
        # Create controller and run
        robot = RobotController()
        robot.run_menu()
        
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user!")
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # LESSON 8: 'finally' block ALWAYS runs, even after errors
        print("\nProgram ended.")

# ============================================================================
# LESSON 9: The if __name__ == "__main__" pattern
# This ensures main() only runs when script is executed directly,
# not when imported as a module
# ============================================================================

if __name__ == "__main__":
    main()
