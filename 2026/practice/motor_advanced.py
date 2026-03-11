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

# Predefined poses - stored as dictionaries
# Format: {motor_id: angle}
POSES = {
    'home': {
        1: 90,   # base center
        2: 90,   # shoulder neutral
        3: 90,   # elbow neutral
        4: 90,   # wrist_pitch neutral
        5: 90,   # wrist_roll neutral
        6: 90    # gripper neutral
    },
    
    'stare': {
        1: 90,   # base forward
        2: 45,   # shoulder up high
        3: 30,   # elbow extended
        4: 60,   # wrist looking down
        5: 90,   # wrist straight
        6: 90    # gripper neutral
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

# Multi-step sequences (list of poses with timing)
SEQUENCES = {
    'wave': [
        {'pose': {'1': 45, '2': 60, '3': 60, '4': 90, '5': 90, '6': 90}, 'duration': 1.5},
        {'pose': {'4': 60}, 'duration': 0.5},   # Wrist up
        {'pose': {'4': 120}, 'duration': 0.5},  # Wrist down
        {'pose': {'4': 60}, 'duration': 0.5},   # Wrist up
        {'pose': {'4': 120}, 'duration': 0.5},  # Wrist down
        {'pose': {'4': 60}, 'duration': 0.5},   # Wrist up
        {'pose': {'4': 90}, 'duration': 0.5},   # Wrist neutral
    ],
    
    'dance': [
        {'pose': {'1': 45, '2': 60, '3': 60}, 'duration': 1.0},   # Left lean
        {'pose': {'1': 135, '2': 60, '3': 60}, 'duration': 1.0},  # Right lean
        {'pose': {'1': 90, '2': 45, '3': 45}, 'duration': 1.0},   # Center up
        {'pose': {'1': 90, '2': 120, '3': 120}, 'duration': 1.0}, # Center down
        {'pose': {'1': 45, '2': 60, '5': 45}, 'duration': 0.8},   # Twist left
        {'pose': {'1': 135, '2': 60, '5': 135}, 'duration': 0.8}, # Twist right
        {'pose': {'1': 90, '5': 90}, 'duration': 1.0},            # Center
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
    # GRIPPER CONTROL - Combining open/close into one toggle function
    # ========================================================================
    
    def gripper_toggle(self):
        """
        Toggle gripper between open and closed
        
        CODING TIP: We could track state, but simpler to just
        open->close->open in sequence
        """
        print("Gripper: Opening...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_OPEN, SPEED_FAST)
        time.sleep(1)
        
        print("Gripper: Closing...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_CLOSED, SPEED_FAST)
        time.sleep(1)
        
        print("Gripper: Back to neutral")
        self.arm.Arm_serial_servo_write(6, GRIPPER_NEUTRAL, SPEED_FAST)
        time.sleep(1)
    
    def gripper_open(self):
        """Open gripper"""
        print("Opening gripper...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_OPEN, SPEED_FAST)
        time.sleep(0.8)
        
    def gripper_close(self):
        """Close gripper"""
        print("Closing gripper...")
        self.arm.Arm_serial_servo_write(6, GRIPPER_CLOSED, SPEED_FAST)
        time.sleep(0.8)
    
    # ========================================================================
    # POSE EXECUTION
    # ========================================================================
    
    def execute_pose(self, pose_name, speed=SPEED_NORMAL):
        """
        Execute a predefined pose
        
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
            
        print(f"Executing pose: {pose_name}")
        pose = POSES[pose_name]
        self.move_multiple(pose, speed)
        return True
    
    # ========================================================================
    # SEQUENCE EXECUTION
    # ========================================================================
    
    def execute_sequence(self, sequence_name):
        """
        Execute a multi-step sequence
        
        Args:
            sequence_name (str): Name from SEQUENCES dictionary
            
        CODING TIP: Sequences are lists of steps. We iterate through
        each step and execute it. This pattern works for any length sequence!
        """
        if sequence_name not in SEQUENCES:
            print(f"Unknown sequence: {sequence_name}")
            print(f"Available sequences: {', '.join(SEQUENCES.keys())}")
            return False
            
        print(f"Starting sequence: {sequence_name}")
        sequence = SEQUENCES[sequence_name]
        
        # LESSON: Iterate through a list of steps
        for i, step in enumerate(sequence, 1):
            print(f"  Step {i}/{len(sequence)}")
            pose = step['pose']
            duration = step['duration']
            
            # Move to this step's position
            self.move_multiple(pose, int(duration * 500))
            time.sleep(duration)
        
        print(f"✓ Sequence '{sequence_name}' complete!")
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
            print("  2. Gripper test (open → close → neutral)")
            print("  3. Open gripper")
            print("  4. Close gripper")
            
            print("\n[Poses]")
            print("  h. HOME position")
            print("  s. STARE position")
            print("  r. REST position")
            
            print("\n[Sequences]")
            print("  w. WAVE sequence")
            print("  d. DANCE sequence")
            
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
                self.gripper_toggle()
                
            elif choice == '3':
                self.gripper_open()
                
            elif choice == '4':
                self.gripper_close()
            
            # ---- POSES ----
            elif choice == 'h':
                self.execute_pose('home')
                
            elif choice == 's':
                self.execute_pose('stare')
                
            elif choice == 'r':
                self.execute_pose('rest')
            
            # ---- SEQUENCES ----
            elif choice == 'w':
                self.execute_sequence('wave')
                
            elif choice == 'd':
                self.execute_sequence('dance')
            
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
