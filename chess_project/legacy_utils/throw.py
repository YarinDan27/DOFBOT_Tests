#!/usr/bin/env python3
"""
DOFBOT 6-Motor Control System using Arm_Lib
This script provides comprehensive control for all 6 servo motors on the DOFBOT
using the official Yahboom Arm_Lib library.

Motor Configuration:
- Motor 1: Base rotation (yaw)
- Motor 2: Shoulder (lift arm up/down)
- Motor 3: Elbow (bend arm)
- Motor 4: Wrist pitch (tilt end effector up/down)
- Motor 5: Wrist roll (rotate end effector)
- Motor 6: Gripper (open/close)
"""

import time
import math
try:
    from Arm_Lib import Arm_Device
    ARM_LIB_AVAILABLE = True
except ImportError:
    print("Warning: Arm_Lib not found. Make sure you're running on the DOFBOT system.")
    print("Running in simulation mode...")
    ARM_LIB_AVAILABLE = False
    
    # Mock Arm_Device for testing
    class Arm_Device:
        def __init__(self):
            print("Mock Arm_Device initialized")
        
        def Arm_serial_servo_write(self, servo_id, angle, time_ms):
            print(f"SIM: Servo {servo_id} -> {angle}° in {time_ms}ms")
        
        def Arm_serial_servo_read(self, servo_id):
            return 90  # Return center position

class DOFBOTController:
    def __init__(self):
        """Initialize the DOFBOT controller using Arm_Lib"""
        self.arm = Arm_Device() if ARM_LIB_AVAILABLE else Arm_Device()
        
        # Motor ID mapping (1-based indexing as per Arm_Lib)
        self.motors = {
            'base': 1,      # Base rotation
            'shoulder': 2,  # Shoulder joint
            'elbow': 3,     # Elbow joint
            'wrist_pitch': 4, # Wrist pitch
            'wrist_roll': 5,  # Wrist roll
            'gripper': 6    # Gripper
        }
        
        # Safe angle limits for each motor (in degrees)
        # Adjust these based on your robot's physical limits
        self.angle_limits = {
            'base': {'min': 0, 'max': 180, 'center': 90},
            'shoulder': {'min': 0, 'max': 180, 'center': 90},
            'elbow': {'min': 0, 'max': 180, 'center': 90},
            'wrist_pitch': {'min': 0, 'max': 180, 'center': 90},
            'wrist_roll': {'min': 0, 'max': 180, 'center': 90},
            'gripper': {'min': 60, 'max': 120, 'center': 90}  # Gripper has smaller range
        }
        
        # Current positions
        self.current_positions = {motor: 90 for motor in self.motors.keys()}
        
        print("DOFBOT Controller initialized with Arm_Lib")
    
    def move_servo(self, motor_name, angle, duration_ms=1000):
        """
        Move a single servo motor
        
        Args:
            motor_name (str): Motor name ('base', 'shoulder', etc.)
            angle (int): Target angle (0-180 degrees)
            duration_ms (int): Movement duration in milliseconds
        """
        if motor_name not in self.motors:
            print(f"Unknown motor: {motor_name}")
            return False
        
        motor_id = self.motors[motor_name]
        limits = self.angle_limits[motor_name]
        
        # Clamp angle to safe limits
        angle = max(limits['min'], min(limits['max'], angle))
        
        # Execute movement
        self.arm.Arm_serial_servo_write(motor_id, angle, duration_ms)
        self.current_positions[motor_name] = angle
        
        print(f"Moving {motor_name} (servo {motor_id}) to {angle}° in {duration_ms}ms")
        return True
    
    def move_multiple_servos(self, motor_angles, duration_ms=1000):
        """
        Move multiple servos simultaneously
        
        Args:
            motor_angles (dict): Dictionary of motor names and target angles
            duration_ms (int): Movement duration in milliseconds
        """
        print(f"Moving multiple servos in {duration_ms}ms:")
        for motor_name, angle in motor_angles.items():
            if self.move_servo(motor_name, angle, duration_ms):
                print(f"  {motor_name}: {angle}°")
        
        # Wait for movement to complete
        time.sleep(duration_ms / 1000.0)
    
    def read_servo_position(self, motor_name):
        """Read current position of a servo"""
        if motor_name not in self.motors:
            return None
        
        motor_id = self.motors[motor_name]
        try:
            position = self.arm.Arm_serial_servo_read(motor_id)
            self.current_positions[motor_name] = position
            return position
        except:
            return self.current_positions[motor_name]
    
    def home_position(self, duration_ms=2000):
        """Move all servos to center/home position"""
        print("Moving to home position...")
        home_angles = {
            'base': 90,
            'shoulder': 90,
            'elbow': 90,
            'wrist_pitch': 90,
            'wrist_roll': 90,
            'gripper': 90
        }
        self.move_multiple_servos(home_angles, duration_ms)
    
    def wave_sequence(self):
        """Make the robot arm wave - a complete waving motion"""
        print("Starting wave sequence...")
        
        # Step 1: Move to initial wave position
        print("Step 1: Moving to wave start position")
        wave_start = {
            'base': 45,        # Turn base to side
            'shoulder': 45,    # Lift shoulder up
            'elbow': 45,       # Bend elbow slightly
            'wrist_pitch': 90, # Keep wrist neutral
            'wrist_roll': 90,  # Keep wrist straight
            'gripper': 90      # Keep gripper neutral
        }
        self.move_multiple_servos(wave_start, 1500)
        time.sleep(0.5)
        
        # Step 2: Wave motion - move wrist back and forth
        print("Step 2: Waving motion")
        for i in range(3):  # Wave 3 times
            # Wave up
            self.move_servo('wrist_pitch', 60, 500)  # Tilt wrist up
            time.sleep(0.5)
            
            # Wave down
            self.move_servo('wrist_pitch', 120, 500)  # Tilt wrist down
            time.sleep(0.5)
        
        # Return wrist to neutral
        self.move_servo('wrist_pitch', 90, 500)
        time.sleep(0.5)
        
        # Step 3: Alternative wave - rotate base while keeping arm up
        print("Step 3: Base rotation wave")
        for i in range(2):
            self.move_servo('base', 20, 800)   # Swing left
            time.sleep(0.8)
            self.move_servo('base', 70, 800)   # Swing right
            time.sleep(0.8)
        
        # Step 4: Return to home
        print("Step 4: Returning to home position")
        self.home_position(2000)
        
        print("Wave sequence complete!")
    
    def advanced_wave(self):
        """More sophisticated waving motion"""
        print("Starting advanced wave sequence...")
        
        # Prepare for wave
        prep_position = {
            'base': 60,
            'shoulder': 60,    # Lift arm higher
            'elbow': 30,       # Straighten elbow more
            'wrist_pitch': 90,
            'wrist_roll': 90,
            'gripper': 90
        }
        self.move_multiple_servos(prep_position, 2000)
        time.sleep(1)
        
        # Complex wave pattern
        wave_positions = [
            {'wrist_pitch': 120, 'wrist_roll': 110},  # Down-right
            {'wrist_pitch': 60, 'wrist_roll': 110},   # Up-right
            {'wrist_pitch': 60, 'wrist_roll': 70},    # Up-left
            {'wrist_pitch': 120, 'wrist_roll': 70},   # Down-left
            {'wrist_pitch': 90, 'wrist_roll': 90},    # Center
        ]
        
        # Execute wave pattern twice
        for cycle in range(2):
            print(f"Wave cycle {cycle + 1}")
            for pos in wave_positions:
                self.move_multiple_servos(pos, 600)
                time.sleep(0.6)
        
        # Final flourish - open and close gripper while waving
        print("Final flourish with gripper")
        for i in range(3):
            self.move_multiple_servos({
                'wrist_pitch': 70,
                'gripper': 110  # Open gripper
            }, 400)
            time.sleep(0.4)
            
            self.move_multiple_servos({
                'wrist_pitch': 110,
                'gripper': 70   # Close gripper
            }, 400)
            time.sleep(0.4)
        
        # Return home
        self.home_position(2000)
        print("Advanced wave complete!")
    
    def throw_sequence(self):
        """Throwing motion - grab, wind back, and throw forward"""
        print("Starting throw sequence...")
        print("WARNING: Make sure the area is clear and Jetson Nano is safe!")
        
        # Step 1: Open gripper and prepare to grab
        print("Step 1: Opening gripper and positioning for grab")
        grab_position = {
            'base': 90,        # Face forward
            'shoulder': 120,   # Lower arm to grab level
            'elbow': 120,      # Bend elbow to reach
            'wrist_pitch': 150, # Point gripper down
            'wrist_roll': 90,  # Keep wrist straight
            'gripper': 120     # Open gripper wide
        }
        self.move_multiple_servos(grab_position, 2000)
        time.sleep(2)
        
        # Step 2: Close gripper to grab object
        print("Step 2: Closing gripper to grab object")
        self.move_servo('gripper', 70, 1000)  # Close gripper firmly
        time.sleep(1.5)
        
        # Step 3: Lift object slightly
        print("Step 3: Lifting object")
        lift_position = {
            'shoulder': 100,   # Lift shoulder slightly
            'elbow': 100,      # Straighten elbow a bit
            'wrist_pitch': 120 # Adjust wrist angle
        }
        self.move_multiple_servos(lift_position, 1000)
        time.sleep(1)
        
        # Step 4: Wind back for throw - this is the key part!
        print("Step 4: Winding back for throw")
        windup_position = {
            'base': 90,        # Keep base forward
            'shoulder': 30,    # Pull shoulder way back (wind up)
            'elbow': 30,       # Straighten arm back
            'wrist_pitch': 60, # Angle wrist back
            'wrist_roll': 90,  # Keep wrist straight
            'gripper': 70      # Keep gripper closed
        }
        self.move_multiple_servos(windup_position, 1500)
        time.sleep(1)
        
        # Step 5: THE THROW! Fast forward motion
        print("Step 5: THROWING! Fast forward motion")
        throw_position = {
            'shoulder': 140,   # Swing shoulder forward fast
            'elbow': 160,      # Extend elbow forward
            'wrist_pitch': 120 # Snap wrist forward
        }
        # Fast movement for the throw!
        self.move_multiple_servos(throw_position, 400)  # Very fast - 400ms
        time.sleep(0.2)  # Brief pause
        
        # Step 6: Release object at peak of throw
        print("Step 6: Releasing object!")
        self.move_servo('gripper', 120, 200)  # Open gripper quickly
        time.sleep(0.3)
        
        # Step 7: Follow through motion
        print("Step 7: Follow through")
        followthrough_position = {
            'shoulder': 150,   # Continue forward motion
            'elbow': 170,      # Full extension
            'wrist_pitch': 140 # Complete wrist snap
        }
        self.move_multiple_servos(followthrough_position, 500)
        time.sleep(1)
        
        # Step 8: Slow return to neutral position
        print("Step 8: Returning to home position")
        self.home_position(3000)  # Slow return
        
        print("Throw sequence complete! Hope it went far!")
        print("Remember to retrieve any thrown objects responsibly!")
    
    def softball_throw(self):
        """Alternative throwing motion - more like a softball pitch"""
        print("Starting softball throw sequence...")
        
        # Assume object is already in gripper
        print("Step 1: Securing grip on object")
        self.move_servo('gripper', 65, 1000)  # Firm grip
        time.sleep(1)
        
        # Step 2: Wind up position - classic pitcher pose
        print("Step 2: Pitcher wind-up position")
        windup = {
            'base': 110,       # Turn slightly to side
            'shoulder': 45,    # Raise arm up and back
            'elbow': 45,       # Bend elbow back
            'wrist_pitch': 45, # Cock wrist back
            'wrist_roll': 90,
            'gripper': 65      # Keep grip
        }
        self.move_multiple_servos(windup, 2000)
        time.sleep(1.5)
        
        # Step 3: The pitch! Fast overhand motion
        print("Step 3: The pitch!")
        pitch = {
            'base': 90,        # Square up to target
            'shoulder': 130,   # Swing arm forward and down
            'elbow': 150,      # Extend arm
            'wrist_pitch': 130 # Snap wrist forward
        }
        self.move_multiple_servos(pitch, 300)  # Very fast pitch
        time.sleep(0.1)
        
        # Step 4: Release at optimal point
        print("Step 4: Release!")
        self.move_servo('gripper', 130, 100)  # Quick release
        time.sleep(0.2)
        
        # Step 5: Follow through
        print("Step 5: Follow through")
        followthrough = {
            'shoulder': 150,
            'elbow': 170,
            'wrist_pitch': 150
        }
        self.move_multiple_servos(followthrough, 400)
        time.sleep(1)
        
        # Return home
        print("Returning to ready position")
        self.execute_pose('ready', 2000)
        
        print("Softball throw complete!")
    
    def predefined_poses(self):
        """Dictionary of useful predefined poses"""
        return {
            'home': {
                'base': 90, 'shoulder': 90, 'elbow': 90,
                'wrist_pitch': 90, 'wrist_roll': 90, 'gripper': 90
            },
            'ready': {
                'base': 90, 'shoulder': 60, 'elbow': 60,
                'wrist_pitch': 90, 'wrist_roll': 90, 'gripper': 90
            },
            'pick_front': {
                'base': 90, 'shoulder': 120, 'elbow': 120,
                'wrist_pitch': 150, 'wrist_roll': 90, 'gripper': 110
            },
            'pick_side': {
                'base': 45, 'shoulder': 110, 'elbow': 110,
                'wrist_pitch': 140, 'wrist_roll': 90, 'gripper': 110
            },
            'rest': {
                'base': 90, 'shoulder': 140, 'elbow': 140,
                'wrist_pitch': 90, 'wrist_roll': 90, 'gripper': 90
            },
            'greeting': {
                'base': 60, 'shoulder': 45, 'elbow': 30,
                'wrist_pitch': 90, 'wrist_roll': 90, 'gripper': 90
            }
        }
    
    def execute_pose(self, pose_name, duration_ms=2000):
        """Execute a predefined pose"""
        poses = self.predefined_poses()
        if pose_name not in poses:
            print(f"Unknown pose: {pose_name}")
            print(f"Available poses: {list(poses.keys())}")
            return
        
        print(f"Executing pose: {pose_name}")
        self.move_multiple_servos(poses[pose_name], duration_ms)
    
    def gripper_control(self, action, duration_ms=1000):
        """
        Control gripper with simple commands
        
        Args:
            action (str): 'open', 'close', or 'neutral'
            duration_ms (int): Movement duration
        """
        gripper_positions = {
            'open': 110,      # Open gripper
            'close': 70,      # Close gripper
            'neutral': 90     # Neutral position
        }
        
        if action in gripper_positions:
            angle = gripper_positions[action]
            self.move_servo('gripper', angle, duration_ms)
            print(f"Gripper: {action} ({angle}°)")
        else:
            print(f"Unknown gripper action: {action}")
    
    def demo_sequence(self):
        """Comprehensive demonstration sequence"""
        print("Starting DOFBOT demonstration...")
        
        # 1. Home position
        print("\n1. Moving to home position")
        self.home_position()
        time.sleep(2)
        
        # 2. Basic wave
        print("\n2. Basic wave sequence")
        self.wave_sequence()
        time.sleep(1)
        
        # 3. Pose demonstration
        print("\n3. Demonstrating poses")
        poses = ['ready', 'greeting', 'pick_front', 'rest']
        for pose in poses:
            print(f"   Executing: {pose}")
            self.execute_pose(pose, 1500)
            time.sleep(1.5)
        
        # 4. Gripper test
        print("\n4. Gripper control test")
        self.gripper_control('open', 1000)
        time.sleep(1)
        self.gripper_control('close', 1000)
        time.sleep(1)
        self.gripper_control('neutral', 1000)
        time.sleep(1)
        
        # 5. Advanced wave
        print("\n5. Advanced wave sequence")
        self.advanced_wave()
        
        # 6. Throwing demonstration (be careful!)
        print("\n6. Throwing motion demonstration")
        user_input = input("Execute throw sequence? This will move the arm rapidly! (y/N): ")
        if user_input.lower() == 'y':
            self.throw_sequence()
        else:
            print("Skipping throw sequence for safety")
        
        # 7. Return home
        print("\n7. Returning to home position")
        self.home_position()
        
        print("\nDemonstration complete!")
    
    def interactive_control(self):
        """Interactive control interface"""
        print("\n=== DOFBOT Interactive Control ===")
        print("Commands:")
        print("  move <motor> <angle>     - Move motor to angle (0-180)")
        print("  pose <name>              - Execute predefined pose")
        print("  grip <open/close/neutral> - Control gripper")
        print("  wave                     - Execute wave sequence")
        print("  advanced_wave            - Execute advanced wave")
        print("  throw                    - Execute throwing sequence (BE CAREFUL!)")
        print("  softball                 - Execute softball pitch motion")
        print("  home                     - Move to home position")
        print("  demo                     - Run full demonstration")
        print("  status                   - Show current positions")
        print("  read <motor>             - Read servo position")
        print("  motors                   - List available motors")
        print("  poses                    - List available poses")
        print("  quit                     - Exit")
        print()
        
        while True:
            try:
                command = input("DOFBOT> ").strip().split()
                
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd in ['quit', 'q', 'exit']:
                    break
                
                elif cmd == 'move':
                    if len(command) >= 3:
                        motor = command[1]
                        try:
                            angle = int(command[2])
                            self.move_servo(motor, angle, 1000)
                        except ValueError:
                            print("Invalid angle value")
                    else:
                        print("Usage: move <motor> <angle>")
                
                elif cmd == 'pose':
                    if len(command) >= 2:
                        pose_name = command[1]
                        self.execute_pose(pose_name)
                    else:
                        print("Usage: pose <name>")
                
                elif cmd == 'grip' or cmd == 'gripper':
                    if len(command) >= 2:
                        action = command[1]
                        self.gripper_control(action)
                    else:
                        print("Usage: grip <open/close/neutral>")
                
                elif cmd == 'wave':
                    self.wave_sequence()
                
                elif cmd == 'advanced_wave':
                    self.advanced_wave()
                
                elif cmd == 'throw':
                    print("⚠️  WARNING: This will move the arm rapidly!")
                    print("   Make sure:")
                    print("   - The area is clear")
                    print("   - Jetson Nano is safely positioned")
                    print("   - No fragile objects nearby")
                    confirm = input("   Continue with throw sequence? (y/N): ")
                    if confirm.lower() == 'y':
                        self.throw_sequence()
                    else:
                        print("   Throw sequence cancelled")
                
                elif cmd == 'softball':
                    print("⚠️  Executing softball throw motion")
                    confirm = input("   Area clear? Continue? (y/N): ")
                    if confirm.lower() == 'y':
                        self.softball_throw()
                    else:
                        print("   Softball throw cancelled")
                
                elif cmd == 'home':
                    self.home_position()
                
                elif cmd == 'demo':
                    self.demo_sequence()
                
                elif cmd == 'status':
                    print("Current positions:")
                    for motor, angle in self.current_positions.items():
                        print(f"  {motor}: {angle}°")
                
                elif cmd == 'read':
                    if len(command) >= 2:
                        motor = command[1]
                        pos = self.read_servo_position(motor)
                        if pos is not None:
                            print(f"{motor} position: {pos}°")
                    else:
                        print("Usage: read <motor>")
                
                elif cmd == 'motors':
                    print("Available motors:")
                    for motor in self.motors.keys():
                        print(f"  {motor}")
                
                elif cmd == 'poses':
                    poses = self.predefined_poses()
                    print("Available poses:")
                    for pose in poses.keys():
                        print(f"  {pose}")
                
                else:
                    print(f"Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
        
        # Return to home position before exit
        print("Returning to home position...")
        self.home_position(1000)
        time.sleep(1)

def main():
    """Main function"""
    print("DOFBOT Controller with Arm_Lib")
    print("==============================")
    
    try:
        # Create controller
        controller = DOFBOTController()
        
        # Start interactive control
        controller.interactive_control()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    
    print("Program ended.")

if __name__ == "__main__":
    main()
