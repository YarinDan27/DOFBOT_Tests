#!/usr/bin/env python3
"""
DOFBOT Interactive Motor & Claw Test Script
Simple interface to test all motors and gripper
"""

import time
import sys

try:
    from Arm_Lib import Arm_Device
    ARM_LIB_AVAILABLE = True
except ImportError:
    print("ERROR: Arm_Lib not found!")
    print("Make sure you're running on the DOFBOT/Jetson system.")
    sys.exit(1)

class MotorTester:
    def __init__(self):
        """Initialize the arm"""
        print("Initializing DOFBOT arm...")
        self.arm = Arm_Device()
        time.sleep(1)
        
        # Motor mapping
        self.motors = {
            '1': 'base',
            '2': 'shoulder', 
            '3': 'elbow',
            '4': 'wrist_pitch',
            '5': 'wrist_roll',
            '6': 'gripper'
        }
        
        print("Arm initialized successfully!")
        
    def move_servo(self, servo_id, angle, speed=1000):
        """Move a single servo"""
        print(f"Moving servo {servo_id} to {angle}° at speed {speed}ms")
        self.arm.Arm_serial_servo_write(servo_id, angle, speed)
        
    def home_position(self):
        """Move all servos to center position"""
        print("Moving to HOME position (all servos to 90°)...")
        for i in range(1, 7):
            self.arm.Arm_serial_servo_write(i, 90, 1000)
        time.sleep(1)
        print("HOME position complete!")
        
    def interactive_menu(self):
        """Main interactive menu"""
        print("\n" + "="*50)
        print("DOFBOT Motor & Claw Test - Interactive Menu")
        print("="*50)
        
        while True:
            print("\n--- Main Menu ---")
            print("1. Move to HOME position (all servos to 90°)")
            print("2. Move INDIVIDUAL servo")
            print("3. GRIPPER open")
            print("4. GRIPPER close")
            print("q. QUIT")
            
            choice = input("\nEnter choice: ").strip().lower()
            
            if choice == 'q':
                print("Returning to home position before exit...")
                self.home_position()
                print("Exiting. Goodbye!")
                break
                
            elif choice == '1':
                self.home_position()
                
            elif choice == '2':
                print("\nMotor IDs:")
                for num, name in self.motors.items():
                    print(f"  {num} = {name}")
                    
                try:
                    servo = int(input("Enter servo ID (1-6): ").strip())
                    if servo < 1 or servo > 6:
                        print("Invalid servo ID!")
                        continue
                        
                    angle = int(input("Enter angle (0-180): ").strip())
                    if angle < 0 or angle > 180:
                        print("Invalid angle!")
                        continue
                        
                    speed = int(input("Enter speed in ms (200-2000, default 1000): ").strip() or "1000")
                    
                    self.move_servo(servo, angle, speed)
                    time.sleep(speed / 1000 + 0.5)
                    
                except ValueError:
                    print("Invalid input!")
                    
            elif choice == '3':
                print("Opening gripper...")
                self.arm.Arm_serial_servo_write(6, 60, 500)
                time.sleep(1)
                
            elif choice == '4':
                print("Closing gripper...")
                self.arm.Arm_serial_servo_write(6, 120, 500)
                time.sleep(1)
                
            else:
                print("Invalid choice! Please try again.")

def main():
    """Main function"""
    print("\n" + "="*50)
    print("DOFBOT Motor & Claw Test Script")
    print("="*50)
    
    try:
        tester = MotorTester()
        tester.interactive_menu()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted! Exiting safely...")
        
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        
    print("\nProgram ended.")

if __name__ == "__main__":
    main()
