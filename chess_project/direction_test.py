#!/usr/bin/env python3
"""
Direction test: nudges one servo at a time from straight up,
asks you which way it moved, then prints a summary.
"""
import time
from Arm_Lib import Arm_Device

UPRIGHT = [90, 90, 90, 90, 90, 90]
NUDGE = 20  # degrees, small and safe

TESTS = [
    (1, "base",     "Did it turn LEFT or RIGHT? (stand behind the arm, facing the board)", {"l": "left", "r": "right"}),
    (2, "shoulder", "Did the arm lean TOWARD the board or AWAY?", {"t": "toward", "a": "away"}),
    (3, "elbow",    "Did the arm lean TOWARD the board or AWAY?", {"t": "toward", "a": "away"}),
    (4, "servo 4",  "Did the top part lean TOWARD the board or AWAY?", {"t": "toward", "a": "away"}),
]

def move_all(arm, angles, duration_ms):
    for i, a in enumerate(angles):
        arm.Arm_serial_servo_write(i + 1, a, duration_ms)
    time.sleep(duration_ms / 1000 + 0.3)

def ask(prompt, options):
    keys = "/".join(options.keys())
    while True:
        answer = input(f"{prompt} [{keys}]: ").strip().lower()
        if answer in options:
            return options[answer]
        print(f"Type one of: {keys}")

def main():
    arm = Arm_Device()
    time.sleep(0.1)
    arm.Arm_serial_set_torque(1)

    print("Moving to straight up (all 90). Keep hands clear.")
    input("Press [Enter] to start...")
    move_all(arm, UPRIGHT, 1500)

    results = {}
    for servo_id, name, question, options in TESTS:
        print(f"\n--- Servo {servo_id} ({name}): going from 90 to {90 + NUDGE} ---")
        input("Watch the arm, press [Enter] to move...")
        arm.Arm_serial_servo_write(servo_id, 90 + NUDGE, 1000)
        time.sleep(1.3)
        results[servo_id] = ask(question, options)
        arm.Arm_serial_servo_write(servo_id, 90, 1000)
        time.sleep(1.3)

    print("\n=== RESULTS (paste this to Claude) ===")
    for servo_id, name, _, _ in TESTS:
        print(f"Servo {servo_id} ({name}): increasing = {results[servo_id]}")

if __name__ == "__main__":
    main()
