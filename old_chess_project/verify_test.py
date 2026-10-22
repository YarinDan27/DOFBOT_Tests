#!/usr/bin/env python3
"""
Verify test: checks what we already know before the rebuild.
1) Center check: all servos to 90, read them back, you confirm it looks straight.
2) Direction check: small SAFE moves only (toward the board).
"""
import time
from Arm_Lib import Arm_Device

NUDGE = 15

# servo, name, safe move, what you should see
DIRECTION_CHECKS = [
    (1, "base",     +NUDGE, "turn LEFT (from behind the arm, facing the board)"),
    (2, "shoulder", -NUDGE, "lean TOWARD the board"),
    (3, "elbow",    -NUDGE, "lean TOWARD the board"),
    (4, "servo 4",  -NUDGE, "tip TOWARD the board"),
]

def read(arm, servo_id):
    for _ in range(3):
        value = arm.Arm_serial_servo_read(servo_id)
        if value is not None:
            return value
        time.sleep(0.05)
    return None

def yes_no(prompt):
    while True:
        answer = input(prompt + " [y/n]: ").strip().lower()
        if answer in ("y", "n"):
            return answer == "y"

def main():
    arm = Arm_Device()
    time.sleep(0.1)
    arm.Arm_serial_set_torque(1)
    report = []

    print("Step 1: all servos to 90 (straight up). Keep hands clear.")
    input("Press [Enter] to move...")
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1500)
    time.sleep(2.0)

    print("\nCommanded 90, servos report:")
    for servo_id in range(1, 7):
        value = read(arm, servo_id)
        print(f"  servo {servo_id}: {value}")
        report.append(f"Servo {servo_id} at 90 reads: {value}")

    straight = yes_no("\nDoes the arm look perfectly straight up?")
    report.append(f"Looks straight at all-90: {'yes' if straight else 'NO'}")
    if not straight:
        off = input("Which servo(s) look off, and which way? (e.g. '2 away'): ").strip()
        report.append(f"  Off: {off}")

    print("\nStep 2: direction checks (safe side only).")
    for servo_id, name, delta, expect in DIRECTION_CHECKS:
        target = 90 + delta
        print(f"\nServo {servo_id} ({name}): 90 -> {target}. It should {expect}.")
        input("Press [Enter] to move...")
        arm.Arm_serial_servo_write(servo_id, target, 800)
        time.sleep(1.2)
        actual = read(arm, servo_id)
        ok = yes_no("Did it move that way?")
        report.append(f"Servo {servo_id} ({name}): sent {target}, reads {actual}, correct direction: {'yes' if ok else 'NO'}")
        arm.Arm_serial_servo_write(servo_id, 90, 800)
        time.sleep(1.2)

    print("\n=== RESULTS (paste this to Claude) ===")
    for line in report:
        print(line)

if __name__ == "__main__":
    main()
