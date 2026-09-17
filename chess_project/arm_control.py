#!/usr/bin/env python3
import time
import config
from Arm_Lib import Arm_Device

class ArmControl:
    def __init__(self):
        print("[Hardware] Connecting to DOFBOT...")
        self.bus = Arm_Device()
        time.sleep(0.1)
        current = self.read_all_joints()
        print(f"[Hardware] Actual current joint angles: {current}")

    def set_servo(self, id, angle, speed):
        """Writes a target angle to one servo."""
        duration = max(100, int(2000 - (speed * 1.5)))
        self.bus.Arm_serial_servo_write(id, int(angle), duration)

    def read_servo(self, id):
        """Reads one servo's actual physical angle."""
        angle = self.bus.Arm_serial_servo_read(id)
        return 0 if angle is None else int(angle)

    def read_all_joints(self):
        """Reads all 6 servos' actual current angles."""
        return [self.read_servo(i) for i in range(1, 7)]

    def move_to_joints(self, joint_array, speed):
        """Moves all 6 joints to the given angles together."""
        duration = max(100, int(2000 - (speed * 1.5)))
        for i in range(6):
            self.bus.Arm_serial_servo_write(i + 1, int(joint_array[i]), duration)
        time.sleep(duration / 1000.0)