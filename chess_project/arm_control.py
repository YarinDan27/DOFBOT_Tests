#!/usr/bin/env python3
import time
import config
from Arm_Lib import Arm_Device

class ArmControl:
    def __init__(self):
        print("[Hardware] Initializing physical DOFBOT Serial Bus Connection...")
        self.bus = Arm_Device()
        time.sleep(0.1)
        # Ensure driver starts up matching your baseline observation profiles
        self.move_to_joints(config.POS_CHESS_OBSERVE, config.HOME_SPEED)

    def set_servo(self, id, angle, speed):
        """Writes raw targeted angle packet straight to the physical servo bus."""
        duration = max(100, int(2000 - (speed * 1.5)))
        self.bus.Arm_serial_servo_write(id, int(angle), duration)

    def read_servo(self, id):
        """Reads physical real-world potentiometer location feedback via the bus."""
        angle = self.bus.Arm_serial_servo_read(id)
        if angle is None:
            return 0
        return int(angle)

    def move_to_joints(self, joint_array, speed):
        """Sequences a multi-axis motion frame by writing to each servo in a fast loop."""
        duration = max(100, int(2000 - (speed * 1.5)))
        for i in range(6):
            servo_id = i + 1
            angle = int(joint_array[i])
            self.bus.Arm_serial_servo_write(servo_id, angle, duration)
        time.sleep(duration / 1000.0)
