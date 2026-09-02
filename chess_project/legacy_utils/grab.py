from Arm_Lib import Arm_Device
import time

Arm = Arm_Device()

def grab_seq():
	print("Starting to grab objects!")
	Arm.Arm_serial_servo_write6(90, 90, 90, 90, 30, 90, 1)
	time.sleep(1)

	Arm.Arm_serial_servo_write(1,120,100)
	time.sleep(1)

	Arm.Arm_serial_servo_write(5,60,500)
	time.sleep(1)

	Arm.Arm_serial_servo_write(6,80,10)
	time.sleep(1)

print("Done")
grab_seq()
