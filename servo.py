from adafruit_servokit import ServoKit
from time import sleep
import numpy as np

# Initialize the ServoKit for 16 channels
kit = ServoKit(channels=16)

def forward_servo360(servo_pin):
    kit.continuous_servo[servo_pin].throttle = 1

def backward_servo360(servo_pin):
    kit.continuous_servo[servo_pin].throttle = -1

def stop_servo360(servo_pin):
    kit.continuous_servo[servo_pin].throttle = 0

def configure_servo(servo_pin, angle=270, min_pulse=500, max_pulse=2500):
    kit.servo[servo_pin].actuation_range = angle
    kit.servo[servo_pin].set_pulse_width_range(min_pulse, max_pulse)

# Function to move a servo smoothly between angles
def move_servo_smoothly(servo_pin, start_angle, end_angle, step, delay):
    if start_angle < end_angle:
        for angle in np.arange(start_angle, end_angle + 1, step):
            kit.servo[servo_pin].angle = angle
            print(f"Servo moved to {angle} degrees")
            sleep(delay)
    else:
        for angle in np.arange(start_angle, end_angle - 1, -step):
            kit.servo[servo_pin].angle = angle
            print(f"Servo moved to {angle} degrees")
            sleep(delay)
