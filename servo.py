from adafruit_servokit import ServoKit
from gpiozero import Button
from signal import pause
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

def move_servo_linear():
    # Configure the button to use Pull-Down mode (False when not pressed, True when pressed)
    button_upper = Button(17, pull_up=False)  # GPIO17
    button_lower = Button(18, pull_up=False)  # GPIO17

    def on_press_upper():
        stop_servo360(4)

    def on_release_upper():
        forward_servo360(4)

    def on_press_lower():
        stop_servo360(4)

    def on_release_lower():
        backward_servo360(4)

    button_upper.when_pressed = on_press_upper
    button_upper.when_released = on_release_upper
    button_lower.when_pressed = on_press_lower
    button_lower.when_released = on_release_lower

    pause()  # Wait for the program to run continuously


# Function to configure servo parameters
def configure_servo(servo_pin,angle=270,min_pulse=500,max_pulse=2500):
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

def stop_servo_at_angle(self, servo_pin):
    # Stop the servo and maintain its current position
    kit.servo[servo_pin].throttle = 0

    # Move the servo step by step
    # move_servo_smoothly(1, 0, 270, 10, 0.1)  # Forward from 0° to 270°
    # move_servo_smoothly(2, 270, 0, 10, 0.1)  # Backward from 270° to 0°


