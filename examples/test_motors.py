#!/usr/bin/python3

import time
from scuttlepy.constants import *
from scuttlepy.motor import Motor

settings = Settings()

leftMotor = Motor(settings.LEFT_WHEEL_MOTOR_PINS)       # Create Left Motor Object (digital, pwm)
rightMotor = Motor(settings.RIGHT_WHEEL_MOTOR_PINS)     # Create Right Motor Object (digital, pwm)

try:

    while True:

        for duty in range(100, -100, -1):
            print(duty)
            leftMotor.setDuty(duty/100)         # Set left motor duty cycle
            rightMotor.setDuty(duty/100)        # Set right motor duty cycle
            time.sleep(0.05)                    # Wait 0.1 seconds

        for duty in range(-100, 100,  1):
            print(duty)
            leftMotor.setDuty(duty/100)         # Set left motor duty cycle
            rightMotor.setDuty(duty/100)        # Set right motor duty cycle
            time.sleep(0.05)                    # Wait 0.1 seconds

except KeyboardInterrupt:

    print('Stopping...')

finally:

    leftMotor.stop()
    rightMotor.stop()
    print('Stopped.')
