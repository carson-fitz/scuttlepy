#!/usr/bin/python3

import RPi.GPIO as GPIO

if GPIO.getmode() is None:
    GPIO.setmode(GPIO.BOARD)

class Motor:

    def __init__(self, pins, frequency=150, invert=False):
        """_summary_

        Args:
            pins (tuple): Digital and PWM Pins to control motor.
            frequency (int, optional): PWM frequency. Defaults to 150.
            invert (bool, optional): Invert motor direction. Defaults to False.
        """
        self.pins = pins                                # First pin will be digital and second pin will be PWM
        self.duty = 0                                   # Initial Duty %
        self.frequency = frequency                      # PWM frequency (Hz)
        self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

        for pin in pins:                                # Set motor pins as outputs
            GPIO.setup(pin, GPIO.OUT)

        self.motor = GPIO.PWM(pins[1], self.frequency)  # Set first pin as PWM and set freq
        self.motor.start(self.duty)

    def setDuty(self, duty):
        """Set motor duty cycle.

        Args:
            duty (float): Duty cycle between -1 and 1.
        """
        self.duty = round(sorted((-1, float(duty), 1))[1], 2)  # Make sure duty is between -1 and 1

        duty = self.duty*100

        GPIO.output(self.pins[0], duty < 0)             # Set direction pin high if duty cycle is negative

        if duty == 0:
            GPIO.output(self.pins[0], False)
            self.motor.ChangeDutyCycle(0)

        else:
            self.motor.ChangeDutyCycle(duty if (duty > 0) else abs(100+duty))

    def stop(self):
        """Stop the motor.
        """
        GPIO.output(self.pins[0], False)
        self.motor.stop()
        GPIO.cleanup(self.pins)
