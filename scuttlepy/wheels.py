#!/usr/bin/python3

import time
import numpy as np
from collections import deque

from . import motor
from . import encoder
from .constants import *

import RPi.GPIO as GPIO
# from simple_pid import PID

if GPIO.getmode() is None:
    GPIO.setmode(GPIO.BOARD)

class Wheel:

    def __init__(self, motorOutput, bus, encoderAddress, motorPwmFreq, wheelRadius=0.04165, invertMotor=False, invertEncoder=False, KP=0.004, KI=0.025, KD=0, openLoop=True):

        self.invertMotor = invertMotor                                  # Invert motor mode
        self.invertEncoder = invertEncoder                              # Invert encoder mode

        self.motor = motor.Motor(motorOutput,                           # Create motor object
                                 frequency=motorPwmFreq,
                                 invert=invertMotor
                                 )

        self.encoder = encoder.Encoder(encoderAddress,                  # Create encoder object
                                       bus=bus,
                                       invert=self.invertEncoder
                                       )

        self.radius = wheelRadius                                       # meters
        self.pulleyRatio = 15/30                                        # number of teeth on motor pulley / number of teeth on wheel pulley

        self.targetAngularVelocity = 0                                  # Create target angular velocity

        self.position = self.encoder.readPosition()                     # Raw encoder 'ticks'
        self.timestamp = time.monotonic_ns()                            # Stores age of data
        self.targetVelocity = 0                                         # (radians/second)
        self.angularVelocity = 0                                        # (radians/second)
        self.linearVelocity = 0                                         # (meters/second)

        self.rolloverLimit = self.pulleyRatio * self.encoder.resolution                     # limit for rollover
        self._positions = deque([self.position, self.encoder.readPosition()], maxlen=2)     # Create FIFO queue object for wheel positions
        self._timestamps = deque([time.monotonic_ns()]*2, maxlen=2)                         # Create FIFO queue object for wheel positions timestamps
        self.update()                                                                       # Update values in object

    def update(self):
        """_summary_
        """
        self._positions.append(self.encoder.readPosition())             # append new position to _positions queue. This will push out the oldest item in the queue
        self._timestamps.append(time.monotonic_ns())                    # append new timestamp to _timestamps queue. This will push out the oldest item in the queue
        self.position = self._positions[1]                              # Set latest position
        self.timestamp = self._timestamps[1]                            # set timestamp of latest data

    def getRotation(self):                                              # calculate the increment of a wheel in ticks
        """_summary_

        Returns:
            _type_: _description_
        """
        rotation = self._positions[1] - self._positions[0]              # calculate how much wheel has rotated
        if(-rotation >= self.rolloverLimit):                            # if movement is large (has rollover)
            rotation = (rotation + self.encoder.resolution)             # handle forward rollover
        if(rotation >= self.rolloverLimit):                             # if movement is large (has rollover) in the negative direction
            rotation = (rotation - self.encoder.resolution)             # handle reverse rollover
        rotation *= self.pulleyRatio                                    # go from motor pulley to wheel pulley
        return rotation                                                 # return wheel advancement in ticks

    def getTravel(self):                                                                # calculate travel of the wheel in meters
        """_summary_

        Returns:
            _type_: _description_
        """
        rotation = self.getRotation()                                                   # get wheel rotation between measurements
        distance = (2*np.pi*self.radius)*(rotation/self.encoder.resolution)             # calculate distance traveled in wheel rotation
        return distance                                                                 # return distance traveled in meters

    def getLinearVelocity(self):                                                        # get wheel linear velocity
        """_summary_

        Returns:
            _type_: _description_
        """
        distance = self.getTravel()                                                     # get wheel travel
        deltaTime = (self._timestamps[1] - self._timestamps[0])/1e9                     # calculate deltaTime, convert from ns to s
        self.linearVelocity = distance/deltaTime                                        # calculate wheel linear velocity
        return self.linearVelocity                                                      # return wheel linear velocity in meters/second

    def getAngularVelocity(self):                                                                           # get wheel angular velocity
        """_summary_

        Returns:
            _type_: _description_
        """
        rotation = self.getRotation()                                                                       # get wheel rotation between measurements
        deltaTime = (self._timestamps[1] - self._timestamps[0])/1e9                                         # calculate deltaTime, convert from ns to s
        self.angularVelocity = ((rotation * (np.pi/(self.encoder.resolution/2))) / deltaTime)               # speed produced from true wheel rotation (rad)
        return self.angularVelocity                                                                         # returns wheel angular velocity in radians/second

    def setAngularVelocity(self, angularVelocity):                                                          # set wheel angular velocity
        """_summary_

        Args:
            angularVelocity (_type_): _description_
        """
        self.targetAngularVelocity = angularVelocity                                                        # store target angular velocity

        if self.targetAngularVelocity > 0.15:
            duty = (0.098*self.targetAngularVelocity)+0.148
        elif self.targetAngularVelocity < -0.15:
            duty = (0.098*self.targetAngularVelocity)-0.148
        else:
            duty = 0

        self.motor.setDuty(duty)                                        # Set duty cycle to motor

    def stop(self):
        """_summary_
        """
        self.motor.stop()
