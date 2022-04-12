#!/usr/bin/python3

import os
import threading
import time

import numpy as np
import RPi.GPIO as GPIO
import yaml

from . import wheels
from .constants import *

# from simple_pid import PID

class SCUTTLE:

    def __init__(self, config=None, openLoop=True):
        """_summary_

        Args:
            config (_type_, optional): _description_. Defaults to None.
            openLoop (bool, optional): _description_. Defaults to True.
        """
        GPIO.setmode(GPIO.BOARD)

        settings = Settings(file=config)

        self.heading = 0
        self.velocity = 0
        self.angularVelocity = 0
        self.globalPosition = [0, 0]

        self.headingOffset = 0

        self.wheelBase = settings.WHEEL_BASE                             # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = settings.WHEEL_RADIUS                         # R - meters


        self.maxVelocity = settings.MAXIMUM_LINEAR_VELOCITY             # Maximum linear velocity (m/s)
        self.maxAngularVelocity = settings.MAXIMUM_ANGULAR_VELOCITY     # Maximum angular velocity (rad/s)

        self.leftWheel  = wheels.Wheel(settings.LEFT_WHEEL_MOTOR_PINS,                          # Create left wheel object
                                       settings.I2C_BUS,
                                       settings.LEFT_WHEEL_ENCODER_ADDRESS,
                                       settings.MOTOR_PWM_FREQUENCY,
                                       invertEncoder=settings.LEFT_WHEEL_ENCODER_INVERT,
                                       invertMotor=settings.LEFT_WHEEL_MOTOR_INVERT,
                                       openLoop=settings.OPENLOOP,
                                       )

        self.rightWheel = wheels.Wheel(settings.RIGHT_WHEEL_MOTOR_PINS,                         # Create right wheel object
                                       settings.I2C_BUS,
                                       settings.RIGHT_WHEEL_ENCODER_ADDRESS,
                                       settings.MOTOR_PWM_FREQUENCY,
                                       invertEncoder=settings.RIGHT_WHEEL_ENCODER_INVERT,
                                       invertMotor=settings.RIGHT_WHEEL_MOTOR_INVERT,
                                       openLoop=settings.OPENLOOP,
                                       )

        self.wheelSpeeds          = [0, 0]                      # [Left wheel speed, Right wheel speed.]
        self.targetMotion         = [0, 0]
        self._loopStart           = time.monotonic_ns()         # Updates when we grab chassis displacements
        self._timeInitial         = time.monotonic_ns()
        self._timeFinal           = 0
        self._angularDisplacement = 0                           # For tracking displacement between waypoints
        self._forwardDisplacement = 0                           # For tracking displacement between waypoints
        self._wheelIncrements     = np.array([0, 0])            # Latest increments of wheels

        self.stopped = False

        self._loopFreq = 50                                             # Target Wheel Loop frequency (Hz)
        self._wait = 1/self._loopFreq                                   # Corrected wait time between encoder measurements (s)

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)   # Create wheel loop thread object
        self.wheelsThread.start()                                       # Start wheel loop thread object

    def sleep(self, startTime):
        """_summary_

        Args:
            startTime (_type_): _description_
        """
        time.sleep(sorted([self._wait-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time

    def _wheelsLoop(self):

        while not self.stopped:

            startTime = time.monotonic_ns()                                 # Record loop start time

            self.leftWheel.update()                                         # Update left wheel readings
            self.rightWheel.update()                                        # Update right wheel readings

            self.velocity, self.angularVelocity = self.getMotion()          # Get scuttle linear and angular velocities

            leftWheelTravel = self.leftWheel.getTravel()                    # Get left wheel travel
            rightWheelTravel = self.rightWheel.getTravel()                  # Get right wheel travel

            wheelbaseTravel = (leftWheelTravel + rightWheelTravel)/2        # Calculate wheel displacement

            self.globalPosition = [self.globalPosition[0]+(wheelbaseTravel*np.cos(self.heading)),   # Calculate global X position
                                   self.globalPosition[1]+(wheelbaseTravel*np.sin(self.heading))    # Calculate global Y position
                                   ]

            self.setHeading(self.heading + ((rightWheelTravel - leftWheelTravel)/(self.wheelBase))) # Calculate and update global heading

            self.sleep(startTime)
            # print((time.monotonic_ns()-startTime)/1e6)        # Print loop time in ms

        self.rightWheel.stop()                                  # Once wheels thread loop has broken, stop right wheel
        self.leftWheel.stop()                                   # Once wheels thread loop has broken, stop left wheel

    def stop(self):                                             # Stop SCUTTLE
        """_summary_
        """
        self.setMotion([0, 0])                                  # Set linear and angular velocity to 0
        self.stopped = True                                     # Set stopped flag to True
        self.wheelsThread.join()                                # Wait for the wheels thread to stop

    def setGlobalPosition(self, pos):                           # Set global position
        """_summary_

        Args:
            pos (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.globalPosition = pos                               # Set global position to desired position
        return self.globalPosition                              # return new global position

    def offsetHeading(self, offset):                           # offset global heading
        """_summary_

        Args:
            offset (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.headingOffset = offset
        heading = self.heading + self.headingOffset
        if heading < -np.pi:                                    # Keep heading within -pi to pi, [-180, 180] degrees
            heading += 2 * np.pi
        elif heading > np.pi:
            heading -= 2 * np.pi
        self.setHeadin(heading)                                 # Set heading to heading with offset
        return self.heading                                     # return new global heading

    def setHeading(self, heading):                              # set global heading
        """_summary_

        Args:
            heading (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.heading = heading
        return self.heading                                     # return new global heading

    def getGlobalPosition(self):                                # get global position
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.globalPosition                              # return global position

    def getHeading(self):                                       # get global heading
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.heading                                     # return global heading

    def getLinearVelocity(self):                                # get linear velocity
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.velocity                                    # return linear velocity

    def getAngularVelocity(self):                               # get angular velocity
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.angularVelocity                             # return angular velocity

    def setLinearVelocity(self, linearVelocity):                # set linear velocity
        """_summary_

        Args:
            linearVelocity (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.targetMotion[0] = linearVelocity
        self.setMotion(self.targetMotion)
        return self.targetMotion                                # return linear velocity

    def setAngularVelocity(self, angularVelocity):              # set angular velocity
        """_summary_

        Args:
            angularVelocity (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.targetMotion[1] = angularVelocity
        self.setMotion(self.targetMotion)
        return self.targetMotion                                # return angular velocity

    def setMotion(self, targetMotion):                          # Take chassis speed and command wheels
        """_summary_

        Args:
            targetMotion (_type_): _description_
        """
                                                                # argument: [x_dot, theta_dot]
        self.targetMotion = targetMotion

        L = self.wheelBase/2
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                             # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([targetMotion[0],                          # Create an array for chassis speed
                      targetMotion[1]])

        C = np.matmul(A, B)                                     # Perform matrix multiplication

        self.leftWheel.setAngularVelocity(C[0])                 # Set angularVelocity = [rad/s]
        self.rightWheel.setAngularVelocity(C[1])                # Set angularVelocity = [rad/s]

    def getMotion(self):                                        # Forward Kinematics
                                                                # Function to update and return [x_dot,theta_dot]
        """_summary_
        """
        L = self.wheelBase/2
        R = self.wheelRadius

        A = np.array([[     R/2,     R/2],                      # This matrix relates [PDL, PDR] to [XD,TD]
                      [-R/(2*L), R/(2*L)]])

        B = np.array([self.leftWheel.getAngularVelocity(),      # make an array of wheel speeds (rad/s)
                      self.rightWheel.getAngularVelocity()])

        C = np.matmul(A, B)                                     # perform matrix multiplication

        self.velocity = C[0]                                    # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                             # Update angularVelocity = [rad/s]

        return [self.velocity, self.angularVelocity]            # return [speed, angularVelocity]
