#!/usr/bin/python3

import numpy as np
from smbus2 import SMBus

class Encoder:

    def __init__(self, address, bus=1, invert=False):
        """testi

        Args:
            address (int): Encoder I2C address. 
            bus (int, optional): Encoder I2C bus. Defaults to 1.
            invert (bool, optional): Invert encoder direction. Defaults to False.
        """

        self.bus = SMBus(bus)                                               # I2C Bus
        self.address = address                                              # Encoder I2C address
        self.invert = invert                                                # Invert encoder direction
        self.resolution = 2**14                                             # Define "ticks", 14 bit encoder
        self.position = self.readPosition()                                 # Read position of encoder
        self.angle = self.readAngle()
        self.magnitude = self.readMagnitude()

    def readPosition(self):
        """Read encoder raw position.

        Returns:
            int: Raw encoder position between 0 and 16384.
        """
        pos = self.bus.read_i2c_block_data(self.address, 0xFE, 2)           # Request data from registers 0xFE & 0xFF of the encoder
                                                                            # Takes ~700 microseconds.
        self.position = (pos[0] << 6) | pos[1]                              # Remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        if self.invert:
            self.position = self.resolution - self.position                 # If encoder is inverted, invert position
        return self.position                                                # Return Raw encoder position (0 to 16384)

    def readAngle(self):
        """Read encoder angle in radians.

        Returns:
            float: Encoder angle in radians.
        """
        self.readPosition()                                                 # Read encoder position
        self.angle = self.position * ((2 * np.pi) / self.resolution)        # Scale values to get radians
        return self.angle                                                   # Return encoder angle in radians

    def readMagnitude(self):
        """Read encoder magnitude.

        Returns:
            int: Encoder magnitude.
        """
        magnitude = self.bus.read_i2c_block_data(self.address, 0xFC, 2)     # Request data from registers 0xFC & 0xFD of the encoder
                                                                            # Takes ~700 microseconds.
        self.magnitude = (magnitude[0] << 6) | magnitude[1]                 # Remove unused bits 6 & 7 from byte 0xFD creating 14 bit value
        return self.magnitude                                               # Return encoder magnitude
