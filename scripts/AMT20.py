#!/usr/bin/python

from math import pi
from spidev import SpiDev
from datetime import datetime

class AMT20():
    def __init__(self, address, direction, bitrate=976000):
        assert address == 0 or address == 1, 'SPI address must be 0 or 1'
        assert direction == 'ccw' or direction == 'cw', 'Direction must be either "ccw" or "cw"'
        self._NOP_A5 = 0x00
        self._RD_POS = 0x10
        self._SET_ZERO_POINT = 0x70
        self.RESOLUTION = 4096
        self.direction = direction

        self.encoder = SpiDev()
        self.encoder.open(0, address)
        self.encoder.max_speed_hz = bitrate

        self.last = {
            'angle': self.angle(),
            'time': datetime.now()
        }

    # This command causes a read of the current position.
    def read_position(self):
        resp = self.encoder.xfer([self._RD_POS], 0, 20)[0]
        if resp == 0:
            raise IOError('Remote I/O error')

        while resp != self._RD_POS:
            resp = self.encoder.xfer([self._NOP_A5], 0, 20)[0]

        MSB = self.encoder.xfer([self._NOP_A5], 0, 20)[0]  # Most Significant Byte
        LSB = self.encoder.xfer([self._NOP_A5], 0, 20)[0]  # Least Significant Byte

        position = (MSB << 8) | LSB

        if self.direction == 'ccw':
            return position
        else:
            return self.RESOLUTION - position

    # This command sets the current position to zero and saves this setting in the EEPROM.
    def set_zero(self):
        resp = self.encoder.xfer([self._SET_ZERO_POINT], 0, 20)[0]
        if resp == 0:
            raise IOError('Remote I/O error')

        while resp != 0x80:
            resp = self.encoder.xfer([self._NOP_A5], 0, 20)[0]

        # The encoder must be power cycled. If the encoder is not power cycled, the position
        # values will not be calculated off the latest zero position. When the encoder is
        # powered on next the new offset will be used for the position calculation.

        return True

    # This command converts the reading of the encoder to a angle in the range [0, 2*pi]
    def angle(self):
        count = self.read_position()
        return (count * 2 * pi / self.RESOLUTION)

    # This command calculates the speed using last and current angle
    def angular_speed(self):
        # Get current time and angle
        angle = self.angle()
        time =  datetime.now()
        # Get angle and time differences
        dTheta = angle - self.last['angle']
        dTime = time - self.last['time']
        # Update angle and time for future calls
        self.last['angle'] = angle
        self.last['time'] = time

        # When the encoder crosses zero, the difference will spike, 
        # unless we add (or substract) 2pi
        if dTheta > 0.9 * self.RESOLUTION:
             dTheta -= 2 * pi
        elif dTheta < -0.9 * self.RESOLUTION:
            dTheta += 2 * pi
        # Convert timedelta object to float (seconds)
        dt = dTime.seconds + dTime.microseconds * 1E-6

        return dTheta / dt

    # This method does a cleanup of the opened resources
    def __del__(self):
        self.encoder.close()

