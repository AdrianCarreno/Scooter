#!/usr/bin/python

from math import pi
from spidev import SpiDev
from datetime import datetime


NOP_A5 = 0x00
RD_POS = 0x10
SET_ZERO_POINT = 0x70


class AMT20():
    def __init__(self, address, direction, bitrate=976000, theta_max=0.15):
        """
        Initialize and open an SPI connection with a CUI AMT20 encoder.
        :param bus: SPI address in the bus (e.g. 0 or 1).
        :type bus: int
        :param direction: Direction of rotation of the encoder, must be either 'ccw' or 'cw'.
        :type direction: string
        :param bitrate: Frequency of the SPI bus. The encoder defaults to 976[kHz].
        :type bitrate: int
        :param bitrate: Max angle variation between samples, in radians. Defaults to 0.15[rad/sample].
        :type bitrate: float
        """
        assert address == 0 or address == 1, "SPI address must be 0 or 1"
        assert direction == "ccw" or direction == "cw", "Direction must be either 'ccw' or 'cw'"
        self.RESOLUTION = 4096
        self.direction = direction
        self.theta_max = theta_max

        self.encoder = SpiDev()
        self.encoder.open(0, address)
        self.encoder.max_speed_hz = bitrate

        self.last = {
            "angle": self.angle(),
            "time": datetime.now()
        }

    def read_position(self):
        """
        Perform a series of SPI transactions, sending the 'RD_POS' command
        and waiting for the response. Then concatenates both bytes received.
        """
        resp = self.encoder.xfer([RD_POS], 0, 20)[0]
        if resp == 0:
            raise IOError("Remote I/O error")

        while resp != RD_POS:
            resp = self.encoder.xfer([NOP_A5], 0, 20)[0]

        MSB = self.encoder.xfer([NOP_A5], 0, 20)[0]  # Most Significant Byte
        LSB = self.encoder.xfer([NOP_A5], 0, 20)[0]  # Least Significant Byte

        position = (MSB << 8) | LSB

        if self.direction == "ccw":
            return position
        else:
            return self.RESOLUTION - position

    def set_zero(self):
        """
        Perform a series of SPI transactions to set the current position
        as zero and save this setting in the EEPROM. 

        The encoder must be power cycled. Otherwise the position values will
        not be calculated off the latest zero position. When the encoder is
        powered on next the new offset will be used for the position calculation.
        """
        resp = self.encoder.xfer([SET_ZERO_POINT], 0, 20)[0]
        if resp == 0:
            raise IOError("Remote I/O error")

        while resp != 0x80:
            resp = self.encoder.xfer([NOP_A5], 0, 20)[0]

        return True

    def angle(self):
        """
        This command converts the reading of the encoder to a angle in the range [0, 2*pi]
        """
        count = self.read_position()
        return (count * 2 * pi / self.RESOLUTION)

    def angular_speed(self):
        """
        This command calculates the speed using the difference between the 
        last and current angle and their corresponding timestamps 
        """
        # Get current time and angle
        angle = self.angle()
        time = datetime.now()
        # Get angle and time differences
        dTheta = angle - self.last["angle"]
        dTime = time - self.last["time"]
        # Update angle and time for future calls
        self.last["angle"] = angle
        self.last["time"] = time

        # When the encoder crosses zero, the difference will spike,
        # unless we add (or substract) 2pi
        if dTheta > self.theta_max * self.RESOLUTION:
            dTheta -= 2 * pi
        elif dTheta < -self.theta_max * self.RESOLUTION:
            dTheta += 2 * pi
        # Convert timedelta object to float (seconds)
        dt = dTime.seconds + dTime.microseconds * 1E-6

        return dTheta / dt

    def __del__(self):
        """
        Do a cleanup of the opened resources.
        """
        self.encoder.close()
