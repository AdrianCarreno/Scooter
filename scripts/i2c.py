#!/usr/bin/env python

import rospy
from scooter.msg import actuation, angle
import smbus
import numpy as np


class I2C:
    def __init__(self):
        self.ADDR_DIR = rospy.get_param("i2c/address_direction")
        self.ADDR_THROTTLE = rospy.get_param("i2c/address_throttle")

        rospy.init_node("i2c", anonymous=True)
        rospy.Subscriber("actuation", actuation, self.callback)
        self.BUS = smbus.SMBus(1)
        self._bus_in_use = False
        self._init_errors = []

        # Check if bus is respondig
        try:
            self.BUS.write_byte(self.ADDR_THROTTLE, 0)
            rospy.loginfo(
                "Connected to throttle device on address %s", hex(self.ADDR_THROTTLE))
        except IOError:
            self._init_errors.append(self.ADDR_THROTTLE)
        try:
            self.BUS.write_byte(self.ADDR_DIR, 0)
            rospy.loginfo(
                "Connected to direction device on address %s", hex(self.ADDR_DIR))
        except IOError:
            self._init_errors.append(self.ADDR_DIR)

        if len(self._init_errors) != 0:
            for i in self._init_errors:
                rospy.logerr(
                    "Can't open I2C device address %s", hex(i))
            rospy.signal_shutdown(
                "Failed to initiate communications with some I2C devices")

    def signed(self, x, n=8):
        if (x >> (n-1)):
            return -(2**n + (~x + 1))
        else:
            return x

    def translate(self, value, in_min, in_max, out_min, out_max):
        # Figure out how 'wide' each range is
        left_span = in_max - in_min
        right_span = out_max - out_min

        # Convert the left range into a 0-1 range (float)
        value_scaled = float(value - in_min) / float(left_span)

        # Convert the 0-1 range into a value in the right range.
        return out_min + (value_scaled * right_span)

    def callback(self, dataIn):
        while self._bus_in_use:
            pass    # Wait for BUS to be available

        self._bus_in_use = True     # Mark BUS as in use

        try:
            self.BUS.write_byte(self.ADDR_THROTTLE, dataIn.throttle)
        except IOError:
            rospy.logerr(
                "Can't send data to I2C device address %s", hex(self.ADDR_THROTTLE))
        try:
            direction = np.clip(self.translate(
                dataIn.direction, np.deg2rad(-150), np.deg2rad(150), -128, 127), -128, 127)
            self.BUS.write_byte(self.ADDR_DIR, direction)
        except IOError:
            rospy.logerr(
                "Can't send data to I2C device address %s", hex(self.ADDR_DIR))

        self._bus_in_use = False    # Mark BUS as available

    def run(self):
        rate = rospy.Rate(rospy.get_param("readings/sampling_frequency"))
        pub = rospy.Publisher("angle", angle, queue_size=10)
        rospy.loginfo("Publisher initialized correctly")
        data = angle()

        while not rospy.is_shutdown():
            while self._bus_in_use:
                pass    # Wait for BUS to be available

            self._bus_in_use = True     # Mark BUS as in use

            try:
                direction = self.signed(self.BUS.read_byte(self.ADDR_DIR))
            except IOError:
                rospy.logerr(
                    "Can't read data from I2C device address %s", hex(self.ADDR_DIR))
                continue

            self._bus_in_use = False    # Mark BUS as available
            data.angle = self.translate(
                direction, -128, 127, np.deg2rad(-150), np.deg2rad(150))
            pub.publish(data)
            rate.sleep()


if __name__ == "__main__":
    try:
        i2c_node = I2C()
        i2c_node.run()
    except rospy.ROSInterruptException:
        pass
