#!/usr/bin/env python

import rospy
from scooter.msg import actuation, reading
import smbus

class I2C:
    def __init__(self):
        self.ADDR_DIR = rospy.get_param('/i2c/address_direction')
        self.ADDR_THROTTLE = rospy.get_param('/i2c/address_throttle')

        rospy.init_node('i2c', anonymous=True)
        rospy.Subscriber('actuation', actuation, self.callback)
        self.BUS = smbus.SMBus(1)

        # Check if bus is respondig
        try:
            self.BUS.read_byte(self.ADDR_THROTTLE)
            rospy.loginfo('Conected to throttle device on address %s', "0x{:02x}".format(self.ADDR_THROTTLE))
        except IOError:
            rospy.logerr("Can't open I2C device address %s", "0x{:02x}".format(self.ADDR_THROTTLE))
            rospy.signal_shutdown("Can't open I2C device address " + "0x{:02x}".format(self.ADDR_THROTTLE))
        try:
            self.BUS.read_byte(self.ADDR_DIR)
            rospy.loginfo('Conected to direction device on address %s', "0x{:02x}".format(self.ADDR_DIR))
        except IOError:
            rospy.logerr("Can't open I2C device address %s", "0x{:02x}".format(self.ADDR_DIR))
            rospy.signal_shutdown("Can't open I2C device address " + "0x{:02x}".format(self.ADDR_DIR))
    
    def signed_byte(self, x):
        if (x >> 7):
            return -(256 + (~x + 1))
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
        try:
            self.BUS.write_byte(self.ADDR_THROTTLE, dataIn.throttle)
        except IOError:
            rospy.logerr("Can't send data to I2C device address %s", "0x{:02x}".format(self.ADDR_THROTTLE))
        try:
            self.BUS.write_byte(self.ADDR_DIR, dataIn.direction)
        except IOError:
            rospy.logerr("Can't send data to I2C device address %s", "0x{:02x}".format(self.ADDR_DIR))

    def run(self):
        rate = rospy.Rate(rospy.get_param('/readings/sampling_frequency'))
        pub = rospy.Publisher('readings', reading, queue_size=10)
        rospy.loginfo('Publisher initialized correctly')
        data = reading()

        while not rospy.is_shutdown():
            try:
                direction = self.signed_byte(self.BUS.read_byte(self.ADDR_DIR))
                data.dir = self.translate(direction, -127, 127, -135, 135)
                pub.publish(data)
                rate.sleep()
            except IOError:
                rospy.logerr("Can't read data from I2C device address %s", "0x{:02x}".format(self.ADDR_DIR))

if __name__ == '__main__':
    try:
        i2c_node = I2C()
        i2c_node.run()
    except rospy.ROSInterruptException:
        pass
