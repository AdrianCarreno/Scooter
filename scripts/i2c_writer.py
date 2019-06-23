#!/usr/bin/env python
import rospy
from scooter.msg import i2c
import smbus

BUS = smbus.SMBus(1)

def callback(data):
    try:
        BUS.write_byte(data.throttle_addr, data.throttle)
    except IOError:
        rospy.logerr("Can't communicate with I2C device address %s", "0x{:02x}".format(data.throttle_addr))
    try:
        BUS.write_byte(data.direction_addr, data.direction)
    except IOError:
        rospy.logerr("Can't communicate with I2C device address %s", "0x{:02x}".format(data.direction_addr))

def listener():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber('actuation', i2c, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
