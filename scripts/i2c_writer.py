#!/usr/bin/env python

import rospy
from scooter.msg import i2c, reading
import smbus

DIR_ADDR = 0x11

BUS = smbus.SMBus(1)
dataOut = reading()

def signed_byte(x):
    if (x >> 7):
        return -(256 + (~x + 1))
    else:
        return x

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
    pub = rospy.Publisher('reading', reading, queue_size=10)
    rate = rospy.Rate(1000)  # 1000hz
    rospy.Subscriber('actuation', i2c, callback)
    
    while not rospy.is_shutdown():
        try:
            dataOut.pot = signed_byte(BUS.read_byte(DIR_ADDR))
            pub.publish(dataOut)
            rate.sleep()
        except IOError:
            rospy.logerr("Can't communicate with I2C device address %s", "0x{:02x}".format(DIR_ADDR))

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
