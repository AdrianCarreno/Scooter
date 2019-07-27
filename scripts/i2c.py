#!/usr/bin/env python

import rospy
from scooter.msg import i2c, raw_reading
import smbus

ADDR_DIR = rospy.get_param('/i2c/direction/address')
ADDR_THROTTLE = rospy.get_param('/i2c/throttle/address')

BUS = smbus.SMBus(1)
dataOut = raw_reading()

def signed_byte(x):
    if (x >> 7):
        return -(256 + (~x + 1))
    else:
        return x

def callback(data):
    try:
        BUS.write_byte(ADDR_THROTTLE, data.throttle)
    except IOError:
        rospy.logerr("Can't send data to I2C device address %s", "0x{:02x}".format(ADDR_THROTTLE))
    try:
        BUS.write_byte(ADDR_DIR, data.direction)
    except IOError:
        rospy.logerr("Can't send data to I2C device address %s", "0x{:02x}".format(ADDR_DIR))

def listener():
    rospy.init_node('i2c', anonymous=True)
    pub = rospy.Publisher('raw_readings', raw_reading, queue_size=10)
    rate = rospy.Rate(rospy.get_param('/readings/sampling_frequency'))
    rospy.Subscriber('actuation', i2c, callback)
    
    while not rospy.is_shutdown():
        try:
            dataOut.pot = signed_byte(BUS.read_byte(ADDR_DIR))
            pub.publish(dataOut)
            rate.sleep()
        except IOError:
            rospy.logerr("Can't read data from I2C device address %s", "0x{:02x}".format(ADDR_DIR))

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
