#!/usr/bin/env python
import rospy
from scooter.msg import i2c
import smbus

BUS = smbus.SMBus(1)

def callback(data):
    BUS.write_byte(data.throttle_addr, data.throttle)
    BUS.write_byte(data.direction_addr, data.direction)
    rospy.loginfo('Throttle addr: %s, throttle val: %s', str(data.throttle_addr), str(data.throttle))
    rospy.loginfo('Direction addr: %s, direction val: %s', str(data.direction_addr), str(data.direction))    

def listener():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber('actuation', i2c, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
