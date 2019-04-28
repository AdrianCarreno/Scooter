#!/usr/bin/env python
import rospy
from scooter.msg import i2c
import smbus

BUS = smbus.SMBus(0)

def callback(data):
    BUS.write_byte(data.throttle_addr, data.throttle_val)
    BUS.write_byte(data.direction_addr, data.direction_val)
    rospy.loginfo('Throttle addr: %s, throttle val: %s', str(data.throttle_addr), str(data.throttle_val))
    rospy.loginfo('Direction addr: %s, direction val: %s', str(data.direction_addr), str(data.direction_val))    

def listener():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber('i2c', i2c, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
