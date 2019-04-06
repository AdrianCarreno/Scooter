#!/usr/bin/env python
import rospy
from scooter.msg import i2c
import smbus

BUS = smbus.SMBus(0)

def callback(data):
    BUS.write_byte(data.addr, data.mode)
    BUS.write_byte(data.addr, data.val)
    rospy.loginfo('addr: %s, mode: %s, val: %s', str(data.addr), str(data.mode), str(data.val))
    
def listener():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber('i2c', i2c, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
