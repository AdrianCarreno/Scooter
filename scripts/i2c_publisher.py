#!/usr/bin/python

import rospy
from scooter.msg import i2c

TRHOTTLE = 0x10
DIRECTION = 0x11

data = i2c()
data.throttle_addr = TRHOTTLE
data.direction_addr = DIRECTION

def talker():
    pub = rospy.Publisher('actuation', i2c, queue_size=10)
    rospy.init_node('i2c_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data.throttle_val = 128
        data.direction_val = 128
        pub.publish(data)
        rospy.loginfo('Throttle addr: %s, throttle val: %s', str(data.throttle_addr), str(data.throttle_val))
        rospy.loginfo('Direction addr: %s, direction val: %s', str(data.direction_addr), str(data.direction_val))    
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
