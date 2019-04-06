#!/usr/bin/env python
import rospy
from scooter.msg import i2c

ADDR = 16

def talker():
    pub = rospy.Publisher('i2c', i2c, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data = i2c()
        data.addr = ADDR
        data.mode = 1
        data.val = 128
        pub.publish(data)
        rospy.loginfo('mode: %s, val: %s', str(data.addr), str(data.mode), str(data.val))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
