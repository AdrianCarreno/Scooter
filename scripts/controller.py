#!/usr/bin/python

import rospy
import numpy
from scooter.msg import i2c, raw_reading, reading, target
from geometry_msgs.msg import Twist
from kinematic import countsToW

ADDR_DIR = rospy.get_param('/i2c/direction/address')
ADDR_THROTTLE = rospy.get_param('/i2c/throttle/address')
RADIUS = rospy.get_param('/radius')
WRITE_FREQ = rospy.get_param('/actuation/write_frequency')
SAMPLING_FREQ = rospy.get_param('/readings/sampling_frequency')

act = i2c()
act.throttle = 0
act.direction = 0

last1 = 0
last2 = 0

targetV = 0 # Target speed
targetD = 0 # Target direction

def getSpeed(dataIn):
    if rospy.get_param('/spi/enc1/wheel') == 'left':
        v_left = RADIUS * countsToW(last1, dataIn.enc1)
        v_right = RADIUS * countsToW(last2, dataIn.enc2)
    else:
        v_left = RADIUS * countsToW(last2, dataIn.enc2)
        v_right = RADIUS * countsToW(last1, dataIn.enc1)
    last1 = dataIn.enc1
    last2 = dataIn.enc2

def setSpeed(target):
    targetV = target.vel
    targetD = target.dir

def controller():
    rospy.init_node('controller', anonymous=True)
    pub_act = rospy.Publisher('actuation', i2c, queue_size=10)
    rate_act = rospy.Rate(WRITE_FREQ)
    # pub_read = rospy.Publisher('readings', reading, queue_size=10)
    # rate_read = rospy.Rate(SAMPLING_FREQ)
    rospy.Subscriber('input', target, setSpeed)
    rospy.Subscriber('raw_readings', raw_reading, getSpeed)

    while not rospy.is_shutdown():
        act.throttle = targetV
        act.direction = targetD
        pub_act.publish(act)
        rate_act.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
