#!/usr/bin/python

import rospy
import numpy
from scooter.msg import i2c, spi, target
from kinematic import countsToW

THR_ADDR = 0x10
DIR_ADDR = 0x11

dataOut = i2c()
dataOut.throttle_addr = THR_ADDR
dataOut.direction_addr = DIR_ADDR
dataOut.throttle = 128
dataOut.direction = 128

radius = 0.15
last1 = 0
last2 = 0

P = 1   # Proportional gain

targetV = 0 # Target speed
targetD = 0 # Target direction
curV = 0    # Current speed

def getCurSpeed(dataIn):
    v1 = radius*countsToW(last1, dataIn.enc1, 1/1000)
    v2 = radius*countsToW(last2, dataIn.enc2, 1/1000)
    curV = (v1 + v2) / 2
    last1 = dataIn.enc1
    last2 = dataIn.enc2

def getTargSpeed(target):
    targetV = target.vel
    targetD = target.dir

def controller():
    rospy.init_node('controller', anonymous=True)
    pub = rospy.Publisher('actuation', i2c, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    rospy.Subscriber('console', target, getTargSpeed)
    rospy.Subscriber('encoders', spi, getCurSpeed)

    while not rospy.is_shutdown():
        err = targetV - curV
        dataOut.throttle = int(numpy.clip(P * err, 0, 255))
        dataOut.direction = targetD
        pub.publish(dataOut)
#         rospy.loginfo('Throttle addr: %s, throttle val: %s', str(dataOut.throttle_addr), str(dataOut.throttle))
#         rospy.loginfo('Direction addr: %s, direction val: %s', str(dataOut.direction_addr), str(dataOut.direction))
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
