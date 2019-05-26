#!/usr/bin/python

import rospy
from scooter.msg import spi
import spidev
import time
from AMT20 import read_position

enc1 = spidev.SpiDev()
enc1.open(0, 0)
enc1.max_speed_hz = 976000

enc2 = spidev.SpiDev()
enc2.open(0, 1)
enc2.max_speed_hz = 976000

data = spi()


def talker():
    pub = rospy.Publisher('encoders', spi, queue_size=10)
    rospy.init_node('encoder_reader', anonymous=True)
    rate = rospy.Rate(1000)  # 1000hz

    while not rospy.is_shutdown():
        data.enc1 = read_position(enc1)
        data.enc2 = read_position(enc2)
        pub.publish(data)
        rospy.loginfo('Enc1: %s, Enc2: %s', str(data.enc1), str(data.enc2))
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
