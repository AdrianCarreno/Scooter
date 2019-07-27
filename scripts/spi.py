#!/usr/bin/python

import rospy
from scooter.msg import raw_reading
import spidev
from AMT20 import read_position

ADDR_ENC1 = rospy.get_param('/spi/enc1/address')
ADDR_ENC2 = rospy.get_param('/spi/enc2/address')

enc1 = spidev.SpiDev()
enc1.open(0, ADDR_ENC1)
enc1.max_speed_hz = rospy.get_param('/spi/enc1/bitrate')

enc2 = spidev.SpiDev()
enc2.open(0, ADDR_ENC2)
enc2.max_speed_hz = rospy.get_param('/spi/enc2/bitrate')

data = raw_reading()

def talker():
    rospy.init_node('spi', anonymous=True)
    pub = rospy.Publisher('raw_readings', raw_reading, queue_size=10)
    rate = rospy.Rate(rospy.get_param('/readings/sampling_frequency'))
    rospy.on_shutdown(shutdown)

    while not rospy.is_shutdown():
        try:
            data.enc1 = read_position(enc1)
        except IOError:
            rospy.logerr("Can't read data from SPI device address %s", str(ADDR_ENC1))
        try:
            data.enc2 = read_position(enc2)
        except IOError:
            rospy.logerr("Can't read data from SPI device address %s", str(ADDR_ENC2))

        pub.publish(data)
        rate.sleep()

def shutdown():
    enc1.close()
    enc2.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
