#!/usr/bin/python

import rospy
from scooter.msg import spi
import spidev
import time

nop_a5 = 0x00
rd_pos = 0x10
set_zero = 0x70

enc1 = spidev.SpiDev()
enc1.open(0, 0)
enc1.max_speed_hz = 976000

enc2 = spidev.SpiDev()
enc2.open(0, 1)
enc2.max_speed_hz = 976000

data = spi()

# This command causes a read of the current position.
def read_position(enc):
    enc.writebytes(rd_pos)
    response = enc.readbytes(1)

    while (response == 0xA5)
        enc.writebytes(nop_a5)
        response = enc.readbytes(1)

    if (response == rd_pos)
        enc.writebytes(nop_a5)
        response = enc.readbytes(1)
        pos = response << 8
        enc.writebytes(nop_a5)
        response = enc.readbytes(1)
        pos = pos | response
    return pos

# This command sets the current position to zero and saves this setting in the EEPROM.
def set_zero(enc):
    enc.writebytes(set_zero_point)
    response = enc.readbytes(1)

    while (response != 0x80)
        enc.writebytes(nop_a5)
        response = enc.readbytes(1)

    # The encoder must be power cycled. If the encoder is not power cycled, the position
    # values will not be calculated off the latest zero position. When the encoder is
    # powered on next the new offset will be used for the position calculation.
    return


def talker():
    pub = rospy.Publisher('encoders', spi, queue_size=10)
    rospy.init_node('encoder_reader', anonymous=True)
    rate = rospy.Rate(1000) # 1000hz

    while not rospy.is_shutdown():
        data.enc1 = read_position(enc1)
        data.enc2 = read_position(enc2)
        pub.publish(data)
        rospy.loginfo('Encoder 1: %s, Encoder 2: %s', str(data.enc1), str(data.enc2))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass