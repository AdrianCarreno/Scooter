#!/usr/bin/python

import rospy
from scooter.msg import reading
from AMT20 import AMT20

class SPI:
    def __init__(self):
        self.ADDR_ENC1 = rospy.get_param('spi/enc1/address')
        self.ADDR_ENC2 = rospy.get_param('spi/enc2/address')
        self.LEFT_ENC = rospy.get_param('spi/left_enc')

        rospy.init_node('spi', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Check if encoders are conected and responding
        try:
            self.enc1 = AMT20(self.ADDR_ENC1, rospy.get_param('spi/enc1/direction'))
            rospy.loginfo('Connected enc1 on address %s', str(self.ADDR_ENC1))
        except IOError:
            self.enc1 = None    # This is so shutdown() doesn't crash
            self.enc2 = None
            rospy.logerr("Can't open SPI device address %s", str(self.ADDR_ENC1))
            rospy.signal_shutdown("Can't open SPI device address " + str(self.ADDR_ENC1))
        try:
            self.enc2 = AMT20(self.ADDR_ENC2, rospy.get_param('spi/enc2/direction'))
            rospy.loginfo('Connected enc2 on address %s', str(self.ADDR_ENC2))
        except IOError:
            self.enc1 = None    # This is so shutdown() doesn't crash
            self.enc2 = None
            rospy.logerr("Can't open SPI device address %s", str(self.ADDR_ENC2))
            rospy.signal_shutdown("Can't open SPI device address " + str(self.ADDR_ENC2))

    def run(self):
        rate = rospy.Rate(rospy.get_param('readings/sampling_frequency'))
        pub = rospy.Publisher('readings', reading, queue_size=10)
        rospy.loginfo('Publisher initialized correctly')
        data = reading()

        while not rospy.is_shutdown():
            if self.LEFT_ENC == 'enc1':
                try:
                    data.w_left = self.enc1.angular_speed()
                except IOError:
                    rospy.logerr("Can't read data from SPI device address %s", str(self.ADDR_ENC1))
                try:
                    data.w_right = self.enc2.angular_speed()
                except IOError:
                    rospy.logerr("Can't read data from SPI device address %s", str(self.ADDR_ENC2))
            else:
                try:
                    data.w_right = self.enc1.angular_speed()
                except IOError:
                    rospy.logerr("Can't read data from SPI device address %s", str(self.ADDR_ENC1))
                try:
                    data.w_left = self.enc2.angular_speed()
                except IOError:
                    rospy.logerr("Can't read data from SPI device address %s", str(self.ADDR_ENC2))

            pub.publish(data)
            rate.sleep()

    def shutdown(self):
        del self.enc1
        del self.enc2

if __name__ == '__main__':
    try:
        spi_node = SPI()
        spi_node.run()
    except rospy.ROSInterruptException:
        pass
