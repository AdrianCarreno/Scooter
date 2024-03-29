#!/usr/bin/python

import rospy
from scooter.msg import odometry
from AMT20 import AMT20


class SPI:
    def __init__(self):
        self.ADDR_ENC1 = rospy.get_param("spi/enc1/address")
        self.ADDR_ENC2 = rospy.get_param("spi/enc2/address")
        self.LEFT_ENC = rospy.get_param("spi/left_enc")
        self._init_errors = []
        rospy.init_node("spi", anonymous=True)
        rospy.on_shutdown(self.shutdown)
        max_speed = rospy.get_param("max_speed")
        radius = rospy.get_param("radius")
        sampling_frequency = rospy.get_param("readings/sampling_frequency")
        theta_max = max_speed / (3.6 * radius * sampling_frequency)

        # Check if encoders are connected and responding
        try:
            self.enc1 = AMT20(
                self.ADDR_ENC1, rospy.get_param("spi/enc1/direction"), theta_max=theta_max)
            rospy.loginfo("Connected enc1 on address %s", str(self.ADDR_ENC1))
        except IOError:
            self._init_errors.append(self.ADDR_ENC1)

        try:
            self.enc2 = AMT20(
                self.ADDR_ENC2, rospy.get_param("spi/enc2/direction"), theta_max=theta_max)
            rospy.loginfo("Connected enc2 on address %s", str(self.ADDR_ENC2))
        except IOError:
            self._init_errors.append(self.ADDR_ENC2)

        if len(self._init_errors) != 0:
            for i in self._init_errors:
                rospy.logerr("Can't open SPI device address %s", str(i))
            self.enc1 = None    # This is so shutdown() doesn"t crash
            self.enc2 = None
            rospy.signal_shutdown(
                "Failed to initiate communications with some SPI devices")

    def run(self):
        rate = rospy.Rate(rospy.get_param("readings/sampling_frequency"))
        pub = rospy.Publisher("odometry", odometry, queue_size=10)
        rospy.loginfo("Publisher initialized correctly")
        data = odometry()

        while not rospy.is_shutdown():
            try:
                angle1 = self.enc1.angle()
                speed1 = self.enc1.angular_speed()
            except IOError:
                rospy.logerr(
                    "Can't read data from SPI device address %s", str(self.ADDR_ENC1))
                continue
            try:
                angle2 = self.enc2.angle()
                speed2 = self.enc2.angular_speed()
            except IOError:
                rospy.logerr(
                    "Can't read data from SPI device address %s", str(self.ADDR_ENC2))
                continue

            if self.LEFT_ENC == "enc1":
                data.theta_left = angle1
                data.theta_right = angle2
                data.w_left = speed1
                data.w_right = speed2
            else:
                data.theta_right = angle1
                data.theta_left = angle2
                data.w_right = speed1
                data.w_left = speed2

            pub.publish(data)
            rate.sleep()

    def shutdown(self):
        del self.enc1
        del self.enc2


if __name__ == "__main__":
    try:
        spi_node = SPI()
        spi_node.run()
    except rospy.ROSInterruptException:
        pass
