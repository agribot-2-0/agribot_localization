#!/usr/bin/env python2

from math import atan2, degrees

import rospy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64


class MagnetometerHeading(object):
    def __init__(self):
        self.declination = None

        rospy.init_node('magnetometer_heading_node')

        self.heading_pub = rospy.Publisher('/agribot/imu/raw/heading',
                                           Float64,
                                           queue_size=1)

        self.magnetometer_sub = rospy.Subscriber("/agribot/imu/raw/mag",
                                                 MagneticField,
                                                 self.magnetometer_callback)

        self.magnetometer_sub = rospy.Subscriber("/agribot/imu/declination",
                                                 Float64,
                                                 self.declination_callback)

    def declination_callback(self, dec_msg):
        self.declination = dec_msg.data

    def magnetometer_callback(self, mag_msg):
        heading = self.calculate_heading_from_magnetig_field(mag_msg)

        if self.declination is not None:
            msg = Float64()
            msg.data = degrees(heading) + self.declination
            self.heading_pub.publish(msg)
        else:
            rospy.logwarn_throttle(
                5,
                """[MagnetometerHeading] Declination not received yet.
                Magnetic heading is not published...""")

    def calculate_heading_from_magnetig_field(self, mag_msg):
        mag_x = mag_msg.magnetic_field.x
        mag_y = mag_msg.magnetic_field.y

        return atan2(-mag_y, mag_x)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    try:
        MH = MagnetometerHeading()
        MH.run()
    except rospy.ROSInterruptException:
        pass
