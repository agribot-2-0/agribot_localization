#!/usr/bin/env python2

import os
from math import radians

import rospy
import rospkg
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geomag import GeoMag

class ImuDeclinationCorrectionNode:

    declination = None  # in radians

    def __init__(self):
        rospy.init_node('imu_declination_correction_node')

        agribot_localization_path = rospkg.RosPack().get_path('agribot_localization')
        wmm_filename = os.path.join(agribot_localization_path, 'params', 'WMM.COF')
        self.geomag = GeoMag(wmm_filename)

        self.filtered_imu_pub = rospy.Publisher('/agribot/imu/filtered',
                                                Imu,
                                                queue_size=1)

        self.navsatfix_sub = rospy.Subscriber("/agribot/gnss", NavSatFix,
                                              self.navsatfix_callback)
        self.prefiltered_imu_sub = rospy.Subscriber(
            "/agribot/imu/prefiltered", Imu, self.prefiltered_imu_callback)

    def navsatfix_callback(self, navsatfix_msg):
        mag = self.geomag.GeoMag(navsatfix_msg.latitude, navsatfix_msg.longitude, navsatfix_msg.altitude)
        self.declination = radians(mag.dec)
        rospy.loginfo_once('[ImuDeclinationCorrectionNode] Navsatfix received')

    def prefiltered_imu_callback(self, prefiltered_imu_msg):
        if self.declination is None:
            rospy.logwarn('[ImuDeclinationCorrectionNode] Navsatfix not received yet. Skipping Imu message...')
            return

        filtered_imu_msg = self.apply_declination_correction(prefiltered_imu_msg)
        self.filtered_imu_pub.publish(filtered_imu_msg)

    def apply_declination_correction(self, imu_msg):
        quat = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        r, p, y = euler_from_quaternion(quat)
        y += self.declination
        quat = quaternion_from_euler(r, p, y)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        return imu_msg

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    try:
        pass
        idcn = ImuDeclinationCorrectionNode()
        idcn.run()
    except rospy.ROSInterruptException:
        pass
