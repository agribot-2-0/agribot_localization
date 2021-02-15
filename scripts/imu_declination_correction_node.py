#!/usr/bin/env python2

from math import radians

import rospy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geomag import GeoMag


class ImuDeclinationCorrectionNode(object):

    declination = None  # in radians

    def __init__(self):
        rospy.init_node('imu_declination_correction_node')

        if rospy.has_param(
                '/imu_declination_correction_node/wmm_config_filepath'):
            wmm_filename = rospy.get_param(
                "/imu_declination_correction_node/wmm_config_filepath")
            self.geomag = GeoMag(wmm_filename)
        else:
            rospy.logerr(
                '[Declination Correction Node] Missing wmm_config_filepath parameter. Exiting...'
            )
            rospy.signal_shutdown('wmm_config_filepath missing...')

        self.filtered_imu_pub = rospy.Publisher('/agribot/imu/filtered',
                                                Imu,
                                                queue_size=1)

        self.declination_pub = rospy.Publisher('/agribot/imu/declination',
                                               Float64,
                                               queue_size=1)

        self.navsatfix_sub = rospy.Subscriber("/agribot/gnss", NavSatFix,
                                              self.navsatfix_callback)
        self.prefiltered_imu_sub = rospy.Subscriber(
            "/agribot/imu/prefiltered", Imu, self.prefiltered_imu_callback)

    def navsatfix_callback(self, navsatfix_msg):
        if navsatfix_msg.status.status is not NavSatStatus.STATUS_NO_FIX:
            mag = self.geomag.GeoMag(navsatfix_msg.latitude,
                                     navsatfix_msg.longitude,
                                     navsatfix_msg.altitude)
            self.declination = radians(mag.dec)

            declination_msg = Float64()
            declination_msg.data = mag.dec
            self.declination_pub.publish(msg)

            rospy.loginfo_once(
                '[ImuDeclinationCorrectionNode] First good NavSatFix received')
        else:
            rospy.logwarn_throttle(
                5,
                '[ImuDeclinationCorrectionNode] NavSatFix ignored - STATUS_NO_FIX'
            )

    def prefiltered_imu_callback(self, prefiltered_imu_msg):
        if self.declination is None:
            rospy.logwarn_throttle(
                5,
                """[ImuDeclinationCorrectionNode] Navsatfix not received yet.
                Imu messages are not republished...""")
            return

        filtered_imu_msg = self.apply_declination_correction(
            prefiltered_imu_msg)
        self.filtered_imu_pub.publish(filtered_imu_msg)

    def apply_declination_correction(self, imu_msg):
        quat = [
            imu_msg.orientation.x, imu_msg.orientation.y,
            imu_msg.orientation.z, imu_msg.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        yaw += self.declination
        quat = quaternion_from_euler(roll, pitch, yaw)
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
        IDCN = ImuDeclinationCorrectionNode()
        IDCN.run()
    except rospy.ROSInterruptException:
        pass
