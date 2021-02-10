#!/usr/bin/env python2

from math import degrees

import rospy
from sensor_msgs.msg import NavSatFix
from gps_common.msg import GPSFix
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class NavsatfixToGPSFixRewriter(object):
    def __init__(self, in_topic, out_topic, odom_topic):
        self.out_pub = rospy.Publisher(out_topic, GPSFix, queue_size=1)
        self.odom = rospy.Subscriber(odom_topic, Odometry, self.odom_cb)
        self.in_sub = rospy.Subscriber(in_topic, NavSatFix, self.navsatfix_cb)
        self.global_heading = None

    def navsatfix_cb(self, navsatfix_msg):
        if self.global_heading is not None:
            gpsfix_msg = GPSFix()
            gpsfix_msg.header = navsatfix_msg.header
            gpsfix_msg.status.header = navsatfix_msg.header
            gpsfix_msg.status.status = navsatfix_msg.status.status
            gpsfix_msg.latitude = navsatfix_msg.latitude
            gpsfix_msg.longitude = navsatfix_msg.longitude
            gpsfix_msg.altitude = navsatfix_msg.altitude
            gpsfix_msg.track = self.global_heading
            gpsfix_msg.position_covariance = navsatfix_msg.position_covariance
            gpsfix_msg.position_covariance_type = navsatfix_msg.position_covariance_type

            self.out_pub.publish(gpsfix_msg)

    def odom_cb(self, odom_msg):
        quat = odom_msg.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        _roll, _pitch, yaw = euler_from_quaternion(quat_list)
        self.global_heading = NavsatfixToGPSFixRewriter.rep105_degrees_to_heading(
            yaw)

    @staticmethod
    def rep105_degrees_to_heading(yaw):
        yaw_deg = degrees(yaw)
        return -(yaw_deg - 90)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('navsatfix_to_gpsfix_rewriter')
    RAW_REWRITER = NavsatfixToGPSFixRewriter('/agribot/gnss',
                                             '/agribot/gpsfix',
                                             '/agribot/odom/global')
    FILTERED_REWRITER = NavsatfixToGPSFixRewriter('/agribot/gnss/filtered',
                                                  '/agribot/gpsfix/filtered',
                                                  '/agribot/odom/global')
    NavsatfixToGPSFixRewriter.run()
