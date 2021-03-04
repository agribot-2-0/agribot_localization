#!/usr/bin/env python2

from math import atan2
from statistics import mean

import rospy
from sensor_msgs.msg import MagneticField, Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from agribot_common.agribot_parameters import get_parameter


class ImuMagnetometerFilter(object):
    def __init__(self):

        rospy.init_node('imu_magnetometer_filter')

        self.queue_size = int(get_parameter('~queue_size'))
        self.yaw_list = []

        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)

        self.magnetometer_sub = rospy.Subscriber("imu/mag", MagneticField,
                                                 self.magnetometer_callback)

    def magnetometer_callback(self, mag_msg):

        # Pop yaw from queue if necessary
        if len(self.yaw_list) >= self.queue_size:
            self.yaw_list.pop(0)

        # Add new yaw to queue
        yaw = ImuMagnetometerFilter.calculate_yaw_from_magnetic_field(mag_msg)
        self.yaw_list.append(yaw)

        imu_msg = Imu()
        imu_msg.header = mag_msg.header

        # Calculate orientation based on yaw average
        yaw_avg = mean(self.yaw_list)
        imu_msg.orientation = ImuMagnetometerFilter.quaternion_msg_from_rpy(
            0, 0, yaw_avg)

        self.imu_pub.publish(imu_msg)

    @staticmethod
    def calculate_yaw_from_magnetic_field(mag_msg):
        mag_x = mag_msg.magnetic_field.x
        mag_y = mag_msg.magnetic_field.y

        # For NEU frame formula is atan2(mag_y, mag_x)
        # So recalculating to ENU frame, we obtain atan2(mag_x, mag_y)
        return atan2(mag_x, mag_y)

    @staticmethod
    def quaternion_msg_from_rpy(roll, pitch, yaw):
        quat = quaternion_from_euler(roll, pitch, yaw)
        quat_msg = Quaternion()
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]

        return quat_msg

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    try:
        IMF = ImuMagnetometerFilter()
        IMF.run()
    except rospy.ROSInterruptException:
        pass
