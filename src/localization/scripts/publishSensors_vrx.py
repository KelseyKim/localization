#!/usr/bin/env python2

import rospy
from math import cos, sin, atan2, pi, radians
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float32

'''
Equations taken from:
https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
'''


class Heading:
    def __init__(self):
        rospy.init_node("publish_sensors")

        self.gps = NavSatFix()
        self.gps.header.frame_id = 'gps'

        self.imu = Imu()
        self.imu.header.frame_id = 'imu'

        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.updateGps)
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, self.updateImu)

        self.imuPub = rospy.Publisher('/sensors/imu', Imu, queue_size=10)
        self.gpsPub = rospy.Publisher('/sensors/gps', NavSatFix, queue_size=10)

    def updateImu(self, data):
        self.imu.header.stamp = rospy.Time.now()
        self.imu.orientation.x = data.orientation.x
        self.imu.orientation.y = data.orientation.y
        self.imu.orientation.z = data.orientation.z
        self.imu.orientation.w = data.orientation.w
        self.imu.orientation_covariance = [.00956973, 0, 0, 0, .00956973, 0, 0, 0, .00956973]
        self.imu.angular_velocity.x = data.angular_velocity.x
        self.imu.angular_velocity.y = data.angular_velocity.y
        self.imu.angular_velocity.z = data.angular_velocity.z
        self.imu.angular_velocity_covariance = [.00956973, 0, 0, 0, .00956973, 0, 0, 0, .00956973]
        self.imu.linear_acceleration.x = data.linear_acceleration.x
        self.imu.linear_acceleration.y = data.linear_acceleration.y
        self.imu.linear_acceleration.z = data.linear_acceleration.z
        self.imu.linear_acceleration_covariance = [.03295677, 0, 0, 0, .03295677, 0, 0, 0, .03295677]

        self.imuPub.publish(self.imu)

    def updateGps(self, data):
        self.gps.header.stamp = rospy.Time.now()
        self.gps.latitude = data.latitude
        self.gps.longitude = data.longitude
        self.gps.altitude = data.altitude
        # self.gps.position_covariance = data.position_covariance
        self.gps.position_covariance = [.7225, 0, 0, 0, .7225, 0, 0, 0, 4]
        self.gps.position_covariance_type = data.position_covariance_type

        self.gpsPub.publish(self.gps)

if __name__ == '__main__':
    headingCalc = Heading()
    rospy.spin()
    rospy.rate(5)
