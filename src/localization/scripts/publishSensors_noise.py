#!/usr/bin/env python2

import rospy
from math import cos, sin, atan2, pi, radians
from random import gauss
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

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.steadyStateDuration = 10
        self.blipDuration = 10

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
        self.imu.orientation_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        self.imu.angular_velocity.x = data.angular_velocity.x
        self.imu.angular_velocity.y = data.angular_velocity.y
        self.imu.angular_velocity.z = data.angular_velocity.z
        self.imu.angular_velocity_covariance = [.00956973, 0, 0, 0, .00956973, 0, 0, 0, .00956973]
        self.imu.linear_acceleration.x = data.linear_acceleration.x
        self.imu.linear_acceleration.y = data.linear_acceleration.y
        self.imu.linear_acceleration.z = data.linear_acceleration.z
        self.imu.linear_acceleration_covariance = [.03295677, 0, 0, 0, .03295677, 0, 0, 0, .03295677]


    def updateGps(self, data):
        self.gps.header.stamp = rospy.Time.now()
        self.gps.latitude = data.latitude
        self.gps.longitude = data.longitude
        self.gps.altitude = data.altitude
        self.gps.position_covariance = data.position_covariance
        self.gps.position_covariance_type = data.position_covariance_type


if __name__ == "__main__":
    hc = Heading()

    print(rospy.get_time())
    # Reach steady state first
    ssEndTime = rospy.get_time() + hc.steadyStateDuration
    while (ssEndTime > rospy.get_time()) and not rospy.is_shutdown():
        hc.gpsPub.publish(hc.gps)
        hc.imuPub.publish(hc.imu)
        rospy.Rate(5)
        # print(rospy.get_time())

    # Introduce super noisy sensor data
    blipEndTime = rospy.get_time() + hc.blipDuration
    while (blipEndTime > rospy.get_time()) and not rospy.is_shutdown():

        '''Angular Velocity Blip'''
        # hc.imu.angular_velocity.z = hc.imu.angular_velocity.z + gauss(0, 3.14159/2)
        # hc.gpsPub.publish(hc.gps)
        # hc.imuPub.publish(hc.imu)

        '''Heading Blip'''
        # quat = [hc.imu.orientation.x, hc.imu.orientation.y, hc.imu.orientation.z, hc.imu.orientation.w]
        # [hc.roll, hc.pitch, hc.yaw] = euler_from_quaternion(quat)
        #
        # hc.yaw = hc.yaw + gauss(0, 3.14159)
        #
        # if hc.yaw < -pi:
        #     hc.yaw = hc.yaw + 2*pi
        # if hc.yaw > pi:
        #     hc.yaw = hc.yaw - 2*pi
        #
        #
        # quat = quaternion_from_euler(hc.roll, hc.pitch, hc.yaw)
        # hc.imu.orientation.x = quat[0]
        # hc.imu.orientation.y = quat[1]
        # hc.imu.orientation.z = quat[2]
        # hc.imu.orientation.w = quat[3]
        #
        # hc.gpsPub.publish(hc.gps)
        # hc.imuPub.publish(hc.imu)

        '''Linear Acceleration Blip'''
        # hc.imu.linear_acceleration.x = hc.imu.linear_acceleration.x + gauss(0,2)
        # hc.imu.linear_acceleration.y = hc.imu.linear_acceleration.x + gauss(0,2)
        # hc.gpsPub.publish(hc.gps)
        # hc.imuPub.publish(hc.imu)

        '''GPS Blip'''
        hc.gps.latitude = hc.gps.latitude + gauss(0,0.0002)
        hc.gps.longitude = hc.gps.longitude + gauss(0,0.0002)
        hc.gpsPub.publish(hc.gps)
        hc.imuPub.publish(hc.imu)

        rospy.Rate(5)

    # Return to normal sensors
    while not rospy.is_shutdown():
        hc.gpsPub.publish(hc.gps)
        hc.imuPub.publish(hc.imu)
        rospy.Rate(5)
        # print(rospy.get_time())
