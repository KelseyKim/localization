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
        rospy.init_node("sensor_corrections")

        self.heading = 0

        self.portLat = 0
        self.portLon = 0

        self.starboardLat = 0
        self.starboardLon = 0

        self.portGps = NavSatFix()
        self.portGps.header.frame_id = 'port_gps'

        self.starboardGps = NavSatFix()
        self.starboardGps.header.frame_id = 'starboard_gps'

        self.gpsImu = Imu()
        self.gpsImu.header.frame_id = 'gps_imu'

        self.adImu = Imu()
        self.adImu.header.frame_id = 'ad_imu'

        rospy.Subscriber('/sensors/portFix', NavSatFix, self.updatePort)
        rospy.Subscriber('/sensors/starboardFix', NavSatFix, self.updateStarboard)
        rospy.Subscriber('/sensors/imu/data', Imu, self.updateImu)

        self.headingPub = rospy.Publisher('/sensors/gpsHeading', Float32, queue_size=10)
        self.adImuPub = rospy.Publisher('/sensors/adImu', Imu, queue_size=10)
        self.gpsImuPub = rospy.Publisher('/sensors/gpsImu', Imu, queue_size=10)
        self.portGpsPub = rospy.Publisher('/sensors/portGps', NavSatFix, queue_size=10)
        self.starboardGpsPub = rospy.Publisher('/sensors/starboardGps', NavSatFix, queue_size=10)

    def updateImu(self, data):
        self.adImu.orientation.x = data.orientation.x
        self.adImu.orientation.y = data.orientation.y
        self.adImu.orientation.z = data.orientation.z
        self.adImu.orientation.w = data.orientation.w
        self.adImu.orientation_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        self.adImu.angular_velocity.x = data.angular_velocity.x
        self.adImu.angular_velocity.y = data.angular_velocity.y
        self.adImu.angular_velocity.z = data.angular_velocity.z
        self.adImu.angular_velocity_covariance = [.00956973, 0, 0, 0, .00956973, 0, 0, 0, .00956973]
        self.adImu.linear_acceleration.x = data.linear_acceleration.x
        self.adImu.linear_acceleration.y = data.linear_acceleration.y
        self.adImu.linear_acceleration.z = data.linear_acceleration.z
        self.adImu.linear_acceleration_covariance = [.03295677, 0, 0, 0, .03295677, 0, 0, 0, .03295677]
        self.adImu.header.stamp = rospy.Time.now()

        self.adImuPub.publish(self.adImu)

    def updatePort(self, data):
        self.portGps.header.stamp = rospy.Time.now()
        self.portGps.latitude = data.latitude
        self.portGps.longitude = data.longitude
        self.portGps.altitude = data.altitude
        self.portGps.position_covariance = data.position_covariance
        self.portGps.position_covariance_type = data.position_covariance_type
        self.portGpsPub.publish(self.portGps)

        self.portLat = data.latitude
        self.portLon = data.longitude
        self.calcBearing()

    def updateStarboard(self, data):
        self.starboardGps.header.stamp = rospy.Time.now()
        self.starboardGps.latitude = data.latitude
        self.starboardGps.longitude = data.longitude
        self.starboardGps.altitude = data.altitude
        self.starboardGps.position_covariance = data.position_covariance
        self.starboardGps.position_covariance_type = data.position_covariance_type
        self.starboardGpsPub.publish(self.starboardGps)

        self.starboardLat = data.latitude
        self.starboardLon = data.longitude
        self.calcBearing()

    def calcBearing(self):
        dlat = radians(self.portLat - self.starboardLat)
        dlon = radians(self.portLon - self.starboardLon)
        psi = atan2(dlon, dlat) + 1.5708    # heading in NED

        psi_enu = -1*psi + 1.5708

        if psi_enu < -pi:
            psi_enu = psi_enu + 2*pi
        if psi_enu > pi:
            psi_enu = psi_enu - 2*pi

        self.heading = psi_enu

        quat = quaternion_from_euler(0, 0, psi_enu)

        self.gpsImu.orientation.x = quat[0]
        self.gpsImu.orientation.y = quat[1]
        self.gpsImu.orientation.z = quat[2]
        self.gpsImu.orientation.w = quat[3]
        self.gpsImu.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        self.gpsImu.angular_velocity_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        self.gpsImu.linear_acceleration_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
        self.gpsImu.header.stamp = rospy.Time.now()
        self.gpsImuPub.publish(self.gpsImu)

        self.headingPub.publish(self.heading)

if __name__ == '__main__':
    headingCalc = Heading()
    rospy.spin()
    rospy.rate(5)
