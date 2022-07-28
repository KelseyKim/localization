#!/usr/bin/env python2

import rospy
from math import cos, sin, atan2, pi, radians
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32

'''
Equations taken from:
https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
'''


class Heading:
    def __init__(self):
        rospy.init_node("diff_gps_heading")

        self.heading = 0

        self.portLat = 0
        self.portLon = 0

        self.starboardLat = 0
        self.starboardLon = 0

        rospy.Subscriber('/sensors/portFix', NavSatFix, self.updatePort)
        rospy.Subscriber('/sensors/starboardFix', NavSatFix, self.updateStarboard)

        self.headingPub = rospy.Publisher('/sensors/gpsHeading', Float32, queue_size=10)

    def updatePort(self, data):
        self.portLat = data.latitude
        self.portLon = data.longitude
        self.calcBearing()

    def updateStarboard(self, data):
        self.starboardLat = data.latitude
        self.starboardLon = data.longitude
        self.calcBearing()

    def calcBearing(self):
        dlat = radians(self.portLat - self.starboardLat)
        dlon = radians(self.portLon - self.starboardLon)
        psi = atan2(dlon, dlat) + 1.5708

        if psi < -pi:
            psi = psi + 2*pi
        if psi > pi:
            psi = psi - 2*pi

        self.heading = psi

        self.headingPub.publish(self.heading)

if __name__ == '__main__':
    headingCalc = Heading()
    rospy.spin()
    rospy.rate(5)
