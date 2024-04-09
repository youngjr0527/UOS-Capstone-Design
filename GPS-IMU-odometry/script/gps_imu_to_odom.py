#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from unitree_legged_msgs.msg import HighState
from pyproj import Proj, transform
from math import sin, cos, pi, pow, sqrt, atan2

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)

        # Publisher 및 Subscriber 생성
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_sub = rospy.Subscriber('/high_state', HighState, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/ublox1/fix', NavSatFix, self.gps_callback)

        # 초기화
        self.proj_UTM52 = Proj(init='epsg:32652')
        self.proj_WGS84 = Proj(init='epsg:4326')

        self.is_gps = False
        self.is_imu = False
        self.is_cali = False

        self.gpsX = 0.0
        self.gpsY = 0.0
        self.past_gpsX = 0.0
        self.past_gpsY = 0.0
        self.imu_yaw = 0.0
        self.odom = Odometry()

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.vel_yaw = 0.0

        self.dx = 0.0
        self.dy = 0.0
        self.yaw_offset = 0.0

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.is_gps == True and self.is_imu == True:
                self.imu_calibration()
                self.odom_combine()
                self.odom_pub.publish(self.odom)

                r.sleep()


    def imu_callback(self, data):
        self.is_imu = True
        self.imu_yaw = data.imu.rpy[2] + self.yaw_offset #+ 1.8515

        self.vel_x = data.velocity[0]
        self.vel_y = data.velocity[1]
        self.vel_z = data.velocity[2]
        self.vel_yaw = data.yawSpeed

    def gps_callback(self, data):
        if self.is_gps == True:
            self.gpsX, self.gpsY = transform(self.proj_WGS84, self.proj_UTM52, data.longitude, data.latitude)
            self.gpsX -= 328540.0
            self.gpsY -= 4161354.0

            self.dx = self.gpsX - self.past_gpsX
            self.dy = self.gpsY - self.past_gpsY
        else:
            self.past_gpsX, self.past_gpsY = transform(self.proj_WGS84, self.proj_UTM52, data.longitude, data.latitude)
            self.past_gpsX -= 328540.0
            self.past_gpsY -= 4161354.0            

        self.is_gps = True

    def imu_calibration(self):
        if self.is_cali == False:
            print("distance = ", sqrt(pow(self.dx, 2) + pow(self.dy, 2)))
            if sqrt(pow(self.dx, 2) + pow(self.dy, 2)) > 1.0:
                self.yaw_offset = atan2(self.dy, self.dx) - self.imu_yaw
                print("yaw_offset is = ", self.yaw_offset)
                self.is_cali = True

        if self.imu_yaw > pi:
            self.imu_yaw - 2 * pi
        elif self.imu_yaw < -pi:
            self.imu_yaw + 2 * pi

        # print("calibated imu yaw = ", self.imu_yaw)

    def odom_combine(self):
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.position.x = self.gpsX
        self.odom.pose.pose.position.y = self.gpsY
        self.odom.pose.pose.position.z = 0

        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = sin(self.imu_yaw*0.5)
        self.odom.pose.pose.orientation.w = cos(self.imu_yaw*0.5)

        self.odom.twist.twist.linear.x = self.vel_x
        self.odom.twist.twist.linear.y = self.vel_y
        self.odom.twist.twist.linear.z = self.vel_z
        self.odom.twist.twist.angular.z = self.vel_yaw

if __name__ == '__main__':
    try:
        odometry_publisher = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
