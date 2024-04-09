#!/usr/bin/env python  

import rospy
import tf
from nav_msgs.msg import Odometry

def handle_go1_pose(msg):
    br = tf.TransformBroadcaster()
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (quat),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_go1_pose,
                     )
    rospy.spin()