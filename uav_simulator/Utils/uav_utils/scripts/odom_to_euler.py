#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy

pub = None
pub1 = None

def callback(odom_msg):
    q = np.array([odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w])

    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = odom_msg.header
    euler_msg.vector.z = e[0]*180.0/3.14159
    euler_msg.vector.y = e[1]*180.0/3.14159
    euler_msg.vector.x = e[2]*180.0/3.14159

    pub.publish(euler_msg)

def imu_callback(imu_msg):
    q = np.array([imu_msg.orientation.x,
                  imu_msg.orientation.y,
                  imu_msg.orientation.z,
                  imu_msg.orientation.w])

    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = imu_msg.header
    euler_msg.vector.z = e[0]*180.0/3.14159
    euler_msg.vector.y = e[1]*180.0/3.14159
    euler_msg.vector.x = e[2]*180.0/3.14159

    pub1.publish(euler_msg)

def joy_callback(joy_msg):
    out_msg = Vector3Stamped()
    out_msg.header = joy_msg.header
    out_msg.vector.z = -joy_msg.axes[3]
    out_msg.vector.y = joy_msg.axes[1]
    out_msg.vector.x = joy_msg.axes[0]

    pub2.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("odom_to_euler")

    pub = rospy.Publisher("~euler", Vector3Stamped, queue_size=10)
    sub = rospy.Subscriber("~odom", Odometry, callback)

    pub1 = rospy.Publisher("~imueuler", Vector3Stamped, queue_size=10)
    sub1 = rospy.Subscriber("~imu", Imu, imu_callback)

    pub2 = rospy.Publisher("~ctrlout", Vector3Stamped, queue_size=10)
    sub2 = rospy.Subscriber("~ctrlin", Joy, joy_callback)

    rospy.spin()
