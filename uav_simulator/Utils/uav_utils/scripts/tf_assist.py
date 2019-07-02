#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy


imu_pub = None
odom_pub = None
br = None


class OdometryConverter(object):

    def __init__(self, frame_id_in_, frame_id_out_, broadcast_tf_, body_frame_id_, intermediate_frame_id_, world_frame_id_):
        self.frame_id_in = frame_id_in_
        self.frame_id_out = frame_id_out_
        self.broadcast_tf = broadcast_tf_
        self.body_frame_id = body_frame_id_
        self.intermediate_frame_id = intermediate_frame_id_
        self.world_frame_id = world_frame_id_
        self.in_odom_sub = None
        self.out_odom_pub = None
        self.out_path_pub = None
        self.path_pub_timer = None
        self.tf_pub_flag = True
        if self.broadcast_tf:
            rospy.loginfo('ROSTopic: [%s]->[%s] TF: [%s]-[%s]-[%s]',
                          self.frame_id_in, self.frame_id_out, self.body_frame_id, self.intermediate_frame_id, self.world_frame_id)
        else:
            rospy.loginfo('ROSTopic: [%s]->[%s] No TF',
                          self.frame_id_in, self.frame_id_out)

        self.path = []

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = tfs.euler_from_quaternion(q, 'rzyx')
        wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = tfs.quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert(in_odom_msg.header.frame_id == self.frame_id_in)
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            if not self.frame_id_in == self.frame_id_out:
                br.sendTransform((0.0, 0.0, 0.0),
                                 tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                 odom_msg.header.stamp,
                                 self.frame_id_in,
                                 self.frame_id_out)

            if not self.world_frame_id == self.frame_id_out:
                br.sendTransform((0.0, 0.0, 0.0),
                                 tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                                 odom_msg.header.stamp,
                                 self.world_frame_id,
                                 self.frame_id_out)

            br.sendTransform((p[0], p[1], p[2]),
                             wqb,
                             odom_msg.header.stamp,
                             self.body_frame_id,
                             self.world_frame_id)

            br.sendTransform(((p[0], p[1], p[2])),
                             wqc,
                             odom_msg.header.stamp,
                             self.intermediate_frame_id,
                             self.world_frame_id)
        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def path_pub_callback(self, event):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self, event):
        self.tf_pub_flag = True


if __name__ == "__main__":
    rospy.init_node('tf_assist')

    converters = []
    index = 0
    while True:
        prefix = "~converter%d/" % index
        try:
            frame_id_in = rospy.get_param('%sframe_id_in' % prefix)
            frame_id_out = rospy.get_param('%sframe_id_out' % prefix)
            broadcast_tf = rospy.get_param('%sbroadcast_tf' % prefix, False)
            body_frame_id = rospy.get_param('%sbody_frame_id' % prefix, 'body')
            intermediate_frame_id = rospy.get_param(
                '%sintermediate_frame_id' % prefix, 'intermediate')
            world_frame_id = rospy.get_param(
                '%sworld_frame_id' % prefix, 'world')

            converter = OdometryConverter(
                frame_id_in, frame_id_out, broadcast_tf, body_frame_id, intermediate_frame_id, world_frame_id)
            converter.in_odom_sub = rospy.Subscriber(
                '%sin_odom' % prefix, Odometry, converter.in_odom_callback, tcp_nodelay=True)
            converter.out_odom_pub = rospy.Publisher(
                '%sout_odom' % prefix, Odometry, queue_size=10, tcp_nodelay=True)
            converter.out_path_pub = rospy.Publisher(
                '%sout_path' % prefix, Path, queue_size=10)

            converter.tf_pub_timer = rospy.Timer(
                rospy.Duration(0.1), converter.tf_pub_callback)

            converter.path_pub_timer = rospy.Timer(
                rospy.Duration(0.5), converter.path_pub_callback)

            index += 1
        except KeyError, e:
            if index == 0:
                raise(KeyError(e))
            else:
                if index == 1:
                    rospy.loginfo(
                        'prefix:"%s" not found. Generate %d converter.' % (prefix, index))
                else:
                    rospy.loginfo(
                        'prefix:"%s" not found. Generate %d converters' % (prefix, index))
                break

    br = tf.TransformBroadcaster()

    rospy.spin()
