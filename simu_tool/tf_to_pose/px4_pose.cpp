//
// Created by ray on 20-11-9.
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped tf_to_pose(tf::StampedTransform tf,std::string frame_id){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x=tf.getOrigin().x();
    pose.pose.position.y=tf.getOrigin().y();
    pose.pose.position.z=tf.getOrigin().z();

    pose.pose.orientation.x=tf.getRotation().x();
    pose.pose.orientation.y=tf.getRotation().y();
    pose.pose.orientation.z=tf.getRotation().z();
    pose.pose.orientation.w=tf.getRotation().w();
    pose.header.stamp=tf.stamp_;
    pose.header.frame_id=frame_id;
    return pose;
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    ros::Publisher base_odom_pub =
            node.advertise<nav_msgs::Odometry>("/state_ukf/odom",50);//发布类型为速度消息类型的速度发布器

    ros::Publisher camera_odom_pub =
            node.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/camera_pose",50);//发布类型为速度消息类型的速度发布器

    tf::TransformListener listener1;
    tf::TransformListener listener2;
    ros::Rate rate(50.0);
    while(node.ok())
    {
        tf::StampedTransform transform1;
        tf::StampedTransform transform2;
        try
        {
            /* code for Try */
            listener1.waitForTransform("/local_origin","/base_link",ros::Time(0),ros::Duration(0.02));
            listener2.waitForTransform("/local_origin","/camera_link",ros::Time(0),ros::Duration(0.02));
            listener1.lookupTransform("/local_origin","/base_link",ros::Time(0),transform1);
            listener2.lookupTransform("/local_origin","/camera_link",ros::Time(0),transform2);
        }
        catch (tf::TransformException &ex)
        {
            /* code for Catch */
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.001).sleep();
            continue;
        }

        geometry_msgs::PoseStamped base_msg=tf_to_pose(transform1,"local_origin");
        nav_msgs::Odometry base_odom;
        base_odom.header=base_msg.header;
        base_odom.pose.pose=base_msg.pose;
        base_odom.child_frame_id="base_link";

        base_odom_pub.publish(base_odom);

        geometry_msgs::PoseStamped camera_msg=tf_to_pose(transform2,"camera_link");;
        camera_odom_pub.publish(camera_msg);

        rate.sleep();

    }
    return 0;
}