// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// System includes
#include <iostream>
using namespace std;

geometry_msgs::TransformStamped tf_fake_gps;
geometry_msgs::PoseStamped pos_mocap;


void Pose_Mocap_CallBack(const geometry_msgs::PoseStamped::ConstPtr msg);
ros::Publisher  fake_gps_pub;
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mocap_fake_gps_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber mocap_pos_sub = nh.subscribe("/Robot/pose",100, Pose_Mocap_CallBack);
    fake_gps_pub = nh.advertise<geometry_msgs::TransformStamped>("/mocap_fake_gps",100);
    ros::spin();
    return 0;
}


void Pose_Mocap_CallBack(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    tf_fake_gps.header = msg->header;
    tf_fake_gps.child_frame_id = "/mocap/intel_rtf";
    tf_fake_gps.transform.translation.x = msg->pose.position.x;
    tf_fake_gps.transform.translation.y = msg->pose.position.y;
    tf_fake_gps.transform.translation.z = msg->pose.position.z;

    tf_fake_gps.transform.rotation.w = msg->pose.orientation.w;
    tf_fake_gps.transform.rotation.x = msg->pose.orientation.x;
    tf_fake_gps.transform.rotation.y = msg->pose.orientation.y;
    tf_fake_gps.transform.rotation.z = msg->pose.orientation.z;
    fake_gps_pub.publish(tf_fake_gps);
    // cout<<"GPS=:"<<endl<<tf_fake_gps<<endl; 
}