// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
// System includes
#include <string>
using namespace std;

geometry_msgs::PointStamped vel_pub;
geometry_msgs::PoseStamped pos_pub;
geometry_msgs::Point position_sub;

double dt = 1.0/120.0;
//for position Kalman Filter
double kp, kq, kr;
Eigen::VectorXd x_e_(6);
Eigen::VectorXd x_e(6);
Eigen::Vector3d z_m;
Eigen::MatrixXd F(6, 6);
Eigen::MatrixXd Tao(6, 6);
Eigen::MatrixXd H(3, 6);
Eigen::MatrixXd I_6(6, 6);
Eigen::MatrixXd I_3(3, 3);


Eigen::MatrixXd P_(6, 6);
Eigen::MatrixXd P(6, 6);
Eigen::MatrixXd Q(6, 6);
Eigen::Matrix3d R(3, 3);
Eigen::MatrixXd K(6, 6);

void GetVelCallBack(const geometry_msgs::PoseStamped::ConstPtr msg);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "get_vel_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    pnh.param("kp", kp, 0.01);
    pnh.param("kq", kq, 0.01);
    pnh.param("kr", kr, 1.0);
    ros::Subscriber robot_pos_sub = nh.subscribe("/Robot/pose",1000, GetVelCallBack);
    ros::Publisher  robot_vel_pub = nh.advertise<geometry_msgs::PointStamped>("/Robot/vel",1000);
    ros::Publisher  robot_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/Robot/mod_pos", 1000);
    ros::Rate loopRate(120);

    F<< 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 

    Tao<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    H<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    I_6<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    I_3<< 1.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0; 
          
    x_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
    P = kp * I_6;
    Q = kq * I_6;
    R = kr * I_3;


    while(ros::ok())
    {
        robot_vel_pub.publish(vel_pub);
        robot_pos_pub.publish(pos_pub);
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}


void GetVelCallBack(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    position_sub = msg->pose.position;

	 // Kalman Filter
	z_m(0) = position_sub.x;
	z_m(1) = position_sub.y;
	z_m(2) = position_sub.z;
	x_e_ = (dt*F + I_6) * x_e ;
	P_   = F * P * F.transpose() + Tao * Q * Tao.transpose();
	K    = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
	x_e  = x_e_ + K * (z_m - H * x_e_);
	P    = (I_6 - K * H) * P_;

 
    pos_pub.header.frame_id = "robot_pos";
    pos_pub.header.stamp = ros::Time::now();
    pos_pub.pose.position.x = x_e(0);
    pos_pub.pose.position.y = x_e(1);
    pos_pub.pose.position.z = x_e(2);
    pos_pub.pose.orientation = msg->pose.orientation;

    vel_pub.header.frame_id = "robot_vel";
    vel_pub.header.stamp = pos_pub.header.stamp; 
    vel_pub.point.x = x_e(3);
    vel_pub.point.y = x_e(4);
    vel_pub.point.z = x_e(5);
}