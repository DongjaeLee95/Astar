#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

double psi;
double PI = 3.14159265359;

int main( int argc, char** argv)
{
    ros::init(argc, argv, "publish_simul");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::Rate rate(10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom",1);
    ros::Publisher obs_pub = nh.advertise<geometry_msgs::PoseStamped>("/dj_struct",1);

    nh_priv.param<double>("yaw_angle",psi,0.0); // [deg]
    psi = psi*PI/180.0;

    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped obs_pose_;

    odom_.header.frame_id = "map";
    odom_.header.seq = 0;
    
    obs_pose_.header.frame_id = "map";
    obs_pose_.header.seq = 0;

    odom_.pose.pose.position.x = 0.0;
    odom_.pose.pose.position.y = 0.0;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation.x = 0.0;
    odom_.pose.pose.orientation.y = 0.0;
    odom_.pose.pose.orientation.z = 0.0;
    odom_.pose.pose.orientation.w = 1.0;

    obs_pose_.pose.position.x = 1.0;
    obs_pose_.pose.position.y = 0.0;
    obs_pose_.pose.position.z = 0.35;

    Eigen::Matrix<double,3,3> R;
    R(0,0) = cos(psi);  R(0,1) = -sin(psi); R(0,2) = 0.0;
    R(1,0) = sin(psi);  R(1,1) = cos(psi);  R(1,2) = 0.0;
    R(2,0) = 0.0;       R(2,1) = 0.0;       R(2,2) = 1.0;
    Eigen::Quaternion<double> quat(R);

    obs_pose_.pose.orientation.w = quat.w();
    obs_pose_.pose.orientation.x = quat.x();
    obs_pose_.pose.orientation.y = quat.y();
    obs_pose_.pose.orientation.z = quat.z();

    while(ros::ok())
    {
        odom_.header.stamp = ros::Time::now();
        obs_pose_.header.stamp = ros::Time::now();

        odom_pub.publish(odom_);
        obs_pub.publish(obs_pose_);   

        odom_.header.seq += 1;
        obs_pose_.header.seq += 1;

        rate.sleep();
    }

    return 0;
}