#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "AstarWoMap.h"
#include "OccMapTransform.h"

#include <Eigen/Geometry>

using namespace cv;
using namespace std;

// TODO
    // obstacle definition 여기에서 해야 함
    // gazebo 상에서 테스트 해볼 수 있으면 좋을텐데.. gazebo에서 사물 부를 수 있으면 좋겠다
    // turtlebot3 사용해서 테스트 해볼 것 


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber obs_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher path_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
nav_msgs::Odometry odom;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;

// Parameter
bool obstacle_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;
pathplanning::Obs obs;
double map_abs_height;
double map_abs_width;
double map_grid_length; 
Eigen::Vector3d obs_CoG;
double obs_yaw;
double obs_abs_width;
double obs_abs_depth;

//-------------------------------- utility function ---------------------------------//
Eigen::Matrix<double,3,1> R2rpy( Eigen::Matrix<double,3,3> R )
{
        double r = atan2(R(2,1), R(2,2));
        double p = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
        double y = atan2(R(1,0), R(0,0));

        Eigen::Matrix<double,3,1> euler;
        euler(0,0) = r;
        euler(1,0) = p;
        euler(2,0) = y;

        return euler;
}

//-------------------------------- Callback function ---------------------------------//
void ObstacleCallback(const geometry_msgs::PoseStamped& msg)
{ // XXX - only one obstacle is assumed
    obs_CoG(0) = msg.pose.position.x;
    obs_CoG(1) = msg.pose.position.y;
    obs_CoG(2) = msg.pose.position.z;

    Eigen::Quaternion<double> quat_;
    quat_.w() = msg.pose.orientation.w;
    quat_.x() = msg.pose.orientation.x;
    quat_.y() = msg.pose.orientation.y;
    quat_.z() = msg.pose.orientation.z;

    Eigen::Matrix<double,3,3> R = quat_.toRotationMatrix();
    Eigen::Matrix<double,3,1> rpy = R2rpy(R);
    
    obs_yaw = rpy(2);
}

void StartPointCallback(const nav_msgs::Odometry& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    if(obstacle_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(obstacle_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    obstacle_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;
    obs_CoG = Eigen::Matrix<double,3,1>::Zero();
    obs_yaw = 0.0;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<double>("InflateLength", config.InflateLength, -1.0);
    nh_priv.param<int>("rate", rate, 10);
    nh_priv.param<double>("map_abs_height", map_abs_height, 4.0);
    nh_priv.param<double>("map_abs_width", map_abs_width, 5.0);
    nh_priv.param<double>("obs_abs_width", obs_abs_width, 0.7);
    nh_priv.param<double>("obs_abs_depth", obs_abs_depth, 0.5);
    nh_priv.param<double>("map_grid_length", map_grid_length, 0.05);

    // Subscribe topics
    obs_sub = nh.subscribe("/structure", 10, ObstacleCallback);
    startPoint_sub = nh.subscribe("/mavros/local_position/odom", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Advertise topics
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);

    // Initialize Astar algorithm
    obs.CoG = obs_CoG;
    obs.yaw = obs_yaw;

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(start_flag)
        {
            astar.InitAstar(obs, map_abs_height, map_abs_width, map_grid_length, config);

            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;
            astar.PathPlanning(startPoint, targetPoint, PathList);
            if(!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for(int i=0;i<PathList.size();i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();

                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            startpoint_flag = false;
            targetpoint_flag = false;
            start_flag = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
