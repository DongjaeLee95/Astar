#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


// subscriber
ros::Subscriber obs_sub;

bool obs_cb_flag;
double obs_abs_width;
double obs_abs_depth;
double obs_abs_height;
Eigen::Vector3d obs_CoG;
Eigen::Quaternion<double> obs_quat;

// callback function
void ObsCb(const geometry_msgs::PoseStamped& msg)
{ // XXX - only one obstacle is assumed
    obs_CoG(0) = msg.pose.position.x;
    obs_CoG(1) = msg.pose.position.y;
    obs_CoG(2) = msg.pose.position.z;

    obs_quat.w() = msg.pose.orientation.w;
    obs_quat.x() = msg.pose.orientation.x;
    obs_quat.y() = msg.pose.orientation.y;
    obs_quat.z() = msg.pose.orientation.z;

    obs_cb_flag = true;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "obs_visualize");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");

    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    nh_priv.param<double>("obs_abs_width", obs_abs_width, 0.7);
    nh_priv.param<double>("obs_abs_depth", obs_abs_depth, 0.5);
    nh_priv.param<double>("obs_abs_height", obs_abs_height, 0.7);

    obs_sub = n.subscribe("/structure",1,ObsCb);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        if(obs_cb_flag == true)
        {
            marker.pose.position.x = obs_CoG(0);
            marker.pose.position.y = obs_CoG(1);
            marker.pose.position.z = obs_CoG(2);
            marker.pose.orientation.x = obs_quat.x();
            marker.pose.orientation.y = obs_quat.y();
            marker.pose.orientation.z = obs_quat.z();
            marker.pose.orientation.w = obs_quat.w();
        }
        else
        {
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        }
        

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = obs_abs_width;
        marker.scale.y = obs_abs_depth;
        marker.scale.z = obs_abs_height;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();
        ros::spinOnce();
    }
}