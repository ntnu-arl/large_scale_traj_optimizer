#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

class RosVisualizer
{
public:
    RosVisualizer(ros::NodeHandle *nh)
    {
        marker_pub = nh->advertise<visualization_msgs::Marker>("marker", 10);
    }
    void plot()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        // marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        std::cout << "Publishing marker" << std::endl;
        marker_pub.publish(marker);
    };

private:
    ros::Publisher marker_pub;
    //  = nh_.advertise<visualization_msgs::Marker>("waypoints", 10);
};

#endif