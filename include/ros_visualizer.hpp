#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;

class RosVisualizer
{
public:
    RosVisualizer(ros::NodeHandle *nh)
    {
        marker_pub = nh->advertise<visualization_msgs::Marker>("marker", 10);
        marker_array_pub = nh->advertise<visualization_msgs::MarkerArray>("marker_array", 10);
    }
    void plot(const MatrixXd &coords)
    {

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        // marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (int i = 0; i < coords.rows(); i++)
        {
            marker.id++;
            marker.pose.position.x = coords(i, 1);
            marker.pose.position.y = coords(i, 2);
            marker.pose.position.z = coords(i, 3);
            marker_array.markers.push_back(marker);
        }
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_array_pub.publish(marker_array);
        marker_array.markers.clear();
    };

private:
    ros::Publisher marker_pub;
    ros::Publisher marker_array_pub;
};

#endif