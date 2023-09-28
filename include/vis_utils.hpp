#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <string>
#include "min_snap.hpp"

using namespace Eigen;
using namespace std;
string sep = "\n------------------------------------------------\n";

class visUtils
{
public:
    visUtils(ros::NodeHandle *nh)
    {
        pub_wp_v = nh->advertise<visualization_msgs::MarkerArray>("wp_v", 10);
        pub_wp_id_v = nh->advertise<visualization_msgs::MarkerArray>("wp_id_v", 10);
        pub_segment_v = nh->advertise<visualization_msgs::MarkerArray>("segment_v", 10);
        pub_min_snap_traj = nh->advertise<visualization_msgs::Marker>("min_snap_traj", 10);
    }

    void visWaypoints(const MatrixXd &coords)
    {

        visualization_msgs::Marker wp;
        visualization_msgs::Marker wp_id;
        visualization_msgs::MarkerArray wp_v;
        visualization_msgs::MarkerArray wp_id_v;

        // draw waypoints as sphere markers
        wp.header.frame_id = "world";
        wp.header.stamp = ros::Time::now();
        wp.id = 0;
        wp.type = visualization_msgs::Marker::SPHERE;
        wp.action = visualization_msgs::Marker::ADD;
        wp.pose.orientation.x = 0.0;
        wp.pose.orientation.y = 0.0;
        wp.pose.orientation.z = 0.0;
        wp.pose.orientation.w = 1.0;
        wp.scale.x = 0.3;
        wp.scale.y = 0.3;
        wp.scale.z = 0.3;
        wp.color.r = 1.0;
        wp.color.g = 1.0;
        wp.color.b = 1.0;
        wp.color.a = 1.0; // Don't forget to set the alpha!

        // draw wp_id as text
        wp_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        wp_id.color.a = 1.0;
        wp_id.color.r = 1.0;
        wp_id.color.g = 1.0;
        wp_id.color.b = 1.0;
        wp_id.scale.z = 0.6;

        // cout << "Generated Waypoints: " << endl;
        // IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        // cout << coords.format(CleanFmt) << sep;

        for (int i = 0; i < coords.cols(); i++)
        {
            if (i == 0) // first wp is RED
            {
                wp.color.a = 1.0;
                wp.color.r = 1.0;
                wp.color.g = 0.0;
                wp.color.b = 0.0;
                wp.scale.x = 0.7;
                wp.scale.y = 0.7;
                wp.scale.z = 0.7;
            }
            else if (i == coords.cols() - 1) // last wp is GREEN
            {
                wp.color.a = 1.0;
                wp.color.r = 0.0;
                wp.color.g = 1.0;
                wp.color.b = 0.0;
                wp.scale.x = 0.7;
                wp.scale.y = 0.7;
                wp.scale.z = 0.7;
            }
            else
            {
                wp.scale.x = 0.65;
                wp.scale.y = 0.65;
                wp.scale.z = 0.65;
                wp.color.r = 1.0;
                wp.color.g = 1.0;
                wp.color.b = 1.0;
                wp.color.a = 1.0; // Don't forget to set the alpha!
            }
            wp.id++;
            wp.pose.position.x = coords(0, i);
            wp.pose.position.y = coords(1, i);
            wp.pose.position.z = coords(2, i);
            wp_v.markers.push_back(wp);

            wp_id.header = wp.header;
            wp_id.id++;
            wp_id.pose = wp.pose;
            wp_id.pose.position.z -= 0.75;
            wp_id.text = to_string(i);
            wp_id_v.markers.push_back(wp_id);
        }

        pub_wp_v.publish(wp_v);
        pub_wp_id_v.publish(wp_id_v);
        wp_v.markers.clear();
        wp_id_v.markers.clear();
    }

    void visRawTraj(const MatrixXd &coords)
    {
        // cout << coords << endl;
        visualization_msgs::Marker segment;
        visualization_msgs::MarkerArray segment_v;

        segment.header.frame_id = "world";
        segment.header.stamp = ros::Time::now();
        segment.id = 0;
        segment.type = visualization_msgs::Marker::LINE_LIST;
        segment.action = visualization_msgs::Marker::ADD;
        segment.pose.orientation.x = 0.0;
        segment.pose.orientation.y = 0.0;
        segment.pose.orientation.z = 0.0;
        segment.pose.orientation.w = 1.0;
        segment.scale.x = 0.2;
        segment.scale.y = 0.2;
        segment.scale.z = 0.2;
        segment.color.a = 1.0; // Don't forget to set the alpha!
        segment.color.r = 0.5;
        segment.color.g = 1.0;
        segment.color.b = 0.5;

        // cout << "visRawTraj Waypoints: " << endl;
        // IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        // cout << coords.format(CleanFmt) << sep;

        for (int i = 1; i < coords.cols(); i++)
        {
            segment.id++;
            geometry_msgs::Point p;
            p.x = coords(0, i);
            p.y = coords(1, i);
            p.z = coords(2, i);
            segment.points.push_back(p);
            p.x = coords(0, i - 1);
            p.y = coords(1, i - 1);
            p.z = coords(2, i - 1);
            segment.points.push_back(p);
            segment_v.markers.push_back(segment);
        }

        pub_segment_v.publish(segment_v);
        segment_v.markers.clear();
    }

    void visSnapTraj(min_snap::Trajectory &traj, const VectorXd ts)
    {
        visualization_msgs::Marker min_snap_traj;

        min_snap_traj.header.stamp = ros::Time::now();
        min_snap_traj.header.frame_id = "world";

        min_snap_traj.id = 0;
        min_snap_traj.type = visualization_msgs::Marker::SPHERE_LIST;
        min_snap_traj.action = 0;
        min_snap_traj.scale.x = 0.2;
        min_snap_traj.scale.y = 0.2;
        min_snap_traj.scale.z = 0.2;
        min_snap_traj.pose.orientation.x = 0.0;
        min_snap_traj.pose.orientation.y = 0.0;
        min_snap_traj.pose.orientation.z = 0.0;
        min_snap_traj.pose.orientation.w = 1.0;

        min_snap_traj.color.a = 1.0;
        min_snap_traj.color.r = 0.0;
        min_snap_traj.color.g = 1.0;
        min_snap_traj.color.b = 1.0;

        double traj_len = 0.0;
        int count = 0;
        Vector3d cur, pre;
        cur.setZero();
        pre.setZero();

        min_snap_traj.points.clear();
        Vector3d pos;
        geometry_msgs::Point pt;

        for (int i = 0; i < ts.size(); i++) // go through each Piece
        {
            // cout << sep;
            // cout << traj[i].getCoeffMat() << endl;
            // cout << sep;
            for (double t = 0.0; t < ts(i); t += 0.1, count += 1) // sample a bit
            {
                pos = traj[i].getPos(t);
                cur(0) = pt.x = pos(0);
                cur(1) = pt.y = pos(1);
                cur(2) = pt.z = pos(2);

                min_snap_traj.points.push_back(pt);

                if (count)
                    traj_len += (pre - cur).norm();
                pre = cur;
            }
        }
        cout << sep;
        cout << "Trajectory length: " << traj_len << endl;
        pub_min_snap_traj.publish(min_snap_traj);
    }

private:
    ros::Publisher pub_wp_v;
    ros::Publisher pub_wp_id_v;
    ros::Publisher pub_segment_v;
    ros::Publisher pub_min_snap_traj;
};

#endif