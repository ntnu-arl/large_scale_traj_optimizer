#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Path.h> // Include header for Path message
#include "min_snap_manager.hpp"
#include "min_snap.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <Eigen/Dense>

using namespace std;
using namespace ros;
using namespace Eigen;

// Function to convert waypoints to Eigen Matrix
MatrixXd convertToEigenMatrix(const trajectory_msgs::MultiDOFJointTrajectory &trajectory)
{
    int num_points = trajectory.points.size()-1;
    MatrixXd route(3, num_points); // Assuming 3D points (x, y, z)

    for (int i = 0; i < num_points; ++i)
    {
        // Extract the positions (x, y, z) from trajectory
        route(0, i) = trajectory.points[i+1].transforms[0].translation.x;
        route(1, i) = trajectory.points[i+1].transforms[0].translation.y;
        route(2, i) = trajectory.points[i+1].transforms[0].translation.z;
    }

    return route;
}

class TrajectorySmoother
{
public:
    TrajectorySmoother(ros::NodeHandle &nh)
    {
        trajectory_sub_ = nh.subscribe("/command/trajectory", 1, &TrajectorySmoother::trajectoryCallback, this);
        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory_smooth", 10);

        // Add publishers for Path messages
        original_path_pub_ = nh.advertise<nav_msgs::Path>("/path/original", 10);
        smooth_path_pub_ = nh.advertise<nav_msgs::Path>("/path/smooth", 10);
    }

    trajectory_msgs::MultiDOFJointTrajectory generateMinimumSnapTrajectory(
        const trajectory_msgs::MultiDOFJointTrajectory &non_smooth_trajectory, TrajectorySmoother &smoother)
    {
        // ROS_INFO("Original Traj Received!");

        trajectory_msgs::MultiDOFJointTrajectory smooth_trajectory;

        // Initialize the min snap optimizer
        min_snap::SnapOpt snapOpt;
        min_snap::Trajectory minSnapTraj;
        MatrixXd route = convertToEigenMatrix(non_smooth_trajectory); // Convert trajectory to Eigen format
        VectorXd ts;
        Matrix3d iS, fS;

        Eigen::Matrix<double, 3, 4> iSS, fSS;
        iS.setZero();
        fS.setZero();
        Vector3d zeroVec(0.0, 0.0, 0.0);

        double max_vel = 1.0;
        double max_acc = 3.0;

        iS.col(0) << route.leftCols<1>();
        fS.col(0) << route.rightCols<1>();

        // ROS_INFO("------------");
        // cout << route << endl;
        // ROS_INFO("------------");

        iSS << iS, Eigen::MatrixXd::Zero(3, 1); // Start state matrix
        fSS << fS, Eigen::MatrixXd::Zero(3, 1); // End state matrix

        // Allocate time based on the route
        ts = smoother.allocateTime(route, max_vel, max_acc);
        std::cout << ts.transpose() << std::endl;

        // Generate minimum snap trajectory
        snapOpt.reset(iSS, fSS, route.cols() - 1); // Reset optimizer with start and end states
        snapOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
        snapOpt.getTraj(minSnapTraj); // Get the optimized trajectory

        Vector3d pos;
        smooth_trajectory.header = non_smooth_trajectory.header;
        smooth_trajectory.joint_names = non_smooth_trajectory.joint_names; // Copy joint names

        for (int i = 0; i < ts.size(); i++) // go through each Piece
        {

            for (double t = 0.0; t < ts(i); t += 0.2) // sample a bit
            {
                trajectory_msgs::MultiDOFJointTrajectoryPoint point;
                geometry_msgs::Transform transform;
                pos = minSnapTraj[i].getPos(t);

                transform.translation.x = pos(0);
                transform.translation.y = pos(1);
                transform.translation.z = pos(2);
                transform.rotation.w = 1.0; // Assuming no rotation, set to default

                point.transforms.push_back(transform);
                smooth_trajectory.points.push_back(point);
            }
        }

        return smooth_trajectory;
    }

    VectorXd allocateTime(const MatrixXd &wayPs, double vel, double acc)
    {
        int N = (int)(wayPs.cols()) - 1;
        VectorXd durations(N);
        if (N > 0)
        {
            Eigen::Vector3d p0, p1;
            double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
            for (int k = 0; k < N; k++)
            {
                p0 = wayPs.col(k);
                p1 = wayPs.col(k + 1);
                D = (p1 - p0).norm();

                acct = vel / acc;
                accd = (acc * acct * acct / 2);
                dcct = vel / acc;
                dccd = acc * dcct * dcct / 2;

                if (D < accd + dccd)
                {
                    t1 = sqrt(D / acc);
                    t2 = t1;
                    dtxyz = t1 + t2;
                }
                else
                {
                    t1 = acct;
                    t2 = (D - accd - dccd) / vel;
                    t3 = dcct;
                    dtxyz = t1 + t2 + t3;
                }

                durations(k) = dtxyz;
            }
        }

        // for(int i=0; i<N; ++i)
        // {
        //     Eigen::Vector3d p0, p1;
        //     p0 = wayPs.col(i);
        //     p1 = wayPs.col(i + 1);
        //     double D = (p1 - p0).norm();
        //     durations(i) = D/vel;
        //     std::cout << durations(i) << " ";
        // }
        // std::cout << std::endl;

        return durations;
    }

private:
    ros::Subscriber trajectory_sub_;
    ros::Publisher trajectory_pub_;

    // Add new publishers for Path messages
    ros::Publisher original_path_pub_;
    ros::Publisher smooth_path_pub_;

    // Callback for receiving the non-smooth trajectory
    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
    {
        ROS_INFO_ONCE("Received non-smooth trajectory");

        // Process the received trajectory to generate a smooth, minimum snap trajectory
        trajectory_msgs::MultiDOFJointTrajectory smooth_trajectory = generateMinimumSnapTrajectory(*msg, *this);

        // Publish the smooth trajectory
        trajectory_pub_.publish(smooth_trajectory);
        // ROS_INFO("Published smooth trajectory");

        // Convert the original trajectory to a Path message and publish it
        nav_msgs::Path original_path;
        original_path.header = msg->header;
        for (const auto &point : msg->points)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose.position.x = point.transforms[0].translation.x;
            pose.pose.position.y = point.transforms[0].translation.y;
            pose.pose.position.z = point.transforms[0].translation.z;
            pose.pose.orientation.w = 1.0;
            original_path.poses.push_back(pose);
        }
        original_path_pub_.publish(original_path);

        // Convert the smoothed trajectory to a Path message and publish it
        nav_msgs::Path smooth_path;
        smooth_path.header = smooth_trajectory.header;
        for (const auto &point : smooth_trajectory.points)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = smooth_trajectory.header;
            pose.pose.position.x = point.transforms[0].translation.x;
            pose.pose.position.y = point.transforms[0].translation.y;
            pose.pose.position.z = point.transforms[0].translation.z;
            pose.pose.orientation.w = 1.0;
            smooth_path.poses.push_back(pose);
        }
        smooth_path_pub_.publish(smooth_path);
    }
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "trajectory_smoother");
    ros::NodeHandle nh;

    // Create the trajectory smoother object
    TrajectorySmoother smoother(nh);

    // Spin to process incoming trajectories
    ros::spin();

    return 0;
}
