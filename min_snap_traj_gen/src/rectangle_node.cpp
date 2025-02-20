#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

void fillPose(const Eigen::Vector3d& vec, geometry_msgs::Pose& msg)
{
  msg.position.x = vec(0);
  msg.position.y = vec(1);
  msg.position.z = vec(2);
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
}

void fillPoseStamped(const std::string& frame_id, const ros::Time& stamp, const Eigen::Vector3d& vec,
                     geometry_msgs::PoseStamped& msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  fillPose(vec, msg.pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rectangle_node");
  ros::NodeHandle nh_;

  ros::Publisher pub_waypoints = nh_.advertise<nav_msgs::Path>("waypoints", 1, true);
  ros::Publisher pub_path = nh_.advertise<nav_msgs::Path>("path", 1, true);

  min_jerk::JerkOpt jerkOpt;
  min_jerk::Trajectory minJerkTraj;

  min_snap::SnapOpt snapOpt;
  min_snap::Trajectory minSnapTraj;

  // Eigen::MatrixXd route;
  Eigen::VectorXd ts;
  Eigen::Matrix3d iS, fS;
  Eigen::Matrix<double, 3, 4> iSS, fSS;
  iS.setZero();
  fS.setZero();
  Eigen::Vector3d zeroVec(0.0, 0.0, 0.0);
  ros::Rate lp(10);
  int groupSize = 100;

  std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
  double d0, d1;

  Eigen::Matrix<double, 3, 5> route;
  route.col(0).setZero();
  route.col(1) << 5, 0, 0;
  route.col(2) << 5, 5, 0;
  route.col(3) << 0, 5, 0;
  route.col(4).setZero();
  const int num_pieces = 4;
  Eigen::VectorXd times(num_pieces);
  times << 2, 2, 2, 2;

  std::cout << "route:\n" << route << '\n';
  std::cout << "times:\n" << times.transpose() << '\n';

  jerkOpt.reset(iS, fS, num_pieces);
  jerkOpt.generate(route.block(0, 1, 3, num_pieces - 1), times);
  jerkOpt.getTraj(minJerkTraj);

  std::cout << "Optim finished with:"
            << "\n\tduration: " << minJerkTraj.getTotalDuration() << "\n\tmax_vel: " << minJerkTraj.getMaxVelRate()
            << "\n\tmax_acc: " << minJerkTraj.getMaxAccRate() << "\n\tpositions:\n"
            << minJerkTraj.getPositions() << '\n';

  const double duration = minJerkTraj.getTotalDuration();
  const double dt = 0.1;
  double time = 0;

  // publishing
  geometry_msgs::PoseStamped ps;

  const Eigen::MatrixXd positions = minJerkTraj.getPositions();
  nav_msgs::Path wp_msg;
  wp_msg.header.frame_id = "map";
  wp_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < positions.cols(); ++i)
  {
    fillPose(positions.col(i), ps.pose);
    wp_msg.poses.push_back(ps);
  }

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();
  while (time < duration)
  {
    const Eigen::Vector3d p = minJerkTraj.getPos(time);
    const Eigen::Vector3d v = minJerkTraj.getVel(time);
    const Eigen::Vector3d a = minJerkTraj.getAcc(time);

    fillPose(p, ps.pose);

    path_msg.poses.push_back(ps);

    time += dt;
  }

  while (ros::ok())
  {
    pub_waypoints.publish(wp_msg);
    pub_path.publish(path_msg);

    ros::spinOnce();
    lp.sleep();
  }

  return 0;
}
