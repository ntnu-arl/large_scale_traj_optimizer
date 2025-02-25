#pragma once

#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include "min_snap_traj_gen/trajectory.hpp"
// TODO: replace with more lightweight optimzier
#include "thirdparty/alglib-cpp/src/stdafx.h"
#include "thirdparty/alglib-cpp/src/optimization.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// global variables
extern min_jerk::JerkOpt jerkOpt_;
extern min_jerk::Trajectory minJerkTraj_;
extern Eigen::Matrix3d iS_;
extern Eigen::Matrix3d fS_;
extern Eigen::VectorXd times_;
extern Eigen::MatrixXd waypoints_;
extern int num_pieces_;
// parameters
extern double rho_t_;
extern double rho_v_;
extern double rho_a_;
extern double vmax_;
extern double amax_;

inline double g(const double x);
void objectiveFunction(const alglib::real_1d_array& x, double& func, void* ptr);

class TrajectoryGeneratorBase
{
public:
  TrajectoryGeneratorBase(ros::NodeHandle& pnh);
  ~TrajectoryGeneratorBase()
  {
  }

  virtual void updateWaypoints();
  virtual void updateStartFinish();
  virtual void updateTimes();
  virtual void rotateWaypoints();
  double interpolateHeight(const double time, const double duration, const double height_gain);
  void run();
  bool optimize();

  std::vector<Eigen::Vector3d> waypoint_vector_;
  std::vector<double> time_vector_;

private:
  void fillPose(const Eigen::Vector3d& pos, geometry_msgs::Pose& msg);
  void fillPose(const Eigen::Vector3d& pos, const double yaw, geometry_msgs::Pose& msg);
  void fillMultiDOFTrajectoryPoint(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                                   const double yaw, const double yaw_rate, const double time,
                                   trajectory_msgs::MultiDOFJointTrajectoryPoint& point);

  void updateMessages();
  void publishOnTimer();
  bool takeoffService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool startService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  // TODO avoid hard coding
  void writeFile(const Trajectory& traj, const std::string& file_name =
                                             "/home/morten/workspaces/noetic_ws/src/large_scale_traj_optimizer/"
                                             "min_snap_traj_gen/trajectory.csv");

  double dt_;
  Eigen::Vector3d offset_;
  bool align_yaw_;
  bool rotate_xy_;  // flip x and y (i.e. rotate waypoints by 90 deg)

  std::string frame_id_;

  int max_iter_;
  int M_;  // recommended 3 <= M <= 7

  nav_msgs::Path wp_msg_;
  nav_msgs::Path path_msg_;
  trajectory_msgs::MultiDOFJointTrajectory takeoff_msg_;
  trajectory_msgs::MultiDOFJointTrajectory traj_msg_;

  ros::Publisher pub_waypoints_;
  ros::Publisher pub_path_;
  ros::Publisher pub_trajectory_;

  ros::ServiceServer srv_takeoff_;
  ros::ServiceServer srv_start_;

  ros::Timer timer_publish_;
};