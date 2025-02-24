#pragma once

#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include "min_snap_traj_gen/trajectory.hpp"
#include "thirdparty/alglib-cpp/src/stdafx.h"
#include <stdlib.h>
#include "thirdparty/alglib-cpp/src/optimization.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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

  void loadParams();
  virtual void updateWaypoints();
  virtual void updateTimes();
  void run();
  bool optimize(const std::vector<double>& time_vector, const std::vector<Eigen::Vector3d>& waypoint_vector);

  std::vector<Eigen::Vector3d> waypoint_vector_;
  std::vector<double> time_vector_;

private:
  void fillPose(const Eigen::Vector3d& vec, geometry_msgs::Pose& msg);
  void fillPoseStamped(const std::string& frame_id, const ros::Time& stamp, const Eigen::Vector3d& vec,
                       geometry_msgs::PoseStamped& msg);
  void updateMessages();
  void publishOnTimer();
  // TODO avoid hard coding
  void writeFile(const Trajectory& traj, const std::string& file_name =
                                             "/home/morten/workspaces/noetic_ws/src/large_scale_traj_optimizer/"
                                             "min_snap_traj_gen/trajectory.csv");

  double dt_;
  double takeoff_height_;

  nav_msgs::Path wp_msg_;
  nav_msgs::Path path_msg_;
  // trajectory_msgs::Multidof traj_msg_; // TODO

  ros::Publisher pub_waypoints_;
  ros::Publisher pub_path_;
  // ros::Publisher pub_trajectory_; // TODO

  ros::Timer timer_publish_;
};