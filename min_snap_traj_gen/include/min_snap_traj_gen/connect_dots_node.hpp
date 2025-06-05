#pragma once

#include "min_snap_traj_gen/trajectory_generator_base.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

class ConnectDots : public TrajectoryGeneratorBase
{
public:
  ConnectDots(ros::NodeHandle& pnh);
  ~ConnectDots()
  {
  }

  void updateWaypoints() override;

private:
  void markerCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

  geometry_msgs::PoseArray pose_array_;

  ros::Subscriber sub_marker_;
  ros::Publisher pub_markers_;
};