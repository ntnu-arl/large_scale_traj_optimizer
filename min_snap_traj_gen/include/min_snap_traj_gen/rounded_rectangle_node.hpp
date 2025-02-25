#pragma once

#include "min_snap_traj_gen/trajectory_generator_base.hpp"

class RoundedRectangle : public TrajectoryGeneratorBase
{
public:
  RoundedRectangle(ros::NodeHandle& pnh);
  ~RoundedRectangle()
  {
  }

  void updateWaypoints() override;

private:
  void addStraight(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double spacing,
                   std::vector<Eigen::Vector3d>& wps);
  void addSemiCircle(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double angle_spacing,
                     std::vector<Eigen::Vector3d>& wps);

  double radius_;
  double length_;
  int num_loops_;
  double linear_spacing_;
  int num_circle_pts_;  // not counting the endpoint
};