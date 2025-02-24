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

  double radius_{ 1.0 };
  double length_{ 50.0 };
  int num_loops_{ 1 };
  double linear_spacing_{ 4.0 };
  int num_circle_pts_{ 1 };  // not counting the endpoint
};