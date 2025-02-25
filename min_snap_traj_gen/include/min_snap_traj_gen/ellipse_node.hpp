#pragma once

#include "min_snap_traj_gen/trajectory_generator_base.hpp"

class Ellipse : public TrajectoryGeneratorBase
{
public:
  Ellipse(ros::NodeHandle& pnh);
  ~Ellipse()
  {
  }

  void updateWaypoints() override;

private:
  // 0th order derivative
  Eigen::Vector3d f0(const double time, const double angle, const double duration);

  double kx_;
  double ky_;
  double omega_;
  double num_loops_;
  double dt_;
  double height_gain_;  // total height gained above offset_
};