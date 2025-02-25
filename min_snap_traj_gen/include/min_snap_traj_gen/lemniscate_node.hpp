#pragma once

#include "min_snap_traj_gen/trajectory_generator_base.hpp"

class Lemniscate : public TrajectoryGeneratorBase
{
public:
  Lemniscate(ros::NodeHandle& pnh);
  ~Lemniscate()
  {
  }

  void updateWaypoints() override;

private:
  // 0th order derivative
  Eigen::Vector3d f0(const double angle);

  double kx_;
  double ky_;
  double omega_;
  double num_loops_;
  double dt_;  // TODO: check if this interfere with parent
};