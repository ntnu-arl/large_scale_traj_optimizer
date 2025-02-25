#include "min_snap_traj_gen/lemniscate_node.hpp"

Lemniscate::Lemniscate(ros::NodeHandle& pnh) : TrajectoryGeneratorBase(pnh)
{
  // parameters
  pnh.param<double>("scale_x", kx_, 1.0);
  pnh.param<double>("scale_y", ky_, 1.0);
  pnh.param<double>("omega", omega_, 1.0);
  pnh.param<double>("num_loops", num_loops_, 3.0);
  pnh.param<double>("wp_dt", dt_, 0.25);
}

void Lemniscate::updateWaypoints()
{
  const int N = int(num_loops_ * 2 * M_PI / (omega_ * dt_) + 1);
  for (int i = 0; i < N; ++i)
  {
    const double angle = i * omega_ * dt_;  // TODO: add phase?

    waypoint_vector_.push_back(f0(angle));
  }
}

Eigen::Vector3d Lemniscate::f0(const double angle)
{
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  const double denom = 1 + std::pow(sa, 2);

  const double x = kx_ * ca / denom;
  const double y = ky_ * ca * sa / denom;

  return Eigen::Vector3d(x, y, 0.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle pnh("~");

  Lemniscate node(pnh);
  node.run();

  ros::spin();

  return 0;
}
