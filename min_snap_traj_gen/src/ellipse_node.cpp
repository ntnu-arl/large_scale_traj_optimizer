#include "min_snap_traj_gen/ellipse_node.hpp"

Ellipse::Ellipse(ros::NodeHandle& pnh) : TrajectoryGeneratorBase(pnh)
{
  // parameters
  pnh.param<double>("scale_x", kx_, 1.0);
  pnh.param<double>("scale_y", ky_, 1.0);
  pnh.param<double>("omega", omega_, 1.0);
  pnh.param<double>("num_loops", num_loops_, 3.0);
  pnh.param<double>("wp_dt", dt_, 0.25);
  pnh.param<double>("height_gain", height_gain_, 0.0);
}

void Ellipse::updateWaypoints()
{
  const int N = int(num_loops_ * 2 * M_PI / (omega_ * dt_) + 1);
  const double duration = N * dt_;
  for (int i = 0; i < N; ++i)
  {
    const double time = i * dt_;
    const double angle = time * omega_;  // TODO: add phase?

    waypoint_vector_.push_back(f0(time, angle, duration));
  }
}

Eigen::Vector3d Ellipse::f0(const double time, const double angle, const double duration)
{
  const double x = kx_ * std::cos(angle);
  const double y = ky_ * std::sin(angle);
  const double z = interpolateHeight(time, duration, height_gain_);

  return Eigen::Vector3d(x, y, z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle pnh("~");

  Ellipse node(pnh);
  node.run();

  ros::spin();

  return 0;
}
