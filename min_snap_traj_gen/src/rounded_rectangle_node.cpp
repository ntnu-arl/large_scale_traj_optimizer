#include "min_snap_traj_gen/rounded_rectangle_node.hpp"

RoundedRectangle::RoundedRectangle(ros::NodeHandle& pnh) : TrajectoryGeneratorBase(pnh)
{
  // parameters
  pnh.param<double>("radius", radius_, 1.0);
  pnh.param<double>("length", length_, 50.0);
  pnh.param<int>("num_loops", num_loops_, 1);
  pnh.param<double>("linear_spacing", linear_spacing_, 4.0);
  pnh.param<int>("num_circle_points", num_circle_pts_, 1);
}

void RoundedRectangle::addStraight(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double spacing,
                                   std::vector<Eigen::Vector3d>& wps)
{
  const double total_distance = delta.norm();
  const size_t N = (size_t)std::round(total_distance / spacing);

  Eigen::Vector3d current = start;
  for (size_t i = 0; i < N; ++i)
  {
    current += delta / N;
    wps.push_back(current);
  }

  start += delta;
}

void RoundedRectangle::addSemiCircle(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double angle_spacing,
                                     std::vector<Eigen::Vector3d>& wps)
{
  const size_t N = (size_t)std::round(M_PI / angle_spacing);

  const Eigen::Vector3d finish = start + delta;
  const Eigen::Vector3d center = (finish + start) / 2;
  const double radius = delta.norm() / 2;
  const double phase = std::atan2(start(1) - center(1), start(0) - center(0));

  for (size_t i = 0; i < N; ++i)
  {
    const double angle = angle_spacing * (i + 1) + phase;
    const double x = radius * std::cos(angle) + center(0);
    const double y = radius * std::sin(angle) + center(1);
    const double z = center(2);

    wps.emplace_back(x, y, z);
  }

  start = finish;
}

void RoundedRectangle::updateWaypoints()
{
  // set start and end position/velocity/acceleration
  const Eigen::Vector3d start(radius_, -length_ / 2, 0);
  iS_.setZero();
  fS_.setZero();
  iS_.col(0) = start;
  fS_.col(0) = iS_.col(0);
  waypoint_vector_.push_back(start);
  // filling remainder of trajectory
  const double angle_spacing = M_PI / (num_circle_pts_ + 1);
  Eigen::Vector3d next = start;

  const int num_loops_ = 3;
  for (int i = 0; i < num_loops_; ++i)
  {
    addStraight(next, { 0, length_, 0 }, linear_spacing_, waypoint_vector_);
    addSemiCircle(next, { -2 * radius_, 0, 0 }, angle_spacing, waypoint_vector_);
    addStraight(next, { 0, -length_, 0 }, linear_spacing_, waypoint_vector_);
    addSemiCircle(next, { 2 * radius_, 0, 0 }, angle_spacing, waypoint_vector_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle pnh("~");

  RoundedRectangle node(pnh);
  node.run();

  ros::spin();

  return 0;
}
