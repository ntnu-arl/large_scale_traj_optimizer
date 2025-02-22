#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

void fillPose(const Eigen::Vector3d& vec, geometry_msgs::Pose& msg)
{
  msg.position.x = vec(0);
  msg.position.y = vec(1);
  msg.position.z = vec(2);
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
}

void fillPoseStamped(const std::string& frame_id, const ros::Time& stamp, const Eigen::Vector3d& vec,
                     geometry_msgs::PoseStamped& msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  fillPose(vec, msg.pose);
}

class Trajectory
{
public:
  Trajectory() = default;
  ~Trajectory() = default;

  void add(const double time, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc)
  {
    time_.push_back(time);
    pos_.push_back(pos);
    vel_.push_back(vel);
    acc_.push_back(acc);
  }

  double getTime(const size_t i) const
  {
    return time_[i];
  }

  const Eigen::Vector3d& getPos(const size_t i) const
  {
    return pos_[i];
  }

  const Eigen::Vector3d& getVel(const size_t i) const
  {
    return vel_[i];
  }

  const Eigen::Vector3d& getAcc(const size_t i) const
  {
    return acc_[i];
  }

  size_t size() const
  {
    return time_.size();
  }

private:
  std::vector<double> time_;
  std::vector<Eigen::Vector3d> pos_;
  std::vector<Eigen::Vector3d> vel_;
  std::vector<Eigen::Vector3d> acc_;
};

void writeFile(const Trajectory& traj, const std::string& file_name = "trajectory.csv")
{
  // TODO avoid hard coding
  std::string temp =
      "/home/morten/workspaces/noetic_ws/src/large_scale_traj_optimizer/min_snap_traj_gen/"
      "trajectory.csv";
  FILE* file = fopen(temp.c_str(), "w");
  if (file != NULL)
  {
    ROS_INFO("Writing to: %s", temp.c_str());
    fprintf(file, "time,px,py,pz,vx,vy,vz,ax,ay,az\n");
    for (size_t i = 0; i < traj.size(); ++i)
    {
      const double time = traj.getTime(i);
      const Eigen::Vector3d pos = traj.getPos(i);
      const Eigen::Vector3d vel = traj.getVel(i);
      const Eigen::Vector3d acc = traj.getAcc(i);

      fprintf(file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", time, pos(0), pos(1), pos(2), vel(0), vel(1), vel(2),
              acc(0), acc(1), acc(2));
    }
  }

  ROS_INFO("Done writing");
}

// TODO: add times
void addStraight(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double velocity,
                 std::vector<Eigen::Vector3d>& wps)
{
  const double total_distance = delta.norm();
  const double dt = total_distance / velocity;
  const size_t N = (size_t)std::round(total_distance / velocity);

  Eigen::Vector3d current = start;
  for (size_t i = 0; i < N; ++i)
  {
    current += delta / N;
    wps.push_back(current);
    // times.push_back(dt);
  }

  start += delta;
}

// TODO:: add times
void addSemiCircle(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double omega,
                   std::vector<Eigen::Vector3d>& wps)
{
  const size_t N = (size_t)std::round(M_PI / omega);

  const Eigen::Vector3d finish = start + delta;
  const Eigen::Vector3d center = (finish + start) / 2;
  const double radius = delta.norm() / 2;
  const double phase = std::atan2(start(1) - center(1), start(0) - center(0));

  for (size_t i = 1; i <= N; ++i)
  {
    const double angle = omega * i + phase;
    const double x = radius * std::cos(angle) + center(0);
    const double y = radius * std::sin(angle) + center(1);
    const double z = center(2);
    // std::cout << "x: " << x << " y: " << y << '\n';
    wps.emplace_back(x, y, z);
    // times.push_back(1);  // REVIEW: not sure why dt is 1
  }

  start += delta;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle nh_;

  ros::Publisher pub_waypoints = nh_.advertise<nav_msgs::Path>("waypoints", 1, true);
  ros::Publisher pub_path = nh_.advertise<nav_msgs::Path>("path", 1, true);

  min_jerk::JerkOpt jerkOpt;
  min_jerk::Trajectory minJerkTraj;
  ros::Rate lp(10);

  // create trajectory
  std::vector<Eigen::Vector3d> waypoint_vector;
  std::vector<double> time_vector;
  // parameters
  const double radius = 1;
  const double length = 10;
  // set start and end position/velocity/acceleration
  const Eigen::Vector3d start(radius, -length / 2, 0);
  Eigen::Matrix3d iS, fS;
  iS.col(0) = start;
  fS.col(0) = iS.col(0);
  waypoint_vector.push_back(start);
  // filling remainder of trajectory
  const double velocity = 1.0;
  const double omega = 45.0 * M_PI / 180.0;
  Eigen::Vector3d next = start;
  addStraight(next, { 0, length, 0 }, velocity, waypoint_vector);
  addSemiCircle(next, { -2 * radius, 0, 0 }, omega, waypoint_vector);
  addStraight(next, { 0, -length, 0 }, velocity, waypoint_vector);
  addSemiCircle(next, { 2 * radius, 0, 0 }, omega, waypoint_vector);

  for (const auto& wp : waypoint_vector)
  {
    time_vector.push_back(0.75);
  }

  // const int num_pieces = 6;
  // Eigen::Matrix<double, 3, num_pieces - 1> route;
  // route.col(0) << radius, length / 2, 0;
  // route.col(1) << 0, radius + length / 2, 0;
  // route.col(2) << -radius, length / 2, 0;
  // route.col(3) << -radius, -length / 2, 0;
  // route.col(4) << 0, -(radius + length / 2), 0;
  // Eigen::VectorXd times(num_pieces);
  // times << 4, 2, 2, 4, 2, 2;

  std::cout << "wp size: " << waypoint_vector.size() << " times size: " << time_vector.size() << '\n';

  const int num_pieces = time_vector.size();
  Eigen::VectorXd times(num_pieces);
  Eigen::MatrixXd route(3, num_pieces - 1);
  std::cout << "waypoints\n";
  for (const auto& wp : waypoint_vector)
  {
    std::cout << wp.transpose() << '\n';
  }
  std::cout << '\n';
  for (const auto& t : time_vector)
  {
    std::cout << t << '\t';
  }
  std::cout << '\n';

  for (int i = 0; i < num_pieces; ++i)
  {
    times(i) = time_vector[i];
    // because waypoint_vector includes start
    if (i != 0)
    {
      route.col(i - 1) = waypoint_vector[i];
    }
  }

  std::cout << "route:\n" << route << '\n';
  std::cout << "times:\n" << times.transpose() << '\n';

  jerkOpt.reset(iS, fS, num_pieces);
  jerkOpt.generate(route, times);
  jerkOpt.getTraj(minJerkTraj);

  std::cout << "Optim finished with:"
            << "\n\tduration: " << minJerkTraj.getTotalDuration() << "\n\tmax_vel: " << minJerkTraj.getMaxVelRate()
            << "\n\tmax_acc: " << minJerkTraj.getMaxAccRate() << "\n\tpositions:\n"
            << minJerkTraj.getPositions() << '\n';

  const double duration = minJerkTraj.getTotalDuration();
  const double dt = 0.01;
  double time = 0;

  // publishing
  geometry_msgs::PoseStamped ps;

  const Eigen::MatrixXd positions = minJerkTraj.getPositions();
  nav_msgs::Path wp_msg;
  wp_msg.header.frame_id = "map";
  wp_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < positions.cols(); ++i)
  {
    fillPose(positions.col(i), ps.pose);
    wp_msg.poses.push_back(ps);
  }

  Trajectory traj;
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();
  while (time < duration)
  {
    const Eigen::Vector3d p = minJerkTraj.getPos(time);
    const Eigen::Vector3d v = minJerkTraj.getVel(time);
    const Eigen::Vector3d a = minJerkTraj.getAcc(time);

    traj.add(time, p, v, a);
    fillPose(p, ps.pose);

    path_msg.poses.push_back(ps);

    time += dt;
  }

  writeFile(traj);

  while (ros::ok())
  {
    pub_waypoints.publish(wp_msg);
    pub_path.publish(path_msg);

    ros::spinOnce();
    lp.sleep();
  }

  return 0;
}
