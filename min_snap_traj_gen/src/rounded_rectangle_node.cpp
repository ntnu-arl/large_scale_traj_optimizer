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
  std::string temp = "/home/morten/workspaces/noetic_ws/src/large_scale_traj_optimizer/min_snap_traj_gen/"
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
  // parameters
  const double radius = 1;
  const double length = 10;
  // set start and end position
  Eigen::Matrix3d iS, fS;
  iS.col(0) << radius, -length / 2, 0;
  fS.col(0) = iS.col(0);
  // waypoints
  const int num_pieces = 6;
  Eigen::Matrix<double, 3, num_pieces - 1> route;
  route.col(0) << radius, length / 2, 0;
  route.col(1) << 0, radius + length / 2, 0;
  route.col(2) << -radius, length / 2, 0;
  route.col(3) << -radius, -length / 2, 0;
  route.col(4) << 0, -(radius + length / 2), 0;
  Eigen::VectorXd times(num_pieces);
  times << 4, 2, 2, 4, 2, 2;

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
  const double dt = 0.1;
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
