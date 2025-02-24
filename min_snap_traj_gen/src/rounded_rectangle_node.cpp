#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include "stdafx.h"
#include <stdlib.h>
#include "optimization.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

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
  fclose(file);
  ROS_INFO("Done writing");
}

// TODO: add times
void addStraight(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double spacing,
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

// TODO:: add times
void addSemiCircle(Eigen::Vector3d& start, const Eigen::Vector3d& delta, const double angle_spacing,
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

min_jerk::JerkOpt jerkOpt;
min_jerk::Trajectory minJerkTraj;
Eigen::Matrix3d iS, fS;
Eigen::VectorXd times;
Eigen::MatrixXd route;
int num_pieces;

inline double g(const double x)
{
  return std::pow(std::max(x, 0.0), 2);
}

void function2(const alglib::real_1d_array& x, double& func, void* ptr)
{
  const double rho_t = 25.0;
  const double rho_v = 200.0;
  const double rho_a = 1.0;
  const double vmax = 4.0;
  const double amax = 6.0;
  const double vmax2 = std::pow(vmax, 2);
  const double amax2 = std::pow(amax, 2);

  for (int i = 0; i < num_pieces; ++i)
  {
    times(i) = x[i];  // REVIEW: enforce positive times
  }

  Eigen::MatrixXd temp_route(route.rows(), route.cols() - 2);
  for (int i = 1; i < route.cols() - 1; ++i)
  {
    temp_route.col(i - 1) = route.col(i);
  }
  static bool first = true;
  if (first)
  {
    first = false;
    std::cout << "route/times/temp: " << route.cols() << '\t' << times.size() << '\t' << temp_route.cols() << '\n';
  }

  jerkOpt.reset(iS, fS, num_pieces);
  jerkOpt.generate(temp_route, times);
  jerkOpt.getTraj(minJerkTraj);

  // calculate objective
  const double J_sigma = jerkOpt.getObjective();
  double J_D_t = 0;
  for (int i = 0; i < num_pieces; ++i)
  {
    J_D_t += times(i);
  }
  double J_D_v = 0;
  double J_D_a = 0;
  for (int i = 0; i < num_pieces - 1; ++i)
  {
    // T_{i} corresponds to q_{i} to q_{i+1} instead of q_{i-1} to q_{i}
    const int j = i + 1;
    // get variables of interest (index according to fig 4 of https://arxiv.org/pdf/2011.02662)
    const Eigen::Vector3d q_im1 = route.col(j - 1);  // q_{i-1}
    const Eigen::Vector3d q_i = route.col(j);        // q_{i}
    const Eigen::Vector3d q_ip1 = route.col(j + 1);  // q_{i+1}
    const double T_i = times(i);                     // T_{i}
    const double T_ip1 = times(i + 1);               // T_{i+1}

    J_D_v += g(((q_ip1 - q_im1) / (T_ip1 + T_i)).squaredNorm() - vmax2);
    J_D_a += g((((q_ip1 - q_i) / T_ip1 - (q_i - q_im1) / T_i) / ((T_ip1 + T_i) / 2.0)).squaredNorm() - amax2);
  }
  // std::cout << "sigma/t/v/a: " << J_sigma << '\t' << J_D_t << '\t' << J_D_v << '\t' << J_D_a << '\n';
  const double J_D = rho_t * J_D_t + rho_v * J_D_v + rho_a * J_D_a;

  func = J_sigma + J_D;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle nh_;

  ros::Publisher pub_waypoints = nh_.advertise<nav_msgs::Path>("waypoints", 1, true);
  ros::Publisher pub_path = nh_.advertise<nav_msgs::Path>("path", 1, true);
  ros::Rate lp(10);

  // create trajectory
  std::vector<Eigen::Vector3d> waypoint_vector;
  std::vector<double> time_vector;
  // parameters
  const double radius = 1;
  const double length = 50;
  // set start and end position/velocity/acceleration
  const Eigen::Vector3d start(radius, -length / 2, 0);
  iS.setZero();
  fS.setZero();
  iS.col(0) = start;
  fS.col(0) = iS.col(0);
  waypoint_vector.push_back(start);
  // filling remainder of trajectory
  const double max_speed = 3.75;
  const double max_accel = 3.0;
  const double linear_spacing = max_speed;
  const size_t N_circle = 1;  // not counting the endpoint
  const double angle_spacing = M_PI / (N_circle + 1);
  Eigen::Vector3d next = start;

  const int N_loops = 3;
  for (int i = 0; i < N_loops; ++i)
  {
    addStraight(next, { 0, length, 0 }, linear_spacing, waypoint_vector);
    addSemiCircle(next, { -2 * radius, 0, 0 }, angle_spacing, waypoint_vector);
    addStraight(next, { 0, -length, 0 }, linear_spacing, waypoint_vector);
    addSemiCircle(next, { 2 * radius, 0, 0 }, angle_spacing, waypoint_vector);
  }

  double prev_speed = 0.0;
  // assuming waypoint vector contains start and stop
  for (size_t i = 1; i < waypoint_vector.size(); ++i)
  {
    // limit acceleration in startup
    const double vmax = std::min(max_speed, std::sqrt(std::pow(prev_speed, 2) + 2 * max_accel * linear_spacing));
    const double t_accel = (vmax - prev_speed) / max_accel;
    const double t_decel = 0.0;  // vmax / max_accel;
    const double t_cruise = std::max(0.0, (linear_spacing - 0.5 * max_accel * std::pow(t_accel + t_decel, 2)) / vmax);

    // std::cout << vmax << '\t' << t_accel << '\t' << t_decel << '\t' << t_cruise << '\n';

    time_vector.push_back(t_accel + t_cruise + t_decel);
    prev_speed = vmax;
  }
  // ROS_INFO("waypoint size before time allocation: %lu", waypoint_vector.size());
  // computeTimeAllocation(waypoint_vector, max_speed, max_accel, initial_speed, time_vector);
  ROS_INFO("Size after allocation waypoints (%lu) and times (%lu)", waypoint_vector.size(), time_vector.size());
  // const int num_pieces = 6;
  // Eigen::Matrix<double, 3, num_pieces - 1> route;
  // route.col(0) << radius, length / 2, 0;
  // route.col(1) << 0, radius + length / 2, 0;
  // route.col(2) << -radius, length / 2, 0;
  // route.col(3) << -radius, -length / 2, 0;
  // route.col(4) << 0, -(radius + length / 2), 0;
  // Eigen::VectorXd times(num_pieces);
  // times << 4, 2, 2, 4, 2, 2;

  num_pieces = time_vector.size();
  times = Eigen::VectorXd(time_vector.size());
  route = Eigen::MatrixXd(3, waypoint_vector.size());
  // std::cout << "waypoints\n";
  // for (const auto& wp : waypoint_vector)
  // {
  //   std::cout << wp.transpose() << '\n';
  // }
  // std::cout << '\n';
  // for (const auto& t : time_vector)
  // {
  //   std::cout << t << '\t';
  // }
  // std::cout << '\n';
  for (size_t i = 0; i < time_vector.size(); ++i)
  {
    times(i) = time_vector[i];
  }
  for (size_t i = 0; i < waypoint_vector.size(); ++i)
  {
    // NOTE: waypoints includes start and end
    route.col(i) = waypoint_vector[i];
  }

  // std::cout << "iS: " << iS.col(0).transpose() << '\n';
  // for (int i = 0; i < route.cols(); ++i)
  // {
  //   std::cout << route.col(i).transpose() << '\n';
  // }
  // std::cout << "fS: " << fS.col(0).transpose() << '\n';

  try
  {
    alglib::real_1d_array x;
    x.setlength(num_pieces);
    for (int i = 0; i < num_pieces; ++i)
    {
      x[i] = time_vector[i];
    }

    double epsg = 0.0000000001;
    double epsf = 0;
    double epsx = 0;
    double diffstep = 1.0e-6;
    alglib::ae_int_t maxits = 0;
    alglib::minlbfgsstate state;
    alglib::minlbfgsreport rep;

    const auto tic = std::chrono::high_resolution_clock::now();
    alglib::minlbfgscreatef(5, x, diffstep, state);
    alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
    alglib::minlbfgsoptimize(state, function2);
    alglib::minlbfgsresults(state, x, rep);
    const auto toc = std::chrono::high_resolution_clock::now();

    printf("terminationType: %d\n", int(rep.terminationtype));  // EXPECTED: 4
    printf("iterationsCount: %d\n", int(rep.iterationscount));
    printf("optimized times: %s\n", x.tostring(2).c_str());  // EXPECTED: [-3,3]
    printf("Optimization duration: %li ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count());
  }
  catch (alglib::ap_error alglib_exception)
  {
    printf("ALGLIB exception with message '%s'\n", alglib_exception.msg.c_str());
    return 1;
  }

  std::cout << "Optim finished with:"
            << "\n\tduration: " << minJerkTraj.getTotalDuration() << "\n\tmax_vel: " << minJerkTraj.getMaxVelRate()
            << "\n\tmax_acc: " << minJerkTraj.getMaxAccRate() << '\n';
  // std::cout << "\tpositions:\n" << minJerkTraj.getPositions() << '\n';

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
