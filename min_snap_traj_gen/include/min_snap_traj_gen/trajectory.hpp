#pragma once

#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>  // TODO: remove dependency on ros

class Trajectory
{
public:
  Trajectory() = default;
  ~Trajectory() = default;

  void reserve(const int N)
  {
    time_.reserve(N);
    pos_.reserve(N);
    vel_.reserve(N);
    acc_.reserve(N);
    yaw_.reserve(N);
    yaw_rate_.reserve(N);
  }

  void add(const double time, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
           const double yaw, const double yaw_rate)
  {
    time_.push_back(time);
    pos_.push_back(pos);
    vel_.push_back(vel);
    acc_.push_back(acc);
    yaw_.push_back(yaw);
    yaw_rate_.push_back(yaw_rate);
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

  double getYaw(const size_t i) const
  {
    return yaw_[i];
  }

  double getYawRate(const size_t i) const
  {
    return yaw_rate_[i];
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
  std::vector<double> yaw_;
  std::vector<double> yaw_rate_;
};