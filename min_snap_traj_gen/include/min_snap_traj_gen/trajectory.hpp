#pragma once

#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>  // TODO: remove dependency

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