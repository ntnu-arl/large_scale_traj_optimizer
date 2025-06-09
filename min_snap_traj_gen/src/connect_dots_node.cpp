#include "min_snap_traj_gen/connect_dots_node.hpp"

Eigen::Vector3d poseToVector(const geometry_msgs::Pose& pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

ConnectDots::ConnectDots(ros::NodeHandle& pnh) : TrajectoryGeneratorBase(pnh)
{
  pub_markers_ = pnh.advertise<geometry_msgs::PoseArray>("markers", 10);
  sub_marker_ =
      pnh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &ConnectDots::markerCallback, this);

  // for debugging
  geometry_msgs::Pose pose;
  pose.position.x = 23;
  pose.position.y = 4.5;
  pose_array_.poses.push_back(pose);
  pose.position.x = -16;
  pose.position.y = -1;
  pose_array_.poses.push_back(pose);
  pose_array_.header.frame_id = "map";
  pose_array_.header.stamp = ros::Time::now();
  pub_markers_.publish(pose_array_);
}

void ConnectDots::markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_array_.header = msg->header;
  pose_array_.poses.push_back(msg->pose);

  pub_markers_.publish(pose_array_);
}

void ConnectDots::updateWaypoints()
{
  if (pose_array_.poses.size() == 2)
  {
    const Eigen::Vector3d p1 = poseToVector(pose_array_.poses.front());
    const Eigen::Vector3d p2 = poseToVector(pose_array_.poses.back());

    // add interpolated points
    const int N = 20;
    for (int i = 0; i <= N - 1; i++)
    {
      const double dt = (double)i / (N - 1);
      waypoint_vector_.push_back(p1 + (p2 - p1) * dt);
    }
  }
  else
  {
    for (const geometry_msgs::Pose p : pose_array_.poses)
    {
      waypoint_vector_.emplace_back(p.position.x, p.position.y, p.position.z);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_snap_traj_node");
  ros::NodeHandle pnh("~");

  ConnectDots node(pnh);
  // // for debugging
  node.run();

  ros::spin();

  return 0;
}
