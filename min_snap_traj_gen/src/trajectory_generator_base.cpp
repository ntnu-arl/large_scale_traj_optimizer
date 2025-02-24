#include "min_snap_traj_gen/trajectory_generator_base.hpp"

// global variables
min_jerk::JerkOpt jerkOpt_;
min_jerk::Trajectory minJerkTraj_;
Eigen::Matrix3d iS_;
Eigen::Matrix3d fS_;
Eigen::VectorXd times_;
Eigen::MatrixXd waypoints_;
int num_pieces_;
// parameters
double rho_t_;
double rho_v_;
double rho_a_;
double vmax_;
double amax_;
double g(const double x)
{
  return std::pow(std::max(x, 0.0), 3);
}

void objectiveFunction(const alglib::real_1d_array& x, double& func, void* ptr)
{
  const double vmax2 = std::pow(vmax_, 2);
  const double amax2 = std::pow(amax_, 2);

  for (int i = 0; i < num_pieces_; ++i)
  {
    times_(i) = x[i];  // REVIEW: enforce positive times
  }

  Eigen::MatrixXd route(waypoints_.rows(), waypoints_.cols() - 2);
  for (int i = 1; i < waypoints_.cols() - 1; ++i)
  {
    route.col(i - 1) = waypoints_.col(i);
  }
  static bool first = true;
  if (first)
  {
    first = false;
    std::cout << "route/times/temp: " << waypoints_.cols() << '\t' << times_.size() << '\t' << route.cols() << '\n';
  }

  jerkOpt_.reset(iS_, fS_, num_pieces_);
  jerkOpt_.generate(route, times_);
  jerkOpt_.getTraj(minJerkTraj_);

  // calculate objective
  const double J_sigma = jerkOpt_.getObjective();
  double J_D_t = 0;
  for (int i = 0; i < num_pieces_; ++i)
  {
    J_D_t += times_(i);
  }
  double J_D_v = 0;
  double J_D_a = 0;
  for (int i = 0; i < num_pieces_ - 1; ++i)
  {
    // T_{i} corresponds to q_{i} to q_{i+1} instead of q_{i-1} to q_{i}
    const int j = i + 1;
    // get variables of interest (index according to fig 4 of https://arxiv.org/pdf/2011.02662)
    const Eigen::Vector3d q_im1 = waypoints_.col(j - 1);  // q_{i-1}
    const Eigen::Vector3d q_i = waypoints_.col(j);        // q_{i}
    const Eigen::Vector3d q_ip1 = waypoints_.col(j + 1);  // q_{i+1}
    const double T_i = times_(i);                         // T_{i}
    const double T_ip1 = times_(i + 1);                   // T_{i+1}

    J_D_v += g(((q_ip1 - q_im1) / (T_ip1 + T_i)).squaredNorm() - vmax2);
    J_D_a += g((((q_ip1 - q_i) / T_ip1 - (q_i - q_im1) / T_i) / ((T_ip1 + T_i) / 2.0)).squaredNorm() - amax2);
  }
  // std::cout << "sigma/t/v/a: " << J_sigma << '\t' << J_D_t << '\t' << J_D_v << '\t' << J_D_a << '\n';
  const double J_D = rho_t_ * J_D_t + rho_v_ * J_D_v + rho_a_ * J_D_a;

  func = J_sigma + J_D;
}

TrajectoryGeneratorBase::TrajectoryGeneratorBase(ros::NodeHandle& pnh)
{
  // TODO: load params
  pnh.param<double>("dt", dt_, 0.01);
  pnh.param<double>("optimization/rho_t", rho_t_, 25.0);
  pnh.param<double>("optimization/rho_v", rho_v_, 200.0);
  pnh.param<double>("optimization/rho_a", rho_a_, 1.0);
  pnh.param<double>("optimization/vmax", vmax_, 4.0);
  pnh.param<double>("optimization/amax", amax_, 4.0);

  pub_waypoints_ = pnh.advertise<nav_msgs::Path>("waypoints", 1, true);
  pub_path_ = pnh.advertise<nav_msgs::Path>("path", 1, true);
  // TODO: pub traj

  timer_publish_ = pnh.createTimer(ros::Duration(0.1), std::bind(&TrajectoryGeneratorBase::publishOnTimer, this));
}

void TrajectoryGeneratorBase::fillPose(const Eigen::Vector3d& vec, geometry_msgs::Pose& msg)
{
  msg.position.x = vec(0);
  msg.position.y = vec(1);
  msg.position.z = vec(2);
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
}

void TrajectoryGeneratorBase::fillPoseStamped(const std::string& frame_id, const ros::Time& stamp,
                                              const Eigen::Vector3d& vec, geometry_msgs::PoseStamped& msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  fillPose(vec, msg.pose);
}

// Assuming waypoint_vector_ includes start and end
void TrajectoryGeneratorBase::updateWaypoints()
{
  waypoint_vector_.push_back(Eigen::Vector3d(0, 0, 0));
  waypoint_vector_.push_back(Eigen::Vector3d(10, 0, 0));
  waypoint_vector_.push_back(Eigen::Vector3d(10, 10, 0));
  waypoint_vector_.push_back(Eigen::Vector3d(0, 10, 0));
  waypoint_vector_.push_back(Eigen::Vector3d(0, 0, 0));
}

// Assuming waypoint_vector_ includes start and end
void TrajectoryGeneratorBase::updateTimes()
{
  for (size_t i = 1; i < waypoint_vector_.size(); ++i)
  {
    time_vector_.push_back(1.0);
  }
}

void TrajectoryGeneratorBase::run()
{
  updateWaypoints();
  updateTimes();
  optimize(time_vector_, waypoint_vector_);
  updateMessages();
}

void TrajectoryGeneratorBase::updateMessages()
{
  geometry_msgs::PoseStamped ps;

  const Eigen::MatrixXd positions = minJerkTraj_.getPositions();
  wp_msg_.header.frame_id = "map";
  wp_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < positions.cols(); ++i)
  {
    fillPose(positions.col(i), ps.pose);
    wp_msg_.poses.push_back(ps);
  }

  Trajectory traj;
  path_msg_.header.frame_id = "map";
  path_msg_.header.stamp = ros::Time::now();
  double time = 0.0;
  const double duration = minJerkTraj_.getTotalDuration();
  while (time < duration)
  {
    const Eigen::Vector3d p = minJerkTraj_.getPos(time);
    const Eigen::Vector3d v = minJerkTraj_.getVel(time);
    const Eigen::Vector3d a = minJerkTraj_.getAcc(time);

    traj.add(time, p, v, a);
    fillPose(p, ps.pose);

    path_msg_.poses.push_back(ps);

    time += dt_;
  }

  writeFile(traj);
}

bool TrajectoryGeneratorBase::optimize(const std::vector<double>& time_vector,
                                       const std::vector<Eigen::Vector3d>& waypoint_vector)
{
  num_pieces_ = time_vector.size();
  times_ = Eigen::VectorXd(time_vector.size());
  waypoints_ = Eigen::MatrixXd(3, waypoint_vector.size());
  for (size_t i = 0; i < waypoint_vector.size(); ++i)
  {
    waypoints_.col(i) = waypoint_vector[i];
  }

  try
  {
    alglib::real_1d_array x;
    x.setlength(num_pieces_);
    for (int i = 0; i < num_pieces_; ++i)
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
    alglib::minlbfgsoptimize(state, objectiveFunction);
    alglib::minlbfgsresults(state, x, rep);
    const auto toc = std::chrono::high_resolution_clock::now();

    ROS_INFO("Optimization results:\n\tterminationType: %d\n\titerationsCount: %d\n\tduration: %li ms",
             int(rep.terminationtype), int(rep.iterationscount),
             std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count());
    ROS_INFO("Trajectory stats:\n\tduration: %f s\n\tmax_vel: %f m/s\n\tmax_acc: %f m/s^2",
             minJerkTraj_.getTotalDuration(), minJerkTraj_.getMaxVelRate(), minJerkTraj_.getMaxAccRate());

    return true;
  }
  catch (alglib::ap_error alglib_exception)
  {
    ROS_INFO("ALGLIB exception with message '%s'\n", alglib_exception.msg.c_str());
    return false;
  }
}

void TrajectoryGeneratorBase::publishOnTimer()
{
  pub_waypoints_.publish(wp_msg_);
  pub_path_.publish(path_msg_);

  // TODO: publish takeoff point
  // TODO: publish multidof trajectory
}

void TrajectoryGeneratorBase::writeFile(const Trajectory& traj, const std::string& file_name)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (file != NULL)
  {
    ROS_INFO("Writing to: %s", file_name.c_str());
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