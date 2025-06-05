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
    times_(i) = std::abs(x[i]);  // REVIEW: enforce positive times
  }

  Eigen::MatrixXd route(waypoints_.rows(), waypoints_.cols() - 2);
  for (int i = 1; i < waypoints_.cols() - 1; ++i)
  {
    route.col(i - 1) = waypoints_.col(i);
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

  // ROS_INFO("num_pieces: %i\twaypoints.size: %i\t times.size: %i", num_pieces_, waypoints_.cols(), times_.size());
  for (int i = 0; i < num_pieces_ - 1; ++i)
  // i => 0 -> 17 (< 19-1)
  // j = i+1
  // j => 1 -> 18
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

  const double J_D = rho_t_ * J_D_t + rho_v_ * J_D_v + rho_a_ * J_D_a;

  // ROS_INFO("J_sigma: %f\tJ_D_t: %f\tJ_D_v: %f\tJ_D_a: %f", J_sigma, J_D_t, J_D_v, J_D_a);

  func = J_sigma + J_D;
}

TrajectoryGeneratorBase::TrajectoryGeneratorBase(ros::NodeHandle& pnh)
{
  // parameters
  pnh.param<double>("traj_dt", dt_, 0.05);
  pnh.param<double>("optimization/rho_t", rho_t_, 25.0);
  pnh.param<double>("optimization/rho_v", rho_v_, 200.0);
  pnh.param<double>("optimization/rho_a", rho_a_, 1.0);
  pnh.param<double>("optimization/vmax", vmax_, 4.0);
  pnh.param<double>("optimization/amax", amax_, 4.0);
  pnh.param<int>("optimization/max_iter", max_iter_, 500);
  pnh.param<int>("optimization/M", M_, 5);
  std::vector<double> d_param;
  pnh.param<std::vector<double>>("offset", d_param, { 0, 0, 1.5 });
  offset_ << d_param[0], d_param[1], d_param[2];
  pnh.param<std::string>("frame_id", frame_id_, "map");
  pnh.param<double>("yaw_deg", yaw_, 0.0);
  yaw_ = yaw_ * M_PI / 180.0;
  pnh.param<bool>("align_yaw", align_yaw_, true);
  pnh.param<bool>("add_yaw_rate", add_yaw_rate_, false);
  pnh.param<bool>("rotate_xy", rotate_xy_, false);
  pnh.param<bool>("start_with_pose", start_with_pose_, false);
  pnh.param<std::string>("pose_topic", pose_topic_, "/mavros/local_position/pose");

  // getting start position
  if (start_with_pose_)
  {  // REVIEW: maybe do in constructor
    ROS_INFO("Waiting for pose on %s", pose_topic_.c_str());
    geometry_msgs::PoseStamped msg;
    msg = *ros::topic::waitForMessage<geometry_msgs::PoseStamped>(pose_topic_, pnh_);

    waypoint_vector_.emplace_back(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  }

  // publishers
  pub_waypoints_ = pnh.advertise<nav_msgs::Path>("waypoints", 1, true);
  pub_path_ = pnh.advertise<nav_msgs::Path>("path", 1, true);
  pub_trajectory_ = pnh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1, false);

  // services
  srv_optimize_ = pnh.advertiseService("optimize", &TrajectoryGeneratorBase::optimizeService, this);
  srv_takeoff_ = pnh.advertiseService("takeoff", &TrajectoryGeneratorBase::takeoffService, this);
  srv_start_ = pnh.advertiseService("start", &TrajectoryGeneratorBase::startService, this);

  // main publish timer
  timer_publish_ = pnh.createTimer(ros::Duration(0.1), std::bind(&TrajectoryGeneratorBase::publishOnTimer, this));
}

void TrajectoryGeneratorBase::fillPose(const Eigen::Vector3d& pos, geometry_msgs::Pose& msg)
{
  msg.position.x = pos(0);
  msg.position.y = pos(1);
  msg.position.z = pos(2);
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
}

void TrajectoryGeneratorBase::fillPose(const Eigen::Vector3d& pos, const double yaw, geometry_msgs::Pose& msg)
{
  msg.position.x = pos(0);
  msg.position.y = pos(1);
  msg.position.z = pos(2);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.orientation = tf2::toMsg(q);
}

void TrajectoryGeneratorBase::fillMultiDOFTrajectoryPoint(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                                          const Eigen::Vector3d& acc, const double yaw,
                                                          const double yaw_rate, const double time,
                                                          trajectory_msgs::MultiDOFJointTrajectoryPoint& point)
{
  geometry_msgs::Transform tf;
  tf.translation.x = pos(0);
  tf.translation.y = pos(1);
  tf.translation.z = pos(2);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf.rotation = tf2::toMsg(q);

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = vel(0);
  vel_msg.linear.y = vel(1);
  vel_msg.linear.z = vel(2);
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = std::isnan(yaw_rate) ? 0.0 : -yaw_rate;

  geometry_msgs::Twist acc_msg;
  acc_msg.linear.x = acc(0);
  acc_msg.linear.y = acc(1);
  acc_msg.linear.z = acc(2);
  acc_msg.angular.x = 0.0;
  acc_msg.angular.y = 0.0;
  acc_msg.angular.z = 0.0;

  point.transforms = { tf };
  point.velocities = { vel_msg };
  point.accelerations = { acc_msg };
  point.time_from_start = ros::Duration(time);
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

void TrajectoryGeneratorBase::updateStartFinish()
{
  // set start and end position/velocity/acceleration
  iS_.setZero();
  fS_.setZero();
  iS_.col(0) = waypoint_vector_.front();
  fS_.col(0) = waypoint_vector_.back();
}

// Assuming waypoint_vector_ includes start and end
void TrajectoryGeneratorBase::updateTimes()
{
  for (size_t i = 1; i < waypoint_vector_.size(); ++i)
  {
    time_vector_.push_back(1.0);
  }
}

void TrajectoryGeneratorBase::rotateWaypoints()
{
  // start
  {
    const Eigen::VectorXd row_0 = iS_.row(0);
    const Eigen::VectorXd row_1 = iS_.row(1);
    iS_.row(0) = row_1;
    iS_.row(1) = row_0;
  }

  // finish
  {
    const Eigen::VectorXd row_0 = fS_.row(0);
    const Eigen::VectorXd row_1 = fS_.row(1);
    fS_.row(0) = row_1;
    fS_.row(1) = row_0;
  }

  // all waypoints
  for (Eigen::Vector3d& wp : waypoint_vector_)
  {
    // swap
    const double temp = wp.x();
    wp.x() = wp.y();
    wp.y() = temp;
  }
}

double TrajectoryGeneratorBase::interpolateHeight(const double time, const double duration, const double height_gain)
{
  return height_gain * time / duration;
}

void TrajectoryGeneratorBase::run()
{
  waypoint_vector_.clear();
  time_vector_.clear();

  updateWaypoints();
  if (waypoint_vector_.empty())
  {
    ROS_ERROR("waypoint_vector_ is empty");
    return;
  }
  updateStartFinish();
  if (rotate_xy_)
  {
    rotateWaypoints();
  }
  updateTimes();
  optimize();
  updateMessages();
}

void TrajectoryGeneratorBase::updateMessages()
{
  std_msgs::Header header;
  header.frame_id = frame_id_;
  header.stamp = ros::Time::now();  // TODO: not synchronized? Does it matter?

  geometry_msgs::PoseStamped ps;
  ps.header = header;

  const Eigen::MatrixXd positions = minJerkTraj_.getPositions();

  wp_msg_.header = header;
  wp_msg_.poses.resize(positions.cols());
  for (int i = 0; i < positions.cols(); ++i)
  {
    fillPose(positions.col(i), ps.pose);
    wp_msg_.poses[i] = ps;
  }

  Trajectory traj;
  path_msg_.header = header;
  traj_msg_.header = header;
  traj_msg_.joint_names.push_back("joint");

  trajectory_msgs::MultiDOFJointTrajectoryPoint tp;

  double time = 0.0;
  const double duration = minJerkTraj_.getTotalDuration();
  const int approx_size = int(duration / dt_);

  traj.reserve(approx_size);
  path_msg_.poses.clear();
  path_msg_.poses.reserve(approx_size);
  traj_msg_.points.clear();
  traj_msg_.points.reserve(approx_size);
  double max_yaw_rate = 0.0;
  while (time < duration)
  {
    const Eigen::Vector3d p = minJerkTraj_.getPos(time);
    const Eigen::Vector3d v = minJerkTraj_.getVel(time);
    const Eigen::Vector3d a = minJerkTraj_.getAcc(time);

    const double vx = v(0);
    const double vy = v(1);
    const double ax = a(0);
    const double ay = a(1);
    double yaw = yaw_;
    double yaw_rate = 0.0;
    if (align_yaw_)
    {
      yaw = std::atan2(vy, vx);
      yaw_rate = add_yaw_rate_ ? (vx * ay - vy * ax) / (vx * vx + vy * vy) : 0.0;
      max_yaw_rate = std::max(max_yaw_rate, std::abs(yaw_rate));
    }

    traj.add(time, p, v, a, yaw, yaw_rate);

    fillPose(p, yaw, ps.pose);
    path_msg_.poses.push_back(ps);

    // REVIEW: time or time+dt
    fillMultiDOFTrajectoryPoint(p, v, a, yaw, yaw_rate, dt_, tp);
    traj_msg_.points.push_back(tp);

    time += dt_;
  }

  if (align_yaw_)
  {
    std::cout << "\tmax yaw rate: " << max_yaw_rate * 180.0 / M_PI << " deg" << '\n';
    // set first yaw equal to second, so takeoff matches
    double yaw = traj.getYaw(1);
    traj.setYaw(0, yaw);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    path_msg_.poses[0].pose.orientation = tf2::toMsg(q);
    traj_msg_.points[0].transforms[0].rotation = tf2::toMsg(q);
  }

  writeFile(traj);
}

bool TrajectoryGeneratorBase::optimize()
{
  num_pieces_ = time_vector_.size();
  times_ = Eigen::VectorXd(time_vector_.size());
  waypoints_ = Eigen::MatrixXd(3, waypoint_vector_.size());

  // add offset
  // TODO: separate function?
  if (!start_with_pose_)
  {
    iS_.col(0) += offset_;
  }
  fS_.col(0) += offset_;
  for (size_t i = 0; i < waypoint_vector_.size(); ++i)
  {
    waypoints_.col(i) = waypoint_vector_[i] + offset_;
  }

  try
  {
    alglib::real_1d_array x;
    x.setlength(num_pieces_);
    for (int i = 0; i < num_pieces_; ++i)
    {
      x[i] = time_vector_[i];
    }

    ROS_INFO("Optimizing with %li waypoints", waypoint_vector_.size());
    std::cout << "iS_: \n" << iS_ << "\nfS_:\n" << fS_ << '\n';

    if (M_ >= (int)waypoint_vector_.size())
    {
      M_ = (int)waypoint_vector_.size() - 1;
    }

    // TODO: set better stopping criteria
    double epsg = 0.0000000001;
    double epsf = 0;
    double epsx = 0;
    double diffstep = 1.0e-6;
    alglib::ae_int_t maxits = (alglib::ae_int_t)max_iter_;
    alglib::minlbfgsstate state;
    alglib::minlbfgsreport rep;

    const auto tic = std::chrono::high_resolution_clock::now();
    alglib::minlbfgscreatef((alglib::ae_int_t)M_, x, diffstep, state);
    alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
    alglib::minlbfgsoptimize(state, objectiveFunction);
    alglib::minlbfgsresults(state, x, rep);
    const auto toc = std::chrono::high_resolution_clock::now();

    // taken from: https://www.alglib.net/translator/man/manual.cpp.html#sub_mincgresults
    std::string termination_string;
    switch (rep.terminationtype)
    {
      case -8:
        termination_string = "internal integrity control detected infinite or NAN values in function/gradient";
        break;
      case 1:
        termination_string = "relative function improvement is no more than EpsF";
        break;
      case 2:
        termination_string = "relative step is no more than EpsX";
        break;
      case 4:
        termination_string = "gradient norm is no more than EpsG";
        break;
      case 5:
        termination_string = "MaxIts steps was taken";
        break;
      case 7:
        termination_string = "stopping conditions are too stringent";
        break;
      case 8:
        termination_string = "terminated by user who called minlbfgsrequesttermination()";
        break;
      default:
        termination_string = "invalid termination type";
        break;
    }

    ROS_INFO("Optimization results:\n\tterminationType: %d (%s)\n\titerationsCount: %d\n\tduration: %li ms",
             int(rep.terminationtype), termination_string.c_str(), int(rep.iterationscount),
             std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count());
    ROS_INFO("Trajectory stats:\n\tduration: %f s\n\tmax_vel: %f m/s\n\tmax_acc: %f m/s^2",
             minJerkTraj_.getTotalDuration(), minJerkTraj_.getMaxVelRate(), minJerkTraj_.getMaxAccRate());

    // print max yaw rate

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
  if (wp_msg_.poses.empty() || path_msg_.poses.empty())
  {
    return;
  }

  pub_waypoints_.publish(wp_msg_);
  pub_path_.publish(path_msg_);

  if (!takeoff_msg_.points.empty())
  {
    pub_trajectory_.publish(takeoff_msg_);
  }
}

bool TrajectoryGeneratorBase::optimizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Optimize service");
  run();  // TODO: expand

  return true;
}

bool TrajectoryGeneratorBase::takeoffService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Takeoff service");

  if (traj_msg_.points.empty())
  {
    ROS_ERROR("Service called while trajectory has %li points", traj_msg_.points.size());
    return false;
  }

  takeoff_msg_.header = traj_msg_.header;
  takeoff_msg_.joint_names = traj_msg_.joint_names;
  takeoff_msg_.points = { traj_msg_.points[0] };

  return true;
}

bool TrajectoryGeneratorBase::startService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Start service");

  if (takeoff_msg_.points.empty())
  {
    ROS_ERROR("Service called pre-takeoff");
    return false;
  }

  takeoff_msg_.points.clear();
  ROS_INFO("Publishing %li point trajectory", traj_msg_.points.size());
  pub_trajectory_.publish(traj_msg_);

  return true;
}

void TrajectoryGeneratorBase::writeFile(const Trajectory& traj, const std::string& file_name)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (file != NULL)
  {
    ROS_INFO("Writing to: %s", file_name.c_str());
    fprintf(file, "time,px,py,pz,vx,vy,vz,ax,ay,az,yaw,yaw_rate\n");
    for (size_t i = 0; i < traj.size(); ++i)
    {
      const double time = traj.getTime(i);
      const Eigen::Vector3d pos = traj.getPos(i);
      const Eigen::Vector3d vel = traj.getVel(i);
      const Eigen::Vector3d acc = traj.getAcc(i);
      const double yaw = traj.getYaw(i);
      const double yaw_rate = traj.getYawRate(i);

      fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", time, pos(0), pos(1), pos(2), vel(0), vel(1), vel(2),
              acc(0), acc(1), acc(2), yaw, yaw_rate);
    }
  }
  fclose(file);
  ROS_INFO("Done writing");
}