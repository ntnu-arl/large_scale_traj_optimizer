#include "traj_min_snap.hpp"
#include "random_route_gen.hpp"
#include "vis_utils.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

using namespace std;
using namespace ros;
using namespace Eigen;

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }
    // cout << "Time allocation vector: " << durations << endl;
    return durations;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap");
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_("~");

    VisUtils vis_utils(&nh_);

    int l_side = 5;
    // WP generation only inside a cube of dimension l_side
    RandomRouteGenerator routeGen(Array3d(-l_side, -l_side, -l_side), Array3d(l_side, l_side, l_side));

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;

    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate r(0.4);
    // int groupSize = 100;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0;

    while (ros::ok())
    {
        // Number of random waypoints to generate
        int n_wp = 2;
        // Timer initialization
        d0 = 0.0;
        // Random waypoints generation
        route = routeGen.generate(n_wp);
        // initialState and finalState definition
        iS.col(0) << route.leftCols<1>();
        fS.col(0) << route.rightCols<1>();

        // route, vel, acc
        ts = allocateTime(route, 3.0, 3.0);

        iSS << iS, Eigen::MatrixXd::Zero(3, 1);
        fSS << fS, Eigen::MatrixXd::Zero(3, 1);

        tc1 = std::chrono::high_resolution_clock::now();
        snapOpt.reset(iSS, fSS, route.cols() - 1);
        snapOpt.generate(route.block(0, 1, 3, n_wp - 1), ts);
        snapOpt.getTraj(minSnapTraj);
        tc2 = std::chrono::high_resolution_clock::now();

        d0 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
        ROS_WARN("Took %f s to build the minSnapTraj", d0);

        vis_utils.vis_waypoints(route);
        vis_utils.vis_raw_traj(minSnapTraj.getPositions());
        vis_utils.vis_traj(minSnapTraj, ts);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
