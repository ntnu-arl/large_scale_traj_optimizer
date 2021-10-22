#include "min_snap.hpp"
#include "min_snap_manager.hpp"
#include "random_route_gen.hpp"
#include "peacock_route_gen.hpp"
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap_gen");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    minSnapManager minSnapMan(nh);
    visUtils visUtil(&nh);

    // Number of random waypoints to generate
    int n_wp = 5;
    // Lenght of the cube in which to generate wps
    int l_side = 10; // [m]
    int fov_h = 90;  // [deg]
    int fov_v = 60;  // [deg]
    int rho_res = 2; // [m]
    int rho_min = 1; // [m]
    int rho_max = 5; // [m]
    double max_vel = 1.0;
    double max_acc = 1.0;

    pnh.getParam("n_wp", n_wp);
    pnh.getParam("l_side", l_side);
    pnh.getParam("fov_h", fov_h);
    pnh.getParam("fov_v", fov_v);
    pnh.getParam("rho_res", rho_res);
    pnh.getParam("rho_min", rho_min);
    pnh.getParam("rho_max", rho_max);
    pnh.getParam("max_vel", max_vel);
    pnh.getParam("max_acc", max_acc);

    // WP generation only inside a cube of dimension l_side
    randomRouteGenerator routeGen(Array3d(-l_side, -l_side, -l_side), Array3d(l_side, l_side, l_side));
    peacockRouteGenerator peacockRouteGen;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    vector<VectorXd> v_ts;
    Matrix3d iS, fS;

    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate r(0.4);
    // int groupSize = 100;
    vector<min_snap::Trajectory> primitives;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0;

    while (ros::ok())
    {
        // Timer initialization
        d0 = 0.0;
        // Random waypoints generation
        // route = routeGen.generate(n_wp);
        int route_split = (rho_max - rho_min) / rho_res + 2;
        cout << "route_split: " << route_split << endl;

        route = peacockRouteGen.generate(fov_h, fov_v, rho_res, rho_min, rho_max);
        cout << "Total wp: " << route.cols() - 1 << endl;
        cout << route << endl;

        for (int i = 0; i < route.cols(); i += route_split)
        {

            cout << "i: " << i << endl;
            // cout << route.col(i) << endl;
            // cout << "route.col(i+route_split): " << endl;
            // cout << route.col(route_split) << endl;

            // initialState and finalState definition
            iS.col(0) << route.col(i);
            fS.col(0) << route.col(i + route_split - 1);
            // cout << "iS: " << endl;
            // cout << iS << endl;
            // cout << "fS: " << endl;
            // cout << fS << endl;
            // fS.col(0) << route.rightCols<1>();

            // route, vel, acc
            ts = minSnapMan.allocateTime(route.block(0, i, 3, route_split), max_vel, max_acc);
            v_ts.push_back(ts);
            iSS << iS, Eigen::MatrixXd::Zero(3, 1);
            fSS << fS, Eigen::MatrixXd::Zero(3, 1);

            tc1 = std::chrono::high_resolution_clock::now();
            // snapOpt.reset(iSS, fSS, route_split - 1);
            snapOpt.reset(iSS, fSS, route_split - 1);

            // snapOpt.generate(route.block(0, 1, 3, route_split - 2), ts);
            snapOpt.generate(route.block(0, i+1, 3, route_split - 2), ts);

            snapOpt.getTraj(minSnapTraj);
            primitives.push_back(minSnapTraj);

            tc2 = std::chrono::high_resolution_clock::now();
            d0 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
            ROS_WARN("Built in %f ms", d0 * 1e3);

            // visUtil.visRawTraj(minSnapTraj.getPositions());
            // visUtil.visSnapTraj(minSnapTraj, ts);
        }
        cout << "primitives.size(): " << primitives.size() << endl;

        visUtil.visWaypoints(route);

        for (int i = 0; i < primitives.size(); i++)
        {
            visUtil.visRawTraj(primitives[i].getPositions());
            visUtil.visSnapTraj(primitives[i], v_ts[i]);
        }

        primitives.clear();
        v_ts.clear();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
