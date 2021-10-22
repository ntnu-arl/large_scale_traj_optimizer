#ifndef PEACOCK_ROUTE_GEN_HPP
#define PEACOCK_ROUTE_GEN_HPP

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <set>
#include <vector>
#include <random>
#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

class peacockRouteGenerator
{
public:
    peacockRouteGenerator(){};

    inline double deg2rad(double deg)
    {
        return deg * M_PI / 180;
    }

    inline MatrixXd generate(int fov_h, int fov_v, int rho_res, int rho_min, int rho_max)
    {
        // MatrixXd route(3, N + 1);
        MatrixXd route;
        Array3d temp;
        double v_theta[2] = {deg2rad(-fov_h / 2), deg2rad(fov_h / 2)};
        double v_pitch[2] = {deg2rad(-fov_v / 2), deg2rad(fov_v / 2)};
        double ang_res = deg2rad(10);
        vector<Array3d> coords;

        for (double theta = v_theta[0]; theta <= v_theta[1]; theta += ang_res) // 10deg
        {
            for (double phi = v_pitch[0]; phi <= v_pitch[1]; phi += ang_res)
            {
                coords.push_back(Array3d(0, 0, 0));
                for (int r = rho_min; r <= rho_max; r += rho_res)
                {
                    temp << r * cos(theta),
                        r * sin(theta),
                        r * sin(phi);

                    coords.push_back(temp);
                }
            }
        }

        route.resize(3, coords.size() + 1);
        route.col(0).setZero();

        for (int i = 0; i < coords.size(); i++)
            route.col(i + 1) << coords[i];

        coords.clear();
        removeColumn(route, 0);

        return route;
    }
    void removeColumn(Eigen::MatrixXd &matrix, unsigned int colToRemove)
    {
        unsigned int numRows = matrix.rows();
        unsigned int numCols = matrix.cols() - 1;

        if (colToRemove < numCols)
            matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.rightCols(numCols - colToRemove);

        matrix.conservativeResize(numRows, numCols);
    }

private:
};

#endif