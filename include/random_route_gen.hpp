#ifndef RANDOM_ROUTE_GEN_HPP
#define RANDOM_ROUTE_GEN_HPP

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <set>
#include <vector>
#include <random>
#include <Eigen/Eigen>

using namespace Eigen;

class randomRouteGenerator
{
public:
    randomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline MatrixXd generate(int N)
    {
        MatrixXd route(3, N + 1);
        Array3d temp;
        route.col(0).setZero();
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.col(i + 1) << temp;
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

#endif