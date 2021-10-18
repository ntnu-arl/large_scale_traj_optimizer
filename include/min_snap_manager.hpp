#ifndef MIN_SNAP_MANAGER_HPP
#define MIN_SNAP_MANAGER_HPP

#include <cfloat>
#include <set>
#include <vector>
#include <random>
#include <Eigen/Eigen>

using namespace Eigen;

class MinSnapManager
{
public:
    MinSnapManager(ros::NodeHandle &n){};

    VectorXd allocateTime(const MatrixXd &wayPs, double vel, double acc)
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

private:
    int n_wp;
};

#endif