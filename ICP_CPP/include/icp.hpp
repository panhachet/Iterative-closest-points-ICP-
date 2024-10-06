#ifndef ICP_HPP
#define ICP_HPP

#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>
#include <tuple>
#include <limits> 
#include <chrono> 

#include <Eigen/Dense>
#include "matplotlibcpp.h"

using namespace Eigen;
using namespace std;
using namespace std::chrono;
namespace plt = matplotlibcpp;

class icp
{
    private:
        vector<vector<double>> P;
        vector<vector<double>> Q;
        vector<double> X;
        int iter;
        double tol;

    public:
        icp(const vector<vector<double>>& P_data, 
            const vector<vector<double>>& Q_data, 
            vector<double> x_init, 
            int iteration, 
            double tolerance);
        static vector<vector<double>> combine_data(const vector<double>& data_x, 
            const vector<double>& data_y);
        vector<pair<int, int>> correspondence(const vector<vector<double>>& P, 
            const vector<vector<double>>& Q);
        MatrixXd rotation(const float& theta) const;
        MatrixXd d_rotation(const float& theta) const;
        MatrixXd jacobian(const vector<double>& x, 
            const vector<double>& p) const;
        MatrixXd error(const vector<double>& x, 
            const vector<double>& p, 
            const vector<double>& q) const;
        tuple<MatrixXd, Vector3d> solver(const vector<double>& x, 
            const vector<vector<double>>& P, 
            const vector<vector<double>>& Q, 
            vector<pair<int, int>>  correspondences);
        tuple<vector<vector<double>>, vector<double>> icp_function();  
};

#endif