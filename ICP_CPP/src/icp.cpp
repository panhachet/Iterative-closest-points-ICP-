#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>
#include <tuple>
#include <limits> 

#include <Eigen/Dense>
#include "matplotlibcpp.h"


using namespace Eigen;
using namespace std;
namespace plt = matplotlibcpp;


tuple<vector<double>, vector<double>, vector<double>, vector<double>>
read_data(const string& filename)
{
    ifstream file(filename);
    vector<double> Qx, Qy, Px, Py;

    if (!file.is_open())
    {
        cerr << "Error: Could not open the file." << endl;
        return make_tuple(Qx, Qy, Px, Py);
    }
    string line;
    getline(file, line); //skip the header

    while(getline(file, line)) //Read each line of the file
    {
        stringstream ss(line);
        string value;
        double qx, qy, px, py;

        getline(ss, value, ','); qx = stod(value);
        getline(ss, value, ','); qy = stod(value);
        getline(ss, value, ','); px = stod(value);
        getline(ss, value, ','); py = stod(value);

        Qx.push_back(qx);
        Qy.push_back(qy);
        Px.push_back(px);
        Py.push_back(py);
    }
    file.close();
    return make_tuple(Qx, Qy, Px, Py);
}

vector<vector<double>> Data(const vector<double>& data_x, const vector<double>& data_y)
{
    vector<vector<double>> data;
    for(size_t i =  0; i<data_x.size(); i++)
    {
        data.push_back({data_x[i], data_y[i]});
    }
    return data;
}

vector<pair<int, int>> correspondence(const vector<vector<double>>& P, const vector<vector<double>>& Q)
{
    int p_size = P.size();
    int q_size = Q.size();
    vector<pair<int, int>> correspondences;
    for (int i = 0; i<p_size; i++)
    {
        vector<double> p_point  = {P[i][0], P[i][1]};
        double min_dist = numeric_limits<double>::infinity();
        int chosen_idx = -1;
        for (int j = 0; j<q_size; j++)
        {
            double sum = 0;
            vector<double> q_point  = {Q[j][0], Q[j][1]};
            for (size_t k = 0; k<p_point.size(); k++)
            {
                double diff = p_point[k] - q_point[k];
                sum += diff * diff;
            }
            double dist = sqrt(sum);
            if (dist < min_dist)
            {
                min_dist =  dist;
                chosen_idx =  j;
            }
        }
        correspondences.push_back({i, chosen_idx});
    }
    return correspondences;
}

MatrixXd rotation(const float& theta)
{
    MatrixXd rot(2,2);
    rot << cos(theta), -sin(theta),
            sin(theta), cos(theta);
    return rot;
}

MatrixXd d_rotation(const float& theta)
{
    MatrixXd d_rot(2,2);
    d_rot << -sin(theta), -cos(theta),
            cos(theta), -sin(theta);
    return d_rot;
}

MatrixXd jacobian(const vector<double>& x, const vector<double>& p)
{
    Vector2d p_point(p[0], p[1]);
    float theta = x[2];
    MatrixXd J = MatrixXd::Zero(2,3);
    J.block<2, 2>(0, 0) = Matrix2d::Identity();
    J.block<2, 1>(0, 2) = d_rotation(theta) * p_point;
    return J;
}

MatrixXd error(const vector<double>& x, const vector<double>& p, const vector<double>& q)
{
    Vector2d p_point(p[0], p[1]);
    Vector2d q_point(q[0], q[1]);
    MatrixXd R = rotation(x[2]);
    Vector2d t(x[0], x[1]);
    return (R * p_point + t - q_point);
}

tuple<MatrixXd, Vector3d> solver(const vector<double>& x, const vector<vector<double>>& P, const vector<vector<double>>& Q, vector<pair<int, int>>  correspondences)
{
    MatrixXd H = MatrixXd::Zero(3,3);
    Vector3d g = Vector3d::Zero(3);
    for (const auto& index : correspondences)
    {
        int i = index.first;
        int j = index.second;
        vector<double> p = P[i];
        vector<double> q = Q[j];
        MatrixXd e = error(x, p, q);
        MatrixXd J = jacobian(x, p);
        H += J.transpose() * J;
        g += J.transpose() * e;
    }
    return make_tuple(H, g);
}


tuple<vector<vector<double>>, vector<double>> icp(
    const vector<vector<double>>& P, 
    const vector<vector<double>>& Q, 
    vector<double> x_init, 
    int iteration, 
    double tolerance)
{
    vector<double> x = x_init;
    vector<vector<double>> P_prev = P;
    vector<vector<double>> x_new = {x_init};
    vector<vector<double>> P_new;
    vector<double> Dx;
    for (int i =0; i<iteration; i++)
    {
        MatrixXd R = rotation(x[2]);
        Vector2d t(x[0], x[1]);
        vector<pair<int, int>> corr = correspondence(P_prev,Q);
        auto [H, g] = solver(x, P, Q, corr);
        Vector3d dx = - H.inverse() * g;
        Dx.push_back(dx.norm());
        for (int i = 0; i < 3; i++) {
            x[i] += dx(i);
        }
        x[2] = atan2(sin(x[2]), cos(x[2])); 
        x_new.push_back(x);
        for (size_t i = 0; i < P.size(); i++)
        {
            Vector2d rotated_point = R * Vector2d(P[i][0], P[i][1]) + t;
            P_prev[i] = {rotated_point(0), rotated_point(1)};
        }
        P_new = P_prev;
        if(dx.norm() < tolerance)
        {
            break;
        }
    }
    return make_tuple(P_new, Dx);
}

void plot_data(const vector<double>& px, const vector<double>& py, 
const vector<double>& qx, const vector<double>& qy)
{

    plt::plot(px, py, "o:");
    plt::plot(qx, qy, "o:");
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    plt::text(px[0]-3,py[1],"P");
    plt::text(qx[0]-3,qy[1],"Q");
    plt::xlim(-10, 40);
    plt::ylim(-10, 40);
    plt::grid(true);
    plt::show();
}



int main()
{
    auto [Qx, Qy, Px, Py] = read_data("/home/robotic/ICP_CPP/src/coordinates.csv");
    vector<vector<double>> P = Data(Px, Py);
    vector<vector<double>> Q = Data(Qx, Qy);
    vector<double> X = {1, 1, 25 * M_PI/180};
    auto [P_new, Dx] = icp(P,Q,X,30,1E-6);
    for (const auto & data : P_new)
    {
        cout << data[0] << "," << data[1]  << endl;
    }
    vector<double> P_new_x, P_new_y;
    for (const auto& p : P_new) {
        P_new_x.push_back(p[0]);
        P_new_y.push_back(p[1]);
    }
    plot_data(P_new_x, P_new_y, Qx,Qy);
    return 0;

}