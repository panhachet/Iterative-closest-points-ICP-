#include "icp.hpp"
#include <unordered_map>
#include <limits>
#include <cmath>
#include "KDTree.hpp"
#include <nanoflann.hpp>

icp::icp(const vector<vector<double>>& P_data, 
            const vector<vector<double>>& Q_data, 
            vector<double> x_init, 
            int iteration, 
            double tolerance)
            : P(P_data), Q(Q_data), X(x_init), iter(iteration), tol(tolerance)
            {
                cout << "===================ICP Slam====================" << endl;
                cout << "Intergration project Master Mars"<< endl;
                cout << "Implementation with ROS 2" << endl;
                cout << "===============================================" << endl;


            }

vector<vector<double>> icp::combine_data(const vector<double>& data_x, const vector<double>& data_y)
{
    vector<vector<double>> data;
    for(size_t i =  0; i<data_x.size(); i++)
    {
        data.push_back({data_x[i], data_y[i]});
    }
    return data;
}

vector<pair<int, int>> icp::correspondence(const vector<vector<double>>& P, const vector<vector<double>>& Q)
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


vector<pair<int, int>> icp::correspondence1(
    const vector<vector<double>>& P,
    const vector<vector<double>>& Q
) {
    int p_size = P.size();
    vector<pair<int, int>> correspondences;
    PointCloudAdaptor2D adaptor(Q);
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor2D>,
        PointCloudAdaptor2D,
        2 
    >;

    KDTree kdtree(2, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree.buildIndex();
    for (int i = 0; i < p_size; ++i) {
        size_t closest_idx;
        double min_dist_sq;
        nanoflann::KNNResultSet<double> resultSet(1); // Nearest neighbor
        resultSet.init(&closest_idx, &min_dist_sq);

        kdtree.findNeighbors(resultSet, P[i].data(), nanoflann::SearchParameters(10));

        correspondences.push_back({i, static_cast<int>(closest_idx)});
    }

    return correspondences;
}




MatrixXd icp::rotation(const float& theta) const
{
    MatrixXd rot(2,2);
    rot << cos(theta), -sin(theta),
            sin(theta), cos(theta);
    return rot;
}

MatrixXd icp::d_rotation(const float& theta) const
{
    MatrixXd d_rot(2,2);
    d_rot << -sin(theta), -cos(theta),
            cos(theta), -sin(theta);
    return d_rot;
}

MatrixXd icp::jacobian(const vector<double>& x, const vector<double>& p) const
{
    Vector2d p_point(p[0], p[1]);
    float theta = x[2];
    MatrixXd J = MatrixXd::Zero(2,3);
    J.block<2, 2>(0, 0) = Matrix2d::Identity();
    J.block<2, 1>(0, 2) = d_rotation(theta) * p_point;
    return J;
}

MatrixXd icp::error(const vector<double>& x, const vector<double>& p, const vector<double>& q) const
{
    Vector2d p_point(p[0], p[1]);
    Vector2d q_point(q[0], q[1]);
    MatrixXd R = rotation(x[2]);
    Vector2d t(x[0], x[1]);
    return (R * p_point + t - q_point);
}

tuple<MatrixXd, Vector3d> icp::solver(const vector<double>& x, const vector<vector<double>>& P, 
const vector<vector<double>>& Q, vector<pair<int, int>>  correspondences)
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

void icp::set_P(const vector<vector<double>>& new_P) {
    P = new_P;
}

void icp::set_Q(const vector<vector<double>>& new_Q) {
    Q = new_Q;
}
void icp::set_X(const vector<double>& new_x)
{
    X = new_x;
}

tuple<vector<vector<double>>, vector<double>, bool> icp::icp_function()
{
    vector<double> x = X;
    vector<vector<double>> P_prev = P;
    vector<vector<double>> x_new = {X};
    vector<vector<double>> P_new;
    vector<vector<double>> P_align;
    vector<double> Dx;
    vector<vector<double>> P_rel;
    for (int i =0; i<iter; i++)
    {
        MatrixXd R = rotation(x[2]);
        Vector2d t(x[0], x[1]);
        vector<pair<int, int>> corr = correspondence1(P_prev,Q);
        auto [H, g] = solver(x, P, Q, corr);
        Vector3d dx = - H.inverse() * g;
        Dx.push_back(dx.norm());
        for (int i = 0; i < 3; i++) {
            x[i] += dx(i);
        }
        x[2] = atan(sin(x[2])/cos(x[2])); 
        x_new.push_back(x);
        for (size_t i = 0; i < P.size(); i++)
        {
            Vector2d rotated_point = R* Vector2d(P[i][0], P[i][1]) + t;
            P_prev[i] = {rotated_point(0), rotated_point(1)};
        }
        P_new = P_prev;
        if(dx.norm() < tol)
        {
            
            cout << "================Convergence================" << endl;
            cout << "Iteration:" << i << endl;
            cout << "Error X\t" << dx[0] <<endl;
            cout << "Error Y\t" << dx[1] <<endl;
            cout << "Error Yaw\t" << dx[2] << endl;
            cout << "====================end=====================" << endl;

            return make_tuple(P_new, Dx, true);
            break;
        }
    }    
    return make_tuple(P_new, Dx, false);
}
