#include "icp.hpp"

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
    vector<vector<double>> P = icp::combine_data(Px, Py);
    vector<vector<double>> Q = icp::combine_data(Qx, Qy);
    vector<double> X = {1, 1, 25 * M_PI/180};
    auto start_time = high_resolution_clock::now();
    icp icp(P, Q, X, 100, 1E-6);
    auto [P_new, Dx] = icp.icp_function();
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - start_time);
    cout << "ICP computation time: " << duration.count() << " milliseconds" << endl;
    vector<double> P_new_x, P_new_y;
    for (const auto& p : P_new) {
        P_new_x.push_back(p[0]);
        P_new_y.push_back(p[1]);
    }
    plot_data(P_new_x, P_new_y, Qx,Qy);
    return 0;
}