#include "matplotlibcpp.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
namespace plt = matplotlibcpp;

int main()
{
    Matrix2d A;
    A << 1, 2,
        3, 4;
    cout << "A: \n " << A << endl;
    plt::plot({1,2,3,4});
    plt::show();
    return 0;
}