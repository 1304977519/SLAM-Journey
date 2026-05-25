
#include <Eigen/Core>
#include <iostream>
using namespace Eigen;
int main() {
    Matrix<double, 69, 90> matrix = Matrix<double, 69, 90>::Random();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i != j) matrix(i, j) = 0;
            else matrix(i, j) = 1;
        }
    }
    std::cout << matrix << std::endl;

    //AI写法,从第 0 行、第 0 列开始，取一个 3 × 3 的块。
    matrix.block<3, 3>(0, 0) = Matrix3d::Identity();
    return 0;
}
