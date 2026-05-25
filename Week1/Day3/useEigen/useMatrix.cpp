
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>
using namespace Eigen;
using namespace std;
#define MAX_SIZE 50
int main() {
    Matrix<float, 2, 3> matrix_23;
    Matrix<float, 3, 3> matrix_33 = Matrix3f::Zero();
    Matrix<float, Dynamic, Dynamic> matrix_dynamic(1, 3);
    matrix_dynamic << 3, 3, 3;
    cout << matrix_dynamic << endl;
    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;
    matrix_23 << 2, 3, 4,
                 5, 3, 8;
    v_3d << 2, 3, 4;
    Matrix<double, 2, 1> result_matrix = matrix_23.cast<double>() * v_3d;
    cout << result_matrix << endl;
    matrix_33 = Matrix3f::Random();
    cout << matrix_33 << endl << endl;
    cout << matrix_33.transpose() << endl << endl;
    cout << 19 * matrix_33 << endl << endl;
    cout << matrix_33.trace() << endl << endl;
    cout << matrix_33.determinant() << endl << endl;
    cout << matrix_33.inverse() << endl << endl;
    cout << matrix_33.sum() << endl << endl << endl;

    SelfAdjointEigenSolver<Matrix3f> solver(matrix_33 * matrix_33.transpose());
    cout << solver.eigenvalues() << endl << endl;
    cout << solver.eigenvectors() << endl << endl;
    
    Matrix<double, MAX_SIZE, MAX_SIZE> matrix_Xd = MatrixXd::Random(MAX_SIZE, MAX_SIZE);
    VectorXd V_Xd = VectorXd ::Random(MAX_SIZE);
    
    clock_t time = clock();
    MatrixXd x = matrix_Xd.inverse() * V_Xd;
    cout << "x.inverse(): "<< (clock() - time) * 1000 / (double)CLOCKS_PER_SEC << "ms" << endl << endl;

    time = clock();
    x = matrix_Xd.colPivHouseholderQr().solve(V_Xd);
    cout << "Qr:" << (clock() - time) * 1000 / (double)CLOCKS_PER_SEC << "ms" << endl << endl;

    
    return 0;

}
