
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
using namespace std;
using namespace Eigen;
using namespace Sophus;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
int main() {
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Quaterniond q(R);
    SO3d SO3_R(R);
    SO3d SO3_q(q);
    cout << SO3_R.matrix() << endl << endl;
    cout << SO3_q.matrix() << endl << endl;
    Vector3d so3 = SO3_R.log();
    cout << so3 << endl << endl;
    //反对称矩阵
    cout << SO3d::hat(so3) << endl << endl;
    //转回向量
    cout << SO3d::vee(SO3d::hat(so3)) << endl << endl;
    Vector3d update_so3(1e-4, 0, 0);
    SO3d SO3_updated = SO3d::exp(update_so3) * SO3_R;
    cout << SO3_updated.matrix() << endl << endl;

    Vector3d t(1, 0, 0);
    SE3d SE3_R(R, t);
    SE3d SE3_q(q, t);
    cout << SE3_R.matrix() << endl << endl;
    cout << SE3_q.matrix() << endl << endl;

    //这个先平移，再旋转
    Vector6d se3 = SE3_R.log();
    cout << se3 << endl << endl;
    cout << SE3d::hat(se3) << endl << endl;
    cout << SE3d::vee(SE3d::hat(se3)) << endl << endl;

    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    SE3d SE3_update = SE3d::exp(update_se3) * SE3_R;
    cout << SE3_update.matrix() << endl << endl;
    return 0;
}
