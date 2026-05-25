
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;
int main() {
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize(), q2.normalize();
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    Isometry3d T1= Isometry3d::Identity(), T2 = Isometry3d::Identity();
    T1.rotate(q1), T1.pretranslate(t1);
    T2.rotate(q2), T2.pretranslate(t2);
    Vector3d p1(0.5, 0, 0.2), p2;
    p2 = T2 * T1.inverse() * p1;
    cout << p2 << endl << endl;
    return 0;
}
