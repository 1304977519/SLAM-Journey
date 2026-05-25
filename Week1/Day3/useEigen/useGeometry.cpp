
#include <cmath>
#include <iostream>
using namespace std;
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;
int main() {
    Matrix3d rotation_matrix = Matrix3d::Identity();
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout << rotation_vector.matrix() << endl << endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout << rotation_matrix << endl << endl;
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << v_rotated << endl << endl;
    v_rotated = rotation_matrix * v;
    cout << v_rotated << endl << endl;
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << euler_angles.transpose() << endl << endl;


    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_matrix);
    T.pretranslate(Vector3d(1, 3, 4));
    cout << T.matrix() << endl << endl;


    Vector3d v_transformed = T * v;
    cout << T.rotation() << endl << endl;
    cout << T.translation().transpose() << endl << endl;
    cout << v_transformed << endl << endl;
      
   
    Quaterniond q = Quaterniond(rotation_vector);
    cout << q.coeffs().transpose() << endl << endl;
    q = Quaterniond(rotation_matrix);
    cout << q.coeffs().transpose() << endl << endl;
    v_rotated = q * v;
    cout << v << endl << endl;
    cout << v_rotated << endl << endl;
   
    // Eigen 打印 coeffs() 的顺序是：x y z w
    // 构造时是：Quaterniond(w, x, y, z)
    
    return 0;
}
