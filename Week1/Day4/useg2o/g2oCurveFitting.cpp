#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
using namespace std;
using namespace ceres;
using namespace g2o;
using namespace Eigen;

class CurveFittingVertex : public BaseVertex<3, Vector3d> {
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setTo_riginImpl() override {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) override {
        _sitimate += Eigen::Vector3d(update);
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) {}
};


int main() {


    return 0;
}