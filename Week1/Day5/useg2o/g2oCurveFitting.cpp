#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;
using namespace Eigen;

class CurveFittingVertex : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream& in) override {
        return true;
    }

    virtual bool write(ostream& out) const override {
        return true;
    }
};

class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : g2o::BaseUnaryEdge<1, double, CurveFittingVertex>(), _x(x) {}

    virtual void computeError() override {
        const CurveFittingVertex* v =
            static_cast<const CurveFittingVertex*>(_vertices[0]);

        const Vector3d abc = v->estimate();

        _error(0, 0) =
            _measurement - exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    }

    virtual void linearizeOplus() override {
        const CurveFittingVertex* v =
            static_cast<const CurveFittingVertex*>(_vertices[0]);

        const Vector3d abc = v->estimate();

        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);

        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(istream& in) override {
        return true;
    }

    virtual bool write(ostream& out) const override {
        return true;
    }

public:
    double _x;
};

int main() {
    double ar = 1.0, br = 2.0, cr = 1.0;       // 真实参数
    double ae = 2.0, be = -1.0, ce = 5.0;      // 初始估计

    int N = 100;
    double w_sigma = 1.0;

    vector<double> x_data, y_data;
    cv::RNG rng;

    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);

        double y = exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma);
        y_data.push_back(y);
    }

    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()
        )
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    auto v = new CurveFittingVertex();
    v->setEstimate(Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i = 0; i < N; i++) {
        auto edge = new CurveFittingEdge(x_data[i]);

        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);

        edge->setInformation(
            Matrix<double, 1, 1>::Identity() * (1.0 / (w_sigma * w_sigma))
        );

        optimizer.addEdge(edge);
    }

    cout << "start optimization" << endl;

    auto t1 = chrono::steady_clock::now();

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    auto t2 = chrono::steady_clock::now();

    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}