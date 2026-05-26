#include <iostream>
#include <ceres/ceres.h>
using namespace std;
using namespace ceres;
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};
int main() {
    double initial_x = 5.0;
    double x = initial_x;
    Problem problem;
    CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    Solver::Options option;
    option.linear_solver_type = DENSE_QR;
    option.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(option, &problem, &summary);
    cout << summary.BriefReport() << endl;
    cout << "X: " << initial_x << 
        " -> " << x << endl;
    return 0;
}