#include <iostream>
#include <vector>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;
using namespace ceres;

// 残差类：每一个数据点都会对应一个 CURVE_FITTING_COST 对象
struct CURVE_FITTING_COST {
    // 构造函数：把当前数据点的 x, y 保存到 _x, _y
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    // 模板函数：Ceres 自动求导需要用 T 类型
    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        // abc[0] = a
        // abc[1] = b
        // abc[2] = c

        // residual = 观测值 - 预测值
        residual[0] = T(_y) - exp(
            abc[0] * T(_x) * T(_x) +
            abc[1] * T(_x) +
            abc[2]
        );

        return true;
    }

    const double _x, _y;
};

int main() {
    // 真实参数，用来生成数据
    double ra = 1.0, rb = 2.0, rc = 1.0;

    // 初始估计参数，故意给错
    double ea = 2.0, eb = -1.0, ec = 5.0;

    int N = 100;
    double w_sigma = 1.0;

    RNG rng;

    vector<double> x_data, y_data;

    // 生成带噪声的数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;

        double y = exp(ra * x * x + rb * x + rc)
                 + rng.gaussian(w_sigma);

        x_data.push_back(x);
        y_data.push_back(y);
    }

    // 待优化变量
    double abc[3] = {ea, eb, ec};

    // 构建 Ceres 优化问题
    Problem problem;

    for (int i = 0; i < N; i++) {
        // 每个数据点添加一个残差块
        problem.AddResidualBlock(
            new AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,
            abc
        );
    }

    // 设置求解器
    Solver::Options options;
    options.linear_solver_type = DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // 开始优化
    ceres::Solve(options, &problem, &summary);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    cout << summary.BriefReport() << endl;

    cout << "estimated a, b, c = ";
    for (auto a : abc) {
        cout << a << " ";
    }
    cout << endl;

    cout << "true a, b, c = "
         << ra << " " << rb << " " << rc << endl;

    return 0;
}