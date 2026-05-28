# SLAM-Journey 🛰️🤖

> **SLAM 学习与科研记录**

---

## 📅 每周学习记录

### 🔹 Week 1

#### 🗓️ Day 1
*   **数学基础与手推公式**：
    *   📝 完整手推了**罗德里格斯旋转公式**并完成备份：[罗德里格斯公式.jpg](Week1/Day1/罗德里格斯公式.jpg)
    *   📝 完整手推了**四元数到旋转矩阵**的转换过程：[四元数到旋转矩阵.jpg](Week1/Day1/四元数到旋转矩阵.jpg)
*   **FAST-LIO2 代码精读与框架分析**：
    *   📂 梳理并建立了 FAST-LIO2 核心文件目录结构说明：[fast_lio目录说明.md](Week1/Day1/fast_lio目录说明.md)
    *   🎨 绘制并整理了 FAST-LIO2 的主流程数据流向图：[fast_lio流程图.png](Week1/Day1/fast_lio流程图.png)

#### 🗓️ Day 2
*   **数学基础与手推公式**：
    *   📝 完整手写推导了**李群与李代数指数映射**（泰勒展开化简）：[指数映射.jpg](Week1/Day2/指数映射.jpg)
    *   📝 完整手写推导了**李代数扰动模型导数**公式：[扰动模型导数.jpg](Week1/Day2/扰动模型导数.jpg)
*   **FAST-LIO2 核心代码精读（逐行注释）**：
    *   📂 精读并注释了 IMU 测量数据接收回调函数：[imu_cbk.cpp](Week1/Day2/imu_cbk.cpp)
    *   📂 剖析并梳理了视野外局部地图点删除逻辑：[lasermap_fov_segment.cpp](Week1/Day2/lasermap_fov_segment.cpp)
    *   📂 深入理解并剖析了 iKD-Tree 增量插入与更新部分：[map_incremental.cpp](Week1/Day2/map_incremental.cpp)

#### 🗓️ Day 3
*   **C++ SLAM 基础与李群代数实践 (`useEigen/` 目录)**：
    *   📂 矩阵运算与几何变换练习：[useMatrix.cpp](Week1/Day3/useEigen/useMatrix.cpp), [useGeometry.cpp](Week1/Day3/useEigen/useGeometry.cpp), [coordinateTransform.cpp](Week1/Day3/useEigen/coordinateTransform.cpp)
    *   📂 轨迹绘制与误差计算：[plotTrajectory.cpp](Week1/Day3/useEigen/plotTrajectory.cpp), [trajectoryError.cpp](Week1/Day3/useEigen/trajectoryError.cpp)
    *   📂 李群李代数 Sophus 库实践：[useSophus.cpp](Week1/Day3/useEigen/useSophus.cpp)
    *   📂 三维位姿可视化实践：[visualizeGeometry.cpp](Week1/Day3/useEigen/visualizeGeometry.cpp)
    *   💻 编译运行结果截图：[Eigen和Sophus代码.png](Week1/Day3/Eigen和Sophus代码.png)
*   **FAST-LIO2 代码精读与状态估计**：
    *   📂 逐行精读点云特征预处理回调函数：[standard_pcl_cbk.cpp](Week1/Day3/standard_pcl_cbk.cpp)
    *   📂 精读并注释了 IMU-LiDAR 时间戳同步与数据包对齐逻辑：[sync_packages.cpp](Week1/Day3/sync_packages.cpp)
    *   📝 整理并绘制了状态估计中各种帧与坐标系含义图：[帧的含义.jpeg](Week1/Day3/帧的含义.jpeg)
    *   🎨 详细手绘了 IMU-LiDAR 从数据输入到进入队列的完整流程图：[数据流图.jpeg](Week1/Day3/数据流图（IMULiDAR进来到队列的完整流程）.jpeg)

#### 🗓️ Day 4
*   **数学基础与非线性优化实践**：
    *   📝 完整手写推导了非线性优化中的**高斯牛顿法（Gauss-Newton）正规方程**：[高斯牛顿法正规方程.jpg](Week1/Day4/高斯牛顿法正规方程.jpg)
    *   📂 Ceres 优化库曲线拟合实践与自动求导：[Ceres_AutoDiff.cpp](Week1/Day4/useCeres/Ceres_AutoDiff.cpp), [ceresCurveFitting.cpp](Week1/Day4/useCeres/ceresCurveFitting.cpp)
    *   📂 g2o 图优化库曲线拟合实践与顶点/边设计：[g2oCurveFitting.cpp](Week1/Day4/useg2o/g2oCurveFitting.cpp)
    *   💻 优化求解输出及曲线拟合可视化结果：[ch6_test1.jpg](Week1/Day4/ch6_test1.jpg), [ch6_test2.png](Week1/Day4/ch6_test2.png), [ch6_test3.png](Week1/Day4/ch6_test3.png)
*   **FAST-LIO2 ESEKF / IEKF（迭代卡尔曼滤波）核心逻辑精读**：
    *   📂 FAST-LIO2 核心迭代卡尔曼滤波框架文件：[esekfom.hpp](Week1/Day4/esekfom.hpp)
    *   📂 IEKF 核心状态迭代更新函数（逐行剖析与注释）：[update_iterated_dyn_share_modified_annotated.cpp](Week1/Day4/update_iterated_dyn_share_modified_annotated.cpp)
    *   📝 手绘 ESEKF 误差状态转移与测量线性化流向图：[update_iterated_dyn_share_modified_annotated.jpeg](Week1/Day4/update_iterated_dyn_share_modified_annotated.jpeg)
    *   🎨 迭代增益 $K$ 计算与 For 循环中断收敛条件分析：[IEKF 迭代 for 循环.png](Week1/Day4/IEKF%20迭代%20for%20循环.png)

#### 🗓️ Day 5
*   **数学基础**：
    *   📝 手写推导了**卡尔曼滤波（KF）5个核心公式**：[KF.jpg](Week1/Day5/KF.jpg)
*   **非线性优化实践**：
    *   📂 重写并完善了 g2o 图优化曲线拟合代码（含详细注释）：[g2oCurveFitting.cpp](Week1/Day5/useg2o/g2oCurveFitting.cpp)
    *   💻 Ceres 与 g2o 两种优化框架的对比总结：[对比 Ceres 和 g2o.png](Week1/Day5/对比%20Ceres%20和%20g2o.png)
*   **FAST-LIO2 IEKF 代码精读**：
    *   📂 IEKF 迭代更新函数完整版逐行注释：[update_iterated_dyn_share_modified.cpp](Week1/Day5/update_iterated_dyn_share_modified.cpp)
    *   🎨 手绘 FAST-LIO2 从 LiDAR 数据输入到位姿输出的完整主线流程图：[主线.jpeg](Week1/Day5/主线.jpeg)

#### 🗓️ Day 6
*   **数学基础**：
    *   📝 手写推导了 **EKF（扩展卡尔曼滤波）** 公式：[EKF.jpg](Week1/Day6/EKF.jpg)
    *   📝 手写推导了 **IEKF（迭代扩展卡尔曼滤波）** 公式及迭代逻辑：[IEKF.jpeg](Week1/Day6/IEKF.jpeg)
    *   📝 分析了 IEKF 为什么需要迭代（线性化误差与重新展开）：[IEKF 为什么需要迭代.jpeg](Week1/Day6/IEKF%20为什么需要迭代.jpeg)
*   **FAST-LIO2 核心代码精读**：
    *   📂 点面残差计算核心函数逐行注释：[h_share_model.cpp](Week1/Day6/h_share_model.cpp)
    *   💻 h_share_model 函数运行截图：[h_share_model.png](Week1/Day6/h_share_model.png)
    *   📂 iKD-Tree 增量插入函数精读：[Add_Points.cpp](Week1/Day6/Add_Points.cpp)
    *   📂 iKD-Tree K近邻搜索函数精读：[Search.cpp](Week1/Day6/Search.cpp)
    *   📂 iKD-Tree 批量删除函数精读：[Delete_Point_Boxes.cpp](Week1/Day6/Delete_Point_Boxes.cpp)
    *   🎨 更新后的 FAST-LIO2 完整主线流程图：[主线.jpeg](Week1/Day6/主线.jpeg)
    *   📝 各模块接口调用关系图：[接口用图.jpeg](Week1/Day6/接口用图.jpeg)

#### 🗓️ Day 7
*   **手写 Python 状态估计器**：
    *   📂 从零手写 1D 卡尔曼滤波仿真：[1DKF.py](Week1/Day7/KF/1DKF.py)
    *   📂 从零手写 2D 扩展卡尔曼滤波仿真：[2DKF.py](Week1/Day7/KF/2DKF.py)
    *   💻 2D EKF 仿真结果（真值/观测/滤波对比图）：[ekf_2d_result.png](Week1/Day7/KF/ekf_2d_result.png)
*   **KF/EKF/IEKF 对比与总结**：
    *   📝 KF、EKF、IEKF 三者对比总结图：[KF.EKF.IEKF 对比.jpeg](Week1/Day7/KF.EKF.IEKF%20对比.jpeg)
*   **FAST-LIO2 完整数据流追踪验证**：
    *   🎨 一帧 LiDAR 数据从输入到输出位姿的完整生命周期图：[一帧 LiDAR 的完整生命周期.jpeg](Week1/Day7/一帧%20LiDAR%20的完整生命周期.jpeg)
    *   📝 数据流追踪总结图：[数据流追踪.jpeg](Week1/Day7/数据流追踪.jpeg)
    *   📝 为什么 IMU 需要积分的原理说明：[为什么 IMU 需要积分.png](Week1/Day7/为什么%20IMU%20需要积分.png)
    *   💻 用 std::cout 打印验证追踪正确性截图：[打印验证追踪.png](Week1/Day7/打印验证追踪.png)

### 🔹 Week 2
*（在此记录你的 Week 2 学习进展...）*

### 🔹 Week 3
*（在此记录你的 Week 3 学习进展...）*

### 🔹 Week 4
*（在此记录你的 Week 4 学习进展...）*

### 🔹 Week 5
*（在此记录你的 Week 5 学习进展...）*

### 🔹 Week 6
*（在此记录你的 Week 6 学习进展...）*

### 🔹 Week 7
*（在此记录你的 Week 7 学习进展...）*

### 🔹 Week 8
*（在此记录你的 Week 8 学习进展...）*
