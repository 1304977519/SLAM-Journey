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
