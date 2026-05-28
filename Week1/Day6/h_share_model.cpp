void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime(); // 记录匹配开始时间，用来统计最近邻搜索/平面拟合耗时
    laserCloudOri->clear(); // 清空上一轮保存的有效原始 LiDAR 点
    corr_normvect->clear(); // 清空上一轮保存的法向量和残差
    total_residual = 0.0; // 本轮所有有效点的残差绝对值总和，后面用于算平均残差

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM); // 如果开启多线程，就设置 OpenMP 线程数
        #pragma omp parallel for // 并行遍历每个降采样后的 LiDAR 点
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; // 当前帧第 i 个降采样点，仍在 LiDAR/body 坐标系下
        PointType &point_world = feats_down_world->points[i]; // 用来保存该点变换到世界坐标系后的结果

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z); // 把 PCL 点转成 Eigen 向量，表示 p_i^L
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos); // p_i^W = R_WI * (R_IL * p_i^L + t_IL) + p_WI，也就是 T p_i
        point_world.x = p_global(0); // 保存世界系 x 坐标
        point_world.y = p_global(1); // 保存世界系 y 坐标
        point_world.z = p_global(2); // 保存世界系 z 坐标
        point_world.intensity = point_body.intensity; // 强度信息不参与残差计算，这里只是顺手保留

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS); // 保存最近邻搜索得到的距离平方，长度是需要的邻近点个数

        auto &points_near = Nearest_Points[i]; // 第 i 个点对应的邻近地图点缓存，后面用这些点拟合局部平面

        if (ekfom_data.converge) // 只有滤波迭代认为已经收敛/可以重新匹配时，才重新做最近邻搜索
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis); // 在 ikd-tree 地图里找 point_world 附近的若干地图点
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true; // 邻近点不够，或者第 N 个邻近点太远，就认为该点不能可靠拟合平面
        }

        if (!point_selected_surf[i]) continue; // 如果这个点没有可靠邻域，直接跳过

        VF(4) pabcd; // 保存拟合出来的平面参数 [a,b,c,d]，平面方程为 ax+by+cz+d=0
        point_selected_surf[i] = false; // 先默认该点无效，只有通过平面拟合和残差筛选后才重新置 true
        if (esti_plane(pabcd, points_near, 0.1f)) // 用邻近地图点拟合局部平面；0.1f 是拟合平面的误差阈值
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); // 点面残差 r_i = n_i^T p_i^W + d_i = n_i^T(Tp_i - q_i)
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); // 匹配质量打分：残差越大分数越低；注意这里的 s 是 score，不是函数参数里的状态 s

            if (s > 0.9) // 只有残差足够小、匹配质量足够高的点才保留下来
            {
                point_selected_surf[i] = true; // 标记该点是有效点面约束
                normvec->points[i].x = pabcd(0); // 保存平面法向量 n_i 的 x 分量，也就是 a
                normvec->points[i].y = pabcd(1); // 保存平面法向量 n_i 的 y 分量，也就是 b
                normvec->points[i].z = pabcd(2); // 保存平面法向量 n_i 的 z 分量，也就是 c
                normvec->points[i].intensity = pd2; // 借用 intensity 保存点面残差 r_i
                res_last[i] = abs(pd2); // 保存残差绝对值，用于统计平均残差
            }
        }
    }

    effct_feat_num = 0; // 有效点数量清零，下面重新压缩有效点

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i]) // 只处理通过筛选的点
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; // 保存原始 LiDAR/body 系下的点 p_i^L，后面算雅可比还要用
            corr_normvect->points[effct_feat_num] = normvec->points[i]; // 保存该点对应的平面法向量 n_i 和残差 r_i
            total_residual += res_last[i]; // 累加残差绝对值
            effct_feat_num ++; // 有效点数量加一
        }
    }

    if (effct_feat_num < 1) // 如果一个有效点都没有，本次量测更新无效
    {
        ekfom_data.valid = false; // 通知 ESEKF：这次观测不可用
        ROS_WARN("No Effective Points! \n"); // 打印警告
        return; // 直接退出，不再构造 H 和 h
    }

    res_mean_last = total_residual / effct_feat_num; // 计算本轮有效点的平均残差
    match_time  += omp_get_wtime() - match_start; // 累加匹配/残差计算耗时
    double solve_start_  = omp_get_wtime(); // 记录构造 H 和 h 的开始时间

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 初始化量测雅可比 H，每个有效点一行，12 列对应位置/姿态/外参旋转/外参平移
    ekfom_data.h.resize(effct_feat_num); // 初始化残差向量 h，每个有效点一个残差

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i]; // 取第 i 个有效原始 LiDAR 点 p_i^L
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z); // 转成 Eigen 向量，仍然是 LiDAR/body 系下的点
        M3D point_be_crossmat; // 准备保存 [p_i^L]x 反对称矩阵
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be); // 构造 LiDAR/body 系点的叉乘矩阵，用于外参旋转雅可比
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // 把点从 LiDAR 系变到 IMU 系：p_i^I = R_IL p_i^L + t_IL
        M3D point_crossmat; // 准备保存 [p_i^I]x 反对称矩阵
        point_crossmat<<SKEW_SYM_MATRX(point_this); // 构造 IMU 系点的叉乘矩阵，用于姿态雅可比

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i]; // 取该点对应的法向量和残差
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z); // norm_vec = n_i，是地图局部平面的法向量，位于世界系

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec); // C = R_WI^T n_i，把世界系法向量转到 IMU/body 局部坐标中
        V3D A(point_crossmat * C); // A = [p_i^I]x C，对应残差对当前姿态误差的雅可比部分
        if (extrinsic_est_en) // 如果开启外参在线估计，就同时构造外参旋转和平移的雅可比
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // B 对应残差对 LiDAR-IMU 外参旋转误差的雅可比
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C); // H_i = [对位置, 对姿态, 对外参旋转, 对外参平移]
        }
        else // 如果不开启外参估计，只更新位姿，外参相关列填 0
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // H_i = [n_i^T, A, 0, 0]
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity; // h_i = -r_i，因为线性化后希望 r_i + H_i * dx = 0，所以 H_i * dx = -r_i
    }
    solve_time += omp_get_wtime() - solve_start_; // 累加构造 H 和 h 的耗时
}
