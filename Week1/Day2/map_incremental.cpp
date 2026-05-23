void map_incremental()
{
    PointVector PointToAdd;                         // 需要加入地图、但加入时还要让 ikd-tree 再做 downsample 判断的点
    PointVector PointNoNeedDownsample;              // 不需要再 downsample、可以直接加入地图的点
    PointToAdd.reserve(feats_down_size);            // 提前给 PointToAdd 预留空间，避免 push_back 时频繁扩容
    PointNoNeedDownsample.reserve(feats_down_size); // 提前给 PointNoNeedDownsample 预留空间

    for (int i = 0; i < feats_down_size; i++)       // 遍历当前帧降采样后的每一个点
    {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i])); // 把当前点从 body/LiDAR 坐标系转到 world 世界坐标系

        if (!Nearest_Points[i].empty() && flg_EKF_inited) // 如果当前点在地图里找到了最近邻旧点，并且 EKF 已经初始化，就认真判断要不要加地图
        {
            const PointVector &points_near = Nearest_Points[i]; // 当前点在 ikd-tree 地图里找到的最近邻旧地图点，只读引用，不复制
            bool need_add = true;                               // 默认当前点需要加入地图，后面如果发现旧点更合适，就改成 false
            BoxPointType Box_of_Point;                          // 这段代码里基本没用，可能是旧版本遗留变量
            PointType downsample_result, mid_point;             // downsample_result 基本没用；mid_point 是当前点所在体素的中心

            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min; // 计算当前点所在体素中心的 x 坐标
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min; // 计算当前点所在体素中心的 y 坐标
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min; // 计算当前点所在体素中心的 z 坐标

            float dist = calc_dist(feats_down_world->points[i], mid_point); // 当前新点到自己所在体素中心的距离，后面拿它和旧地图点比较

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) // 如果最近的旧地图点在 x/y/z 三个方向上都离当前体素中心超过半个体素，说明当前体素附近基本没合适旧点
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); // 当前新点直接加入“不需要 downsample”的点集合
                continue;                                                     // 当前点处理完，直接看下一个点
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) // 遍历当前点附近的若干个旧地图点
            {
                if (points_near.size() < NUM_MATCH_POINTS) break; // 保险判断：如果最近邻数量不够，就退出，防止访问越界

                if (calc_dist(points_near[readd_i], mid_point) < dist) // 如果某个旧地图点比当前新点更靠近体素中心
                {
                    need_add = false; // 说明这个体素里已经有更好的代表点，当前新点不用加
                    break;            // 已经确定不用加了，退出循环
                }
            }

            if (need_add) PointToAdd.push_back(feats_down_world->points[i]); // 如果没有旧点比当前点更靠近体素中心，当前点仍然值得加入
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]); // 如果没找到最近邻旧点，或者 EKF 没初始化，就不复杂判断，直接放入待加入点集合
        }
    }

    double st_time = omp_get_wtime();                                      // 记录开始加点的时间
    add_point_size = ikdtree.Add_Points(PointToAdd, true);                 // 把 PointToAdd 加入 ikd-tree，true 表示加入时还要做 downsample 控制
    ikdtree.Add_Points(PointNoNeedDownsample, false);                      // 把 PointNoNeedDownsample 加入 ikd-tree，false 表示不再 downsample，直接插入
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();     // 统计本次准备加入地图的点数，这里会覆盖上一句 Add_Points 的返回值
    kdtree_incremental_time = omp_get_wtime() - st_time;                   // 统计本次 ikd-tree 增量加点耗时
}