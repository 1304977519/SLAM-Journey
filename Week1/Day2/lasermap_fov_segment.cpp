void lasermap_fov_segment()
{
    cub_needrm.clear(); // 清空本次需要从 ikd-tree 删除的地图盒子列表
    kdtree_delete_counter = 0; // 本次删除的点数量先置 0
    kdtree_delete_time = 0.0; // 本次删除地图点的耗时先置 0
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world); 
    V3D pos_LiD = pos_lid; // 当前 LiDAR 解算中心在世界坐标系下的位置
    if (!Localmap_Initialized){ // 如果局部地图还没有初始化
        for (int i = 0; i < 3; i++){ // 分别初始化 x、y、z 三个方向的地图边界
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0; // 以当前 LiDAR 为中心，向负方向扩半个 cube_len
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0; // 以当前 LiDAR 为中心，向正方向扩半个 cube_len
        }
        Localmap_Initialized = true; // 标记局部地图已经初始化完成
        return; // 第一次只初始化地图盒子，不移动地图，也不删除旧地图
    }
    float dist_to_map_edge[3][2]; // 记录 LiDAR 到局部地图六个边界的距离，3 个方向，每个方向有 min/max 两侧
    bool need_move = false; // 标记局部地图是否需要移动，默认不移动
    for (int i = 0; i < 3; i++){ // 遍历 x、y、z 三个方向
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]); // LiDAR 到当前方向最小边界的距离
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]); // LiDAR 到当前方向最大边界的距离

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true; // 只要靠近任意一侧边界，就需要移动局部地图
    }
    if (!need_move) return; // 如果 LiDAR 没有靠近边界，说明局部地图不用动，直接退出
    BoxPointType New_LocalMap_Points, tmp_boxpoints; // New_LocalMap_Points 是移动后的新地图盒子，tmp_boxpoints 是本次要删除的旧边缘盒子
    New_LocalMap_Points = LocalMap_Points; // 先把旧地图盒子复制给新地图盒子，后面只改需要移动的方向
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1))); // 计算地图每次平移的距离，取较大值保证移动后 LiDAR 回到安全区域
    for (int i = 0; i < 3; i++){ // 遍历 x、y、z 三个方向，分别判断是否要移动
        tmp_boxpoints = LocalMap_Points; // 先把待删除盒子初始化成旧地图盒子，后面只裁剪当前方向
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){ // 如果 LiDAR 靠近当前方向的最小边界
            New_LocalMap_Points.vertex_max[i] -= mov_dist; // 新地图最大边界往负方向移动 mov_dist
            New_LocalMap_Points.vertex_min[i] -= mov_dist; // 新地图最小边界往负方向移动 mov_dist
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist; // 旧地图正方向多出来的一条边缘区域，是本次要删除的区域
            cub_needrm.push_back(tmp_boxpoints); // 把这个待删除盒子加入删除列表
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){ // 如果 LiDAR 靠近当前方向的最大边界
            New_LocalMap_Points.vertex_max[i] += mov_dist; // 新地图最大边界往正方向移动 mov_dist
            New_LocalMap_Points.vertex_min[i] += mov_dist; // 新地图最小边界往正方向移动 mov_dist
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist; // 旧地图负方向多出来的一条边缘区域，是本次要删除的区域
            cub_needrm.push_back(tmp_boxpoints); // 把这个待删除盒子加入删除列表
        }
    }
    LocalMap_Points = New_LocalMap_Points; // 用移动后的新地图盒子替换旧地图盒子
    points_cache_collect(); // 处理缓存点，防止地图移动和删除时漏掉还没加入 ikd-tree 的点
    double delete_begin = omp_get_wtime(); // 记录开始删除旧地图点的时间
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); // 如果有待删除盒子，就从 ikd-tree 删除这些盒子范围内的旧地图点
    kdtree_delete_time = omp_get_wtime() - delete_begin; // 统计本次删除操作耗时
}