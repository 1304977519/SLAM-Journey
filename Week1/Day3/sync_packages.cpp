bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;// LiDAR 或 IMU 任意一个队列为空，都没法组成完整测量包
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();// 取出 LiDAR 队列最前面这一帧，先放进 meas，注意此时还没有从 lidar_buffer 弹出
        meas.lidar_beg_time = time_buffer.front();// 当前 LiDAR 帧的开始时间，也就是点云 msg 的 header.stamp

        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;// 点太少，无法用最后一个点的相对时间估计帧长，只能用历史平均帧长兜底
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;// 最后一个点的相对时间太短，不像正常一帧，认为这个时间不可靠，用平均帧长兜底
        }// 上面两个分支都是为了避免异常帧长影响同步，也避免异常扫描时间污染 lidar_mean_scantime
        else
        {
            scan_num ++;// 记录参与平均扫描时间统计的正常 LiDAR 帧数，不是点数
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);// 当前帧结束时间 = 当前帧开始时间 + 最后一个点的帧内相对时间
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;// 增量平均：用当前正常帧的扫描时间更新平均扫描时间
        }
        if(lidar_type == MARSIM)// 如果是 MARSIM 仿真点云，就把一帧当成瞬时点云处理
            lidar_end_time = meas.lidar_beg_time;// 仿真情况下不考虑帧内扫描持续时间，开始时间等于结束时间

        meas.lidar_end_time = lidar_end_time;// 把当前 LiDAR 帧的结束时间存进 meas，后面 IMU 处理和去畸变要用

        lidar_pushed = true;// 当前 LiDAR 帧已经被选中并放进 meas，后面如果 IMU 不够，就继续等这同一帧，不要重新取新的 LiDAR
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;// 最新 IMU 时间还没覆盖到当前 LiDAR 帧结束时间，说明 IMU 数据还不够，先等下一次调用
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();// 清空上一次测量包里的 IMU，因为下面要重新装当前 LiDAR 帧对应的 IMU；LiDAR 指针会被直接赋值覆盖，所以不用 clear
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();// 查看当前 IMU 队首数据的时间
        if(imu_time > lidar_end_time) break;// 如果这个 IMU 已经超过当前 LiDAR 帧结束时间，就留给下一帧 LiDAR 用
        meas.imu.push_back(imu_buffer.front());// 把当前 LiDAR 帧结束时间之前的 IMU 放进 meas.imu
        imu_buffer.pop_front();// 这个 IMU 已经被当前测量包取走，所以从 IMU 队列弹出
    }

    lidar_buffer.pop_front();// 当前 LiDAR 帧已经和对应 IMU 打包完成，可以从 LiDAR 队列弹出
    time_buffer.pop_front();// 当前 LiDAR 帧的开始时间也同步弹出
    lidar_pushed = false;// 当前帧处理完成，下次 sync_packages 可以去取下一帧 LiDAR
    return true;// 返回 true 表示成功组成一个完整测量包：一帧 LiDAR + 对应时间段 IMU
}