void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;// 收到一帧 LiDAR 点云，帧计数加 1
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)//较当前帧 LiDAR 时间戳和上一帧 LiDAR 时间戳，检查时间有没有倒退
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);//把 ROS 的 PointCloud2 点云解析/预处理成 FAST-LIO 内部使用的 PCL 点云
    lidar_buffer.push_back(ptr);//存当前帧点云本体
    time_buffer.push_back(msg->header.stamp.toSec());//存当前帧点云的时间戳
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;//记录第 scan_count 帧点云预处理花了多久
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}