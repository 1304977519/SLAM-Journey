
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) // msg_in 是一个指向只读 IMU 消息的智能指针引用。
// ConstPtr 表示不能修改它指向的 IMU 消息内容。
// 前面的 const 表示不能在函数里修改这个智能指针本身。
// & 表示引用传参，避免复制智能指针。
{
    publish_count ++;// 记录收到的第几帧 IMU 消息
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));// msg_in 是只读的，所以复制一份新的 IMU 消息，用 msg 指向它，后面可以修改时间戳

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);// 用用户配置的固定时间差，把 IMU 时间戳归算到 LiDAR 时间系
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)// 如果开启自动时间同步，并且程序估计出的 LiDAR/IMU 时间差绝对值超过 100ms
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());// 用程序估计出的时间差重新修正 IMU 时间戳；上面是用户配置的固定补偿，这里是自动补偿
    }

    double timestamp = msg->header.stamp.toSec();// 取出修正后的 IMU 时间戳，转成 double 秒

    mtx_buffer.lock();// 给共享 buffer 加锁，防止别的线程同时读写

    if (timestamp < last_timestamp_imu)// 判断修正后的 IMU 时间戳是否倒流
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();// 清空旧的 IMU 缓存，防止前后时间顺序乱掉，旧缓存不要了，从当前这帧重新开始。
    }

    last_timestamp_imu = timestamp;// 记录当前这帧 IMU 的时间，作为下一帧的比较基准

    imu_buffer.push_back(msg);// 把修正过时间戳的 IMU 消息放进 IMU 缓存队列
    mtx_buffer.unlock();// 解锁，允许其他线程继续访问共享 buffer
    sig_buffer.notify_all();// 通知等待 buffer 数据的线程：现在有新 IMU 数据了，可以醒来检查
}