有些小车车身比较长，如果是一个激光雷达，顾前不顾后，有比较大的视野盲区，这对小车导航定位避障来说都是一个问题，比如AGV小车，

所有想在小车前后各加一个雷达，那问题是ROS的建图或者定位导航都只是支持一个雷达，这个时候就需要我们做2个雷达的融合了。

方法比较简单：我的思路是先将两个激光雷达获得的laser_scan转成point_cloud也就是点云，利用pcl库将两个点云拼接在一起，然后在把拼接后的点云重新转成laser_scan。

这样ros里面建图导航都可以用了。

关键点是要把两个激光雷达的偏移量算好，以及雷达的时间同步。代码也比较简单，贴出部分关键代码：

//时间同步

message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_left(nh, left_topic, 10);
message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_right(nh, right_topic, 10);
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), points_sub_left, points_sub_right);
sync.registerCallback(boost::bind(&callback, _1, _2));

g_left_right_point_pub = nh.advertise<sensor_msgs::PointCloud2>(fusion_topic, 10);

//ros回调函数，拼接点云

void callback(const sensor_msgs::PointCloud2::ConstPtr& left_input, const sensor_msgs::PointCloud2::ConstPtr& right_input)
{
pcl::PointCloud<pcl::PointXYZI>::Ptr left_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
pcl::fromROSMsg(*left_input, *left_local_laser);

pcl::PointCloud<pcl::PointXYZI>::Ptr left_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::transformPointCloud(*left_local_laser, *left_calibration_cloud, g_left_calibration_matrix);

for(std::size_t i = 0; i < left_calibration_cloud->size(); ++i)
{
left_calibration_cloud->points[i].intensity = 64;
}

// publishCloudI(&g_left_calib_point_pub, *left_calibration_cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr right_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
pcl::fromROSMsg(*right_input, *right_local_laser);

pcl::PointCloud<pcl::PointXYZI>::Ptr right_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::transformPointCloud(*right_local_laser, *right_calibration_cloud, g_right_calibration_matrix);
for(std::size_t i = 0; i < right_calibration_cloud->size(); ++i)
{
right_calibration_cloud->points[i].intensity = 128;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr left_right_middle_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
*left_right_middle_calibration_cloud = *left_calibration_cloud + *right_calibration_cloud;

publishCloudI(&g_left_right_point_pub, *left_right_middle_calibration_cloud);
}
