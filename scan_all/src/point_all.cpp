#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <iostream>
  
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

laser_geometry::LaserProjection projector_;

//sensor_msgs::PointCloud2 cloud;
//sensor_msgs::PointCloud2 cloud_2;
sensor_msgs::PointCloud2 cloud_all;
int statue=0;

   Eigen::Matrix4f transform_1;
   Eigen::Matrix4f transform_2;
   //Eigen::Matrix<float, 4, 4> matrix_44;
  // 定义一个旋转矩阵 (见 https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta_1 = 0.0; // 弧度角
  float theta_2 = 3.142;
  //transform_1 (0,1) = -sin(theta);
  //transform_1 (1,0) = sin (theta);
  //transform_1 (1,1) = cos (theta);
      	//(行, 列)
 
  // 在 X 轴上定义一个 2.5 米的平移.
  //transform_1(0,3) = -0.240;

  //transform_1(2,3) = -0.106;

 //Eigen::Affine3f transform= Eigen::Affine3f::Identity();
 //Eigen::Affine3f transform_2;



void callback(const sensor_msgs::PointCloud2::ConstPtr& point_msg,const sensor_msgs::PointCloud2::ConstPtr& point_2_msg)
{ 
        
       /* transform_1 (0,0) = cos (theta);
        transform_1 (0,1) = -sin(theta);
        transform_1 (1,0) = sin (theta);
        transform_1 (1,1) = cos (theta);
        transform_1 (0,3) = -0.240;
        transform_1 (2,3) = 0.106;*/
        transform_1 (0,0) = cos (theta_1);
        transform_1 (0,1) = -sin(theta_1);
        transform_1 (1,0) = sin (theta_1);
        transform_1 (1,1) = cos (theta_1);
        transform_1 (0,3) = -0.240;
        transform_1 (2,3) = -0.133;

        transform_2 (0,0) = cos (theta_2);
        transform_2 (0,1) = -sin(theta_2);
        transform_2 (1,0) = sin (theta_2);
        transform_2 (1,1) = cos (theta_2);
        //transform_2 (0,3) = 0.0;
        transform_2 (2,3) = -0.240;

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_pcl_2(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::fromROSMsg(*point_msg, *scan_pcl);
        pcl::fromROSMsg(*point_2_msg, *scan_pcl_2);
 
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::transformPointCloud (*scan_pcl, *transformed_cloud, transform_1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_2 (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::transformPointCloud (*scan_pcl_2, *transformed_cloud_2, transform_2);

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_all_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        *scan_all_pcl = *transformed_cloud + *transformed_cloud_2;
        pcl::toROSMsg(*scan_all_pcl, cloud_all);
        statue=1;
        std::cout<<statue<<std::endl;


        
        
        
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "point_all");  
  ros::NodeHandle n_;
  ros::Publisher point_cloud_publisher_;
  point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud_all", 10000, false);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub(n_, "cloud", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_2_sub(n_, "cloud_2", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_sub, point_2_sub);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    sync.registerCallback(boost::bind(&callback, _1, _2));
    std::cout<<"123"<<std::endl;
    if(statue==1)
    {
      point_cloud_publisher_.publish(cloud_all);
      statue=1;
    }
        
    ros::spinOnce();
    loop_rate.sleep();
 }
  return 0;
}



