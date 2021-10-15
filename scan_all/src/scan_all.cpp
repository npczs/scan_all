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

sensor_msgs::PointCloud2 cloud;
sensor_msgs::PointCloud2 cloud_2;
sensor_msgs::PointCloud2 cloud_all;
int statue=0;

   Eigen::Matrix4f transform_1;
   //Eigen::Matrix<float, 4, 4> matrix_44;
  // 定义一个旋转矩阵 (见 https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = 3.142; // 弧度角

  //transform_1 (0,1) = -sin(theta);
  //transform_1 (1,0) = sin (theta);
  //transform_1 (1,1) = cos (theta);
      	//(行, 列)
 
  // 在 X 轴上定义一个 2.5 米的平移.
  //transform_1(0,3) = -0.240;

  //transform_1(2,3) = -0.106;

 //Eigen::Affine3f transform= Eigen::Affine3f::Identity();
 //Eigen::Affine3f transform_2;



void callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,const sensor_msgs::LaserScan::ConstPtr& scan_2_msg)
{ 
        
        transform_1 (0,0) = cos (theta);
        transform_1 (0,1) = -sin(theta);
        transform_1 (1,0) = sin (theta);
        transform_1 (1,1) = cos (theta);
        transform_1 (0,3) = -0.240;
        transform_1 (2,3) = -0.106;

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_pcl_2(new pcl::PointCloud<pcl::PointXYZI>());
        projector_.projectLaser(*scan_msg, cloud);
        pcl::fromROSMsg(cloud, *scan_pcl);
        //std::cout<<"transfrom done"<<std::endl;
        projector_.projectLaser(*scan_2_msg, cloud_2);
        pcl::fromROSMsg(cloud_2, *scan_pcl_2);
        //matrix_44 << cos (theta),-sin(theta),0,-0.240,sin (theta),cos (theta),0,0,0,0,1,-0.106,0,0,0,1;
        //transform.translation() << -0.240, 0.0, -0.106;
        //transform.rotate (Eigen::AngleAxisf (3.142, Eigen::Vector3f::UnitZ()));

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::transformPointCloud (*scan_pcl_2, *transformed_cloud, transform_1);
        /*for(std::size_t i = 0; i < transformed_cloud->size(); ++i)
        {
          transformed_cloud->points[i].intensity = 64;
        }
        for(std::size_t i = 0; i < scan_pcl->size(); ++i)
        {
          transformed_cloud->points[i].intensity = 128;
        }*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_all_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        *scan_all_pcl =   *scan_pcl+*transformed_cloud;
        pcl::toROSMsg(*scan_all_pcl, cloud_all);
        //point_cloud_publisher_.publish(cloud_all);
        //publishCloudI(&point_cloud_publisher_, *scan_all_pcl);
        statue=1;
        std::cout<<statue<<std::endl;


        
        
        
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "scan_all");  
  ros::NodeHandle n_;
  
  //ros::Publisher scan_all_pub = n.advertise<sensor_msgs::LaserScan>("scan_all", 1000);
  /*ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe("scan", 1000, Callback);
  ros::Subscriber sub_2 = .subscribe("scan_2", 1000, Callback);*/
  ros::Publisher point_cloud_publisher_;
  point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud", 10000, false);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(n_, "scan", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_2_sub(n_, "scan_2_filtered", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan_sub, scan_2_sub);
  //message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync(scan_sub, scan_2_sub, 10);
  //sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::Rate loop_rate(30);
  //SubscribeAndPublish SAPObject;
  while (ros::ok())
  {
    sync.registerCallback(boost::bind(&callback, _1, _2));
    std::cout<<"123"<<std::endl;
    point_cloud_publisher_.publish(cloud_all);    
    ros::spinOnce();
    loop_rate.sleep();
 }
  //ros::spin();
  return 0;
}



