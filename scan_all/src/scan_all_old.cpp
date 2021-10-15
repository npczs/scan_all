
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
  
/*#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include <pcl/visualization/pcl_visualizer.h>*/

laser_geometry::LaserProjection projector_;

sensor_msgs::PointCloud2 cloud;
sensor_msgs::PointCloud2 cloud_2;

  
//Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

//transform_2.translation() << -0.240, 0.0, -0.106;
//transform_2.rotate (Eigen::AngleAxisf (3.142, Eigen::Vector3f::UnitZ()));

/*void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
     scan_all_pub.publish(msg);
}*/

/*class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish()  
  {  
        //Topic you want to publish  
        pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_all", 1000);  
        point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        //Topic you want to subscribe  

        //sub_ = n_.subscribe("scan", 1000, &SubscribeAndPublish::callback, this);
        //sub_2_= n_.subscribe("scan_2", 1000, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
  }  
      
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,const sensor_msgs::LaserScan::ConstPtr& scan_2_msg)  
  {  
        //pub_.publish(msg);
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan_msg, cloud); 
        point_cloud_publisher_.publish(cloud);
  }
      
private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;
      ros::Publisher point_cloud_publisher_;
      laser_geometry::LaserProjection projector_;      
      
      ros::Subscriber sub_;  
      ros::Subscriber sub_2_;
}; */


void callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,const sensor_msgs::LaserScan::ConstPtr& scan_2_msg)
{ 
        
        //tf::TransformListener tfListener_;
        //tfListener_.setExtrapolationLimit(ros::Duration(0.1));
      
      
        /*if(!tfListener_.waitForTransform(
          scan_msg->header.frame_id,
        "laser_link",
        scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment),
        ros::Duration(1.0)))
        {
           return;
        }
        projector_.transformLaserScanToPointCloud("laser_link", *scan_msg, cloud, tfListener_);*/
        projector_.projectLaser(*scan_msg, cloud);
        //std::cout<<"transfrom done"<<std::endl;
        projector_.projectLaser(*scan_2_msg, cloud_2);
        
        
        
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
  point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(n_, "scan", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_2_sub(n_, "scan_2", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan_sub, scan_2_sub);
  //message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync(scan_sub, scan_2_sub, 10);

  ros::Rate loop_rate(10);
  //SubscribeAndPublish SAPObject;
  while (ros::ok())
  {
    sync.registerCallback(boost::bind(&callback, _1, _2));
    point_cloud_publisher_.publish(cloud);

    ros::spinOnce();
    loop_rate.sleep();
 }
  //ros::spin();
  return 0;
}



