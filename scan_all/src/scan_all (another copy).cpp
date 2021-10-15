
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <laser_geometry/laser_geometry.h>
#include <iostream>
 
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
//#include <pcl/visualization/pcl_visualizer.h>


/*void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
     scan_all_pub.publish(msg);
}*/

class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish()  
  {  
        //Topic you want to publish  
        pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_all", 1000);  
        point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        //Topic you want to subscribe  
        sub_ = n_.subscribe("scan", 1000, &SubscribeAndPublish::callback, this);
        sub_2_= n_.subscribe("scan_2", 1000, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
  }  
      
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg)  
  {  
        //pub_.publish(msg);
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*msg, cloud); 
        point_cloud_publisher_.publish(cloud);
  }
      
private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;
      ros::Publisher point_cloud_publisher_;
      laser_geometry::LaserProjection projector_;      
      
      ros::Subscriber sub_;  
      ros::Subscriber sub_2_;
}; 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "scan_all");
  /*ros::NodeHandle n;
  ros::Publisher scan_all_pub = n.advertise<sensor_msgs::LaserScan>("scan_all", 1000);
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe("scan", 1000, Callback);
  ros::Subscriber sub_2 = n.subscribe("scan_2", 1000, Callback);*/
  
  SubscribeAndPublish SAPObject;
  /*while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  ros::spin();
  return 0;
}



