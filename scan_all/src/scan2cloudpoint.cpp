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

class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish()  
  {  
        //Topic you want to publish   
        point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud2> ("/cloud", 1000, false);
        //Topic you want to subscribe  
        sub_ = n_.subscribe("scan", 1000, &SubscribeAndPublish::callback, this);
       // sub_2_= n_.subscribe("scan_2", 1000, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
  }  
      
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg)  
  {  
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
     
}; 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sacn2cloudpoint");
  
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}

