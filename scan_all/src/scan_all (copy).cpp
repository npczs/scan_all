
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

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
      
        //Topic you want to subscribe  
        sub_ = n_.subscribe("scan", 1000, &SubscribeAndPublish::callback, this);
        sub_2_= n_.subscribe("scan_2", 1000, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
  }  
      
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg)  
  {  
        pub_.publish(msg);  
  }
      
private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;  
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



