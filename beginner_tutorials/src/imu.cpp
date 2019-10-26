#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Imu.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::Imu>("imu", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/android/imu", 1, &SubscribeAndPublish::callback, this);

  }



  void callback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    sensor_msgs::Imu m;
    m =*msg;
    m.header.frame_id = "imu";


    pub_.publish(m);
    
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publishs");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();


  return 0;
}
