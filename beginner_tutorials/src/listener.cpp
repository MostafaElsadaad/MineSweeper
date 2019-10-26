#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
float x ;
float y ;
void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  ROS_INFO("I heard: [%f]", msg->axes[0]);


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1000, chatterCallback);

  ros::spin();
  
  return 0;
}

//Murder Mystery
