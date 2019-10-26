#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "beginner_tutorials/motion.h"
int R ;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<beginner_tutorials::motion>("motion", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("joy", 1, &SubscribeAndPublish::callback, this);

  }

int mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


  void callback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    beginner_tutorials::motion m;

     m.RL = mapfloat(msg->axes[0] , -1, 1, 0, 1000);
     m.FB = mapfloat(msg->axes[1] , -1, 1, 0, 1000);
    // m.LT = mapfloat(msg->axes[3] , 1, -1, 0, 255);
     m.RT = mapfloat(msg->axes[4] , 1, -1, 0, 255);
     m.FB2 = mapfloat(msg->axes[3] , -1, 1, 0, 1000);
     ROS_INFO("ForwardBackward: [%i]", m.FB);
   ROS_INFO("Rightleft: [%i]", m.RL);
     m.A = msg->buttons[0];
     m.B = msg->buttons[1];
     m.Y = msg->buttons[2];
     m.X = msg->buttons[3];

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
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();


  return 0;
}
