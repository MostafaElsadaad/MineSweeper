#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <using_markers/nav.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>



bool firstmine = true;
class Listener
{
public:
float x=0;
float y=0;
float z=0;
int mine=0;
int dir=0;
//std_msgs::Int64 Direction;

void callback(const using_markers::nav::ConstPtr& msg);
void callbackk(const nav_msgs::Odometry::ConstPtr& M);
void callbackkk(const std_msgs::Int64::ConstPtr& msg);
};//End of class SubscribeAndPublish

  void Listener::callback(const using_markers::nav::ConstPtr& msg)
  {
  //  beginner_tutorials::motion m;
  mine = msg->mine;
  z    = msg->mine_z;
     
    //pub_.publish(m);
    
  }

  void Listener::callbackk(const nav_msgs::Odometry::ConstPtr& M)
  {
   

  x = M->pose.pose.position.x;
  y = M->pose.pose.position.y;
   //ROS_INFO("I heard: [%f]", x);

    //pub_.publish(m);
    
  }

  void Listener::callbackkk(const std_msgs::Int64::ConstPtr& msg)
  {
  //  beginner_tutorials::motion m;
  dir = msg->data;
     ROS_INFO("I heard: [%i]", dir);

    //pub_.publish(m);
    
  }


int main( int argc, char** argv )
{
	//NEW//
	float prevX = 0;
	float prevY = 0;
	///////
	
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(20);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  Listener listener;
   ros::Subscriber sub = n.subscribe<using_markers::nav>("mine_detection", 1, &Listener::callback, &listener);
   ros::Subscriber subb = n.subscribe<nav_msgs::Odometry>("odom", 1, &Listener::callbackk, &listener);
   ros::Subscriber subbb = n.subscribe<std_msgs::Int64>("directions", 1, &Listener::callbackkk, &listener);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
  ros::Rate loop_rate(1000);

// ROS_INFO("This is theta2f: %i", listener.mine);
    visualization_msgs::Marker marker;
	marker.mesh_resource = "file:///home/mostafa/Desktop/repaired_glued.dae";
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
  while (ros::ok())
  {
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
if(listener.dir == 2)
    marker.pose.position.x = listener.x - 0.77;
else if(listener.dir == 4)
    marker.pose.position.x = listener.x + 0.77;
else 
    marker.pose.position.x = listener.x;

if(listener.dir == 1)
    marker.pose.position.y = listener.y + 0.77;
else if(listener.dir == 3)
    marker.pose.position.y = listener.y - 0.77;
else
    marker.pose.position.y = listener.y;
    marker.pose.position.z = listener.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
 
    marker.lifetime = ros::Duration();

    // Publish the marker

    if (listener.mine == 1 && ( ( abs(prevX - listener.x) > 0.4) || ( abs(prevY - listener.y) > 0.4) || firstmine )){
firstmine = false;
	//NEW//
	if(listener.z == 1){
	marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
	}else{
    	marker.color.r = 1.0f;
    	marker.color.g = 0.0f;
    	marker.color.b = 0.0f;
	}
	///////

    ROS_INFO("MINE DETECTED !!at x=%f y=%f z=%f",listener.x,listener.y,listener.z);
    marker_pub.publish(marker);
    marker.id ++ ;
    prevX = listener.x;
    prevY = listener.y;
    }

 ros::spinOnce();
loop_rate.sleep();
  }

}
