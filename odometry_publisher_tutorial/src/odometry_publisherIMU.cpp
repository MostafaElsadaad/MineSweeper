//***Save this as odometry.cpp***//
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
double pth;

ros::Time current_time, last_time;
double DistancePerCount = (3.14159265 * 0.0287)/2000; //the wheel diameter is 0.1m last calibrated = 0.0287 574\

//2.87m	1m
//1	1

//final odometric datas
double x= 9.5; 
double y= -10.8;
double v_left;//left motor speed
double v_right;//right motor speed
double vth;//angular velocity of robot
double deltaLeft;//no of ticks in left encoder since last update
double deltaRight;//no of ticks in right encoder since last update
double dt;
double delta_distance;//distance moved by robot since last update
float delta_th;//corresponging change in heading
double delta_x ;//corresponding change in x direction
double delta_y;//corresponding change in y direction
float tempth;
float accelerationX;
float accelerationY;
float accelerationZ;
geometry_msgs::Quaternion IMUraw;

double roll, pitch, yaw, referenceyaw;
double th = 3.14159265/2;
double thshift =  3.14159265/2;
bool firstyaw = true;
#define PI 3.14159265
#define TwoPI 6.28318531
int mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& obj){

IMUraw = obj->orientation;
tf::Quaternion q(IMUraw.x,IMUraw.y,IMUraw.z,IMUraw.w);
tf::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
if(firstyaw) { referenceyaw = yaw; firstyaw = false; }
th =  yaw - referenceyaw + thshift;

}


void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
 current_time = ros::Time::now();


 deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
 deltaRight = ticks->y - _PreviousRightEncoderCounts;
 dt = (current_time - last_time).toSec();
 v_left = deltaLeft * DistancePerCount/dt;
 v_right = deltaRight * DistancePerCount/dt;
 delta_distance=0.5*(double)(deltaLeft+deltaRight)*DistancePerCount;
 //delta_th = (double)(deltaRight-deltaLeft)*DistancePerCount/0.754; //Distance between the two wheels is 0.36m 0.586 bestFactor yet 0.754
 delta_x = delta_distance*(double)cos(th);
 delta_y = delta_distance*(double)sin(th);
 x += delta_x;
 y += delta_y;
 th += delta_th;

if (th > PI)
 th -= TwoPI;
 else
 if ( th <= -PI)
 th += TwoPI;



_PreviousLeftEncoderCounts = ticks->x;
 _PreviousRightEncoderCounts = ticks->y;
 last_time = current_time;
}
int main(int argc, char **argv)
{
 ros::init(argc, argv, "odometry_publisher");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("pos", 100, WheelCallback);
 ros::Subscriber sub1 = n.subscribe("android/imu", 100, IMUCallback);
 ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
 tf::TransformBroadcaster odom_broadcaster;
 tf::Transform transform;
 ros::Rate r(40);
 while(n.ok()){
//since all odometry is 6DOF we'll need a quaternion created from yaw
   transform.setOrigin( tf::Vector3(0.0, 0.66, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "carrot1"));
 geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
//first, we'll publish the transform over tf
 geometry_msgs::TransformStamped odom_trans;
 odom_trans.header.stamp = current_time;
 odom_trans.header.frame_id = "odom";
 odom_trans.child_frame_id = "base_link";
 odom_trans.transform.translation.x = x;
 odom_trans.transform.translation.y = y;
 odom_trans.transform.translation.z = 0.0;
 odom_trans.transform.rotation = odom_quat;
 //send the transform
 odom_broadcaster.sendTransform(odom_trans);
 //next, we'll publish the odometry message over ROS
 nav_msgs::Odometry odom;
 odom.header.stamp = current_time;
 odom.header.frame_id = "odom";
 //set the position
 odom.pose.pose.position.x = x;
 odom.pose.pose.position.y = y;
 odom.pose.pose.position.z = 0.0;
 odom.pose.pose.orientation = odom_quat;
 //set the velocity
 odom.child_frame_id = "base_link";
 odom.twist.twist.linear.x=delta_x/dt;
 odom.twist.twist.linear.y=delta_y/dt;
 odom.twist.twist.angular.z = delta_th/dt;
 //publish the message
 odom_pub.publish(odom);
 last_time = current_time;
 ros::spinOnce();
 r.sleep();
 }
}
