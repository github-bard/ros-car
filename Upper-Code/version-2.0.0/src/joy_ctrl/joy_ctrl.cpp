#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <math.h>
using namespace std;
float pre_angle=0;

void callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
	geometry_msgs::Twist v;
    float angle_speed,delta_angle,current_angle,dt=0.1;
	current_angle=atan2(-Joy->axes[3],Joy->axes[4]);
	delta_angle=current_angle-pre_angle;
	angle_speed=delta_angle / dt * 1000;
	v.angular.z=angle_speed;
	v.linear.x=Joy->axes[1]*1000;
	v.linear.y=Joy->axes[0]*1000;
    pub.publish(v)
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"joy_ctrl");
	ros::NodeHandle n;
	ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	ros::Subscriber sub=n.subscribe<sensor_msgs::Joy>("joy",10,callback);
	ros::spin();
 	return 0;

