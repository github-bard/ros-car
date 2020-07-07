#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include<math.h>
using namespace std;
 float pre_angle=0;
class Teleop
{
	public:
   	Teleop();
 
	private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);
    ros::NodeHandle n; //实例化节点
    ros::Subscriber sub ;
    ros::Publisher pub ;
    float angle_speed,delta_angle,current_angle,dt;//我们控制乌龟的速度，是通过这两个变量调整
};
 
Teleop::Teleop()
{   
 	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);//将速度发给乌龟 
	sub = n.subscribe<sensor_msgs::Joy>("joy",10,&Teleop::callback,this);	//订阅游戏手柄发来的数据}
}
void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)
{ 
	dt=100;
	geometry_msgs::Twist v;
	current_angle=atan2(-Joy->axes[4],Joy->axes[3]);
    delta_angle=current_angle-pre_angle;
    angle_speed=delta_angle / dt;
	pre_angle=current_angle;
    v.angular.z=angle_speed;
    v.linear.x=Joy->axes[1]/10;
    v.linear.y=Joy->axes[0]/10;
    pub.publish(v);
}

int main(int argc,char** argv)
{
 	ros::init(argc, argv, "joy_ctrl");
 	Teleop telelog; 
	ros::spin();
 	return 0;
}
