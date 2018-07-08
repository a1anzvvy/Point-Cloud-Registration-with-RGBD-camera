#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <iostream>

using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Keyboard_Input");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Char>("/Keyboard_Input", 100);
  ros::Rate loop_rate(10);
  
  while(ros::ok())
	{
		std_msgs::Char msg;
		char a;
		cin >> a;
		printf("Sending Character : %c\n", a);
		msg.data = a;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
 return 0;
}