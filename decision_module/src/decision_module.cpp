/*
 * decision_module.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#include "decision_module/decision_module.h"
using namespace decision_module;

int main(int argc, char **argv)
{
	DecisionModule decision_process;
	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;
    ros::Timer timer = ros_node.createTimer(ros::Duration(0.008), control_loop);
    //center change
    decision_process.center_change_pub = ros_node.advertise<diana_msgs::CenterChange>("/diana/center_change",100);

	while(ros_node.ok())
	{

		ros::spinOnce();
	}

	return 0;

}
void control_loop(const ros::TimerEvent&)
{
/*	center_change_msg.change_type = "center_change";
	center_change_msg.center_change = 0.5;
	center_change_msg.time_change = 1;
	center_change_msg.time_change_waist = 1;
	center_change_pub.publish(center_change_msg);*/

	  printf("!!!!!!!!!!!");

}



