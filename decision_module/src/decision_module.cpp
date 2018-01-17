/*
 * decision_module.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#include "decision_module/decision_module.h"

void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	//ready_check = msg->data;
	//printf("RUN! %d \n", ready_check);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;

	ros::Subscriber ready_check_sub = ros_node.subscribe("/ready_check", 5, readyCheckMsgCallBack);

	center_change_pub = ros_node.advertise<diana_msgs::CenterChange>("/d",1);
	ros::Timer timer = ros_node.createTimer(ros::Duration(0.007), control_loop);

	center_change_pub.publish(center_change_msg);


    ros::Rate loop_rate(1);

	while(ros_node.ok())
	{
		ros::spinOnce();

		loop_rate.sleep();

	}

    return 0;
}


void control_loop(const ros::TimerEvent&)
{
	if(ready_check)
		decision_process.process();
	else
		return;


}



