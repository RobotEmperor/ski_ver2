/*
 * decision_module.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#include "decision_module/decision_module.h"

int main(int argc, char **argv)
{
	DecisionModule *decision_process;
	decision_process = new DecisionModule;

	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;
	ready_check_sub = ros_node.subscribe("/ready_check", 5, readyCheckMsgCallBack);
	ros::Timer timer = ros_node.createTimer(ros::Duration(0.008), control_loop);



	while(ros_node.ok())
	{
		ros::spinOnce();
	}

	return 0;
}

void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	ready_check = msg->data;
	printf("RUN! %d \n", ready_check);

}

void control_loop(const ros::TimerEvent&)
{
	if(ready_check)
		decision_process->process();
	else
		return;


}



