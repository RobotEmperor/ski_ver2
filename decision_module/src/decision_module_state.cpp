/*
 * decision_module_state.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */
#include "decision_module/decision_module_state.h"

using namespace decision_module;

DecisionModule::DecisionModule()
{
	gazebo_check = false;
}
DecisionModule::~DecisionModule()
{
}

void DecisionModule::initialize()
{
	ROS_INFO("< -------  Initialize Module : Decision Module  !!  ------->");
}


void DecisionModule::process()
{
	printf("RUN! \n");
}



