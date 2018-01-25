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
	robot_position_on_flag_x = 0;
	robot_position_on_flag_y = 0;


	for(int i = 0; i <3 ; i++)
	{
		temp_flag0[i] = 0; // 0 :x 1 : y 2 : z;
		temp_flag1[i] = 0; // 0 :x 1 : y 2 : z;
		temp_flag2[i] = 0; // 0 :x 1 : y 2 : z;
		temp_flag3[i] = 0; // 0 :x 1 : y 2 : z;
	}

	is_moving_check = false;
	turn_direction = "center";

	left_x_detect_margin = 0;
	left_y_detect_margin_min = 0;
	left_y_detect_margin_max = 0;
	right_x_detect_margin = 0;
	right_y_detect_margin_min = 0;
	right_y_detect_margin_max = 0;


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
	decision_function(temp_flag0);
}

void DecisionModule::decision_function(double flag[3])
{
	if(is_moving_check == false)
	{
		if(flag[0]*flag[1] < 0) // right turn
		{
			if(flag[0] < right_x_detect_margin  && right_y_detect_margin_min < flag[1] && right_y_detect_margin_max > flag[1])
			{
				turn_direction = "right_turn";
			}
			else
				turn_direction = "center";

		}
		else if(flag[0]*flag[1] > 0) // left turn
		{
			if(flag[0] < left_x_detect_margin  && left_y_detect_margin_min < flag[1] && left_y_detect_margin_max > flag[1])
			{
				turn_direction = "left_turn";
			}
			else
				turn_direction = "center";
		}
		else
		{
			turn_direction = "center";
			return;
		}
	}
	else
	{
		return;
	}
}
void DecisionModule::parseMotionData()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/turn/time_variables.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.
	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	//margin load //
	left_x_detect_margin = doc["left_x"].as<double>();
	left_y_detect_margin_min = doc["left_y_min"].as<double>();
	left_y_detect_margin_max = doc["left_y_max"].as<double>();

	right_x_detect_margin = doc["right_x"].as<double>();
	right_y_detect_margin_min = doc["right_y_min"].as<double>();
	right_y_detect_margin_max = doc["right_y_max"].as<double>();
}





