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

	}

	is_moving_check = false;
	turn_direction = "center";

	left_x_detect_margin = 0;
	left_y_detect_margin_min = 0;
	left_y_detect_margin_max = 0;
	right_x_detect_margin = 0;
	right_y_detect_margin_min = 0;
	right_y_detect_margin_max = 0;

	//head
	head_follow_flag_yaw_compensation = 0;
	pre_head_follow_flag_yaw_compensation = 0;

	filter_head = new control_function::Filter;

	pre_flag_sequence = 0;
	flag_sequence = 0;
	 flag_check = false;
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
	headFollowFlag(temp_flag0[0] , temp_flag0[1]);
	top_view(temp_flag0);
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
// head
void DecisionModule::headFollowFlag(double x , double y)
{
	double flag_length = 0;

	flag_length = sqrt(pow(x,2) + pow(y,2));

	if(flag_length > 1 && x > 0)
	{
		if(y > 0)
			head_follow_flag_yaw_compensation = acos(x/flag_length);
		if(y < 0)
			head_follow_flag_yaw_compensation = -acos(x/flag_length);
	}
	else
		head_follow_flag_yaw_compensation = 0;

	//head_follow_flag_yaw_compensation = filter_head->lowPassFilter(head_follow_flag_yaw_compensation, pre_head_follow_flag_yaw_compensation , 0, 0.008);

	pre_head_follow_flag_yaw_compensation = head_follow_flag_yaw_compensation;
}
void DecisionModule::top_view(double flag_position[3])
{
	if(filter_head->averageFilter(flag_position[0],50,-5,20) != 0 && filter_head->averageFilter(flag_position[1],50,-20,20) !=0)
	{
		top_view_position.x = filter_head->averageFilter(flag_position[0],50,-5,20);
		top_view_position.y = filter_head->averageFilter(flag_position[1],50,-20,20);

		if(fabs(sqrt(pow(pre_top_view_position.x,2) + pow(pre_top_view_position.y,2)) - sqrt(pow(top_view_position.x,2) + pow(top_view_position.y,2))) > 8)
		{
			flag_sequence ++;
		}
		else
		{
			flag_check = 0; // flag change
		}
		if(pre_flag_sequence != flag_sequence)
			flag_check = 1; // flag change
	}

	pre_flag_sequence = flag_sequence;

	pre_top_view_position.x = top_view_position.x;
	pre_top_view_position.y = top_view_position.y;

	printf("X:: %f    Y :: %f \n", top_view_position.x, top_view_position.y);
	printf("flag_sequence ::  %d \n", flag_sequence);
}





