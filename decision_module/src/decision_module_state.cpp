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


	for(int i = 0; i <3 ; i++)
	{
		temp_flag0[i] = 0; // 0 :x 1 : y 2 : z;
		temp_flag1[i] = 0; // 0 :x 1 : y 2 : z;
		pre_temp_flag_0[i] = 0;
		pre_temp_flag_1[i] = 0;
		true_flag_0[i] = 0;
		true_flag_1[i] = 0;
		final_in_flag[i] = 0;

		flag0_on_robot_top[i] = 0;
		flag1_on_robot_top[i] = 0;
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

	top_view_flag_position.x = 0;
	top_view_flag_position.y = 0;
	top_view_flag_position.z = 0;

	temp_top_view_flag_position.x = 0;
	temp_top_view_flag_position.y = 0;
	temp_top_view_flag_position.z = 0;

	pre_top_view_flag_position.x = 0;
	pre_top_view_flag_position.y = 0;
	pre_top_view_flag_position.z = 0;

	top_view_robot_position.x = 0;
	top_view_robot_position.y = 0;
	top_view_robot_position.z = 0;

	direction_command = 0;
	initialize_time_count = 0;
	for(int i = 0; i < 5 ; i++)
	{
		flag_in_data[i][0] = 0;
		flag_in_data[i][1] = 0;
		flag_out_data[i][0] = 0;
		flag_out_data[i][1] = 0;
	}
	init_complete_check = false;
	flag_count = 0;

}
DecisionModule::~DecisionModule()
{
}

void DecisionModule::initialize(double sampling_time, double update_time)
{

	initialize_time_count = initialize_time_count + sampling_time;
	if(initialize_time_count < update_time)
	{
		flag_in_data[0][0] = temp_flag0[0];
		flag_in_data[0][1] = temp_flag0[1];
		flag_out_data[0][0] = temp_flag0[0];

		for(int flag_num = 1; flag_num < 5; flag_num++)
		{
			flag_in_data[flag_num][0]  = flag_in_data[flag_num-1][0] + 15;
			flag_in_data[flag_num][1]  = flag_in_data[flag_num-1][1];

			flag_out_data[flag_num][0] = flag_in_data[flag_num][0];
		}
		if(flag_in_data[0][1] > 0) // 우턴 부터 시작
		{
			flag_out_data[0][1] = temp_flag0[1] - 5;

			flag_out_data[1][1] = flag_in_data[1][1] + 5;
			flag_out_data[2][1] = flag_in_data[2][1] - 5;
			flag_out_data[3][1] = flag_in_data[3][1] + 5;
			flag_out_data[4][1] = flag_in_data[4][1] - 5;
		}
		if(flag_in_data[0][1] < 0) // 좌턴 부터 시작
		{
			flag_out_data[0][1] = temp_flag0[1] + 5;

			flag_out_data[1][1] = flag_in_data[1][1] - 5;
			flag_out_data[2][1] = flag_in_data[2][1] + 5;
			flag_out_data[3][1] = flag_in_data[3][1] - 5;
			flag_out_data[4][1] = flag_in_data[4][1] + 5;
		}
		init_complete_check = false;
	}
	else
	{
		init_complete_check = true;
		initialize_time_count = 0;
		return;
	}
	ROS_INFO(" Initialize First flag and Map !! \n");
}


void DecisionModule::process()
{
	classification_function(temp_flag0, temp_flag1);
	//top_view(true_flag_0);

	decision_function(true_flag_0, true_flag_1);

	//headFollowFlag(temp_flag0[0] , temp_flag0[1]);
	//top_view(temp_flag0);
}
void DecisionModule::classification_function(double flag0[3], double flag1[3])
{
	if(pre_temp_flag_0[0] == flag0[0] && pre_temp_flag_1[0] == flag1[0] && pre_temp_flag_0[1] == flag0[1] && pre_temp_flag_1[1] == flag1[1]) // 기문 둘다 안들어올 경우
	{
		true_flag_0[0] = flag_in_data[flag_sequence][0] - top_view_robot_position.x;
		true_flag_0[1] = flag_in_data[flag_sequence][1] - top_view_robot_position.y;
		true_flag_1[0] = flag_out_data[flag_sequence][0] - top_view_robot_position.x;
		true_flag_1[1] = flag_out_data[flag_sequence][1] - top_view_robot_position.y;

		printf("11111111111  X :: %f , Y :: %f \n", true_flag_0[0], true_flag_0[1]);
	}
	if(pre_temp_flag_0[0] != flag0[0] && pre_temp_flag_1[0] != flag1[0] && pre_temp_flag_0[1] != flag0[1] && pre_temp_flag_1[1] != flag1[1]) // 기문 둘다 들어올 경우
	{
		true_flag_0[0] = flag0[0];
		true_flag_0[1] = flag0[1];
		true_flag_1[0] = flag1[0];
		true_flag_1[1] = flag1[1];

		printf("2222222222222  X :: %f , Y :: %f \n", true_flag_0[0], true_flag_0[1]);
	}
	if(pre_temp_flag_0[0] != flag0[0] && pre_temp_flag_0[1] != flag0[1] && pre_temp_flag_1[0] == flag1[0] && pre_temp_flag_1[1] == flag1[1])// 한개의 기문만 들어올때
	{
		if(fabs(pre_temp_flag_1[1] - flag0[1] + top_view_robot_position.y) > 4 || fabs(pre_temp_flag_1[1] - flag0[1] + top_view_robot_position.y) < -4)// 바깥 기문 인식.
		{
			true_flag_1[0] = flag0[0];
			true_flag_1[1] = flag0[1];
			// assume
			true_flag_0[0] = flag0[0];
			if(flag_in_data[0][1] > 0)
			{
				if(flag_sequence/2 == 0) // 우턴 시작
					true_flag_0[1] = flag0[1] - 5; //y 값 추정
				else
					true_flag_0[1] = flag0[1] + 5;
			}
			if(flag_in_data[0][1] < 0)
			{
				if(flag_sequence/2 == 0) // 좌턴 시작
					true_flag_0[1] = flag0[1] + 5;
				else
					true_flag_0[1] = flag0[1] - 5;
			}
		}
		else //안쪽 기문
		{
			true_flag_0[0] = flag0[0];
			true_flag_0[1] = flag0[1];
			true_flag_1[0] = flag1[0];
			true_flag_1[1] = flag1[1];
		}

		printf("333333333333   X :: %f , Y :: %f \n", true_flag_0[0], true_flag_0[1]);
	}


	pre_temp_flag_0[0] = flag0[0];
	pre_temp_flag_0[1] = flag0[1];
	pre_temp_flag_1[0] = flag1[0];
	pre_temp_flag_1[1] = flag1[1];

}
void DecisionModule::decision_function(double flag0[3], double flag1[3])
{
	if(is_moving_check == false)
	{
		if(flag0[0]*flag0[1] < 0) // right turn
		{
			if(flag0[0] < right_x_detect_margin  && right_y_detect_margin_min < flag0[1] && right_y_detect_margin_max > flag0[1])
			{
				direction_command = -1;
				turn_direction = "right_turn";
			}
			else
				turn_direction = "center";

		}
		else if(flag0[0]*flag0[1] > 0) // left turn
		{
			if(flag0[0] < left_x_detect_margin  && left_y_detect_margin_min < flag0[1] && left_y_detect_margin_max > flag0[1])
			{
				direction_command = 1;
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

	if(flag_length > 2 && x > 1)
	{
		if(y > 0)
		{
			if(acos(x/flag_length) > 60*DEGREE2RADIAN)
				head_follow_flag_yaw_compensation = 60*DEGREE2RADIAN;
			else
				head_follow_flag_yaw_compensation = acos(x/flag_length);

		}

		if(y < 0)
		{
			if(acos(x/flag_length) < -60*DEGREE2RADIAN)
				head_follow_flag_yaw_compensation = -60*DEGREE2RADIAN;
			else
				head_follow_flag_yaw_compensation = -acos(x/flag_length);

		}
	}
	else
		head_follow_flag_yaw_compensation = 0;

	//head_follow_flag_yaw_compensation = filter_head->lowPassFilter(head_follow_flag_yaw_compensation, pre_head_follow_flag_yaw_compensation , 0.5, 0.008);

	//pre_head_follow_flag_yaw_compensation = head_follow_flag_yaw_compensation;
}
void DecisionModule::top_view(double flag_position[3])
{
	if(fabs(sqrt(pow(pre_top_view_flag_position.x,2) + pow(pre_top_view_flag_position.y,2)) - sqrt(pow(flag_position[0],2) + pow(flag_position[1],2))) > 8)
	{
		flag_sequence ++;
	}
	else
	{
		flag_check = 0; // flag change
	}

	if(pre_flag_sequence != flag_sequence && flag_sequence != 0)
	{
		top_view_flag_position.x = pre_top_view_robot_position.x + flag_position[0]; // 고정된값
		top_view_flag_position.y = pre_top_view_robot_position.y + flag_position[1]; // 고정된값
		flag_check = 1; // flag change
	}


	top_view_robot_position.x = -flag_position[0] + top_view_flag_position.x;
	top_view_robot_position.y = -flag_position[1] + top_view_flag_position.y;

	pre_flag_sequence = flag_sequence;
	pre_top_view_flag_position.x = top_view_flag_position.x;
	pre_top_view_flag_position.y = top_view_flag_position.y;

	pre_top_view_robot_position.x = top_view_robot_position.x;
	pre_top_view_robot_position.y = top_view_robot_position.y;

	//printf("flag  ::  X:: %f    Y :: %f \n", top_view_flag_position.x, top_view_flag_position.y);
	//printf("flag  ::  X:: %f    Y :: %f \n", top_view_robot_position.x, top_view_robot_position.y);
	//printf("flag_sequence ::  %d \n", flag_sequence);
}





