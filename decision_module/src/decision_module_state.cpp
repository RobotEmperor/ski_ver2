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
	flag_direction = 0;
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
	for(int i = 0; i < 50 ; i++)
	{
		flag_in_data[i][0] = 0;
		flag_in_data[i][1] = 0;
		flag_out_data[i][0] = 0;
		flag_out_data[i][1] = 0;
	}
	init_complete_check = false;
	data_in_check_1 = false;
	data_in_check_2 = false;
	init_flag_check = false;
	pre_flag_top_veiw_x = 0;

	//neutral check
	neutral_check = false;

	x_detect_margin = 0;
	y_detect_margin_min = 0;
	y_detect_margin_max = 0;

	initial_turn = 0;

	pre_turn_direction = "basic";

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
			flag_direction = -1;

			flag_out_data[1][1] = flag_in_data[1][1] + 5;
			flag_out_data[2][1] = flag_in_data[2][1] - 5;
			flag_out_data[3][1] = flag_in_data[3][1] + 5;
			flag_out_data[4][1] = flag_in_data[4][1] - 5;
		}
		if(flag_in_data[0][1] < 0) // 좌턴 부터 시작
		{
			flag_out_data[0][1] = temp_flag0[1] + 5;
			flag_direction = 1;

			flag_out_data[1][1] = flag_in_data[1][1] - 5;
			flag_out_data[2][1] = flag_in_data[2][1] + 5;
			flag_out_data[3][1] = flag_in_data[3][1] - 5;
			flag_out_data[4][1] = flag_in_data[4][1] + 5;
		}
		init_complete_check = false;

		pre_temp_flag_0[0] = temp_flag0[0];
		pre_temp_flag_0[1] = temp_flag0[1];
		pre_temp_flag_1[0] = temp_flag1[0];
		pre_temp_flag_1[1] = temp_flag1[1];
	}
	else
	{
		init_flag_check = true;
		init_complete_check = true;
		initialize_time_count = 0;
		return;
	}
	ROS_INFO(" Initialize First flag and Map !! \n");
}


void DecisionModule::process()
{
	classification_function(temp_flag0, temp_flag1, data_in_check_1, data_in_check_2);
	top_view(true_flag_0);

	decision_function(true_flag_0, true_flag_1);

	//headFollowFlag(temp_flag0[0] , temp_flag0[1]);
	//top_view(temp_flag0);
}
void DecisionModule::classification_function(double flag0[3], double flag1[3], bool check_1, bool check_2)
{
	//if(pre_temp_flag_0[0] == flag0[0] && pre_temp_flag_1[0] == flag1[0] && pre_temp_flag_0[1] == flag0[1] && pre_temp_flag_1[1] == flag1[1] && check == false) // 기문 둘다 안들어올 경우
	if(check_1 == false &&  check_2 == false)
	{
		/*		true_flag_0[0] = flag_in_data[flag_sequence][0];
		true_flag_0[1] = flag_in_data[flag_sequence][1];
		true_flag_1[0] = flag_out_data[flag_sequence][0];
		true_flag_1[1] = flag_out_data[flag_sequence][1];*/

		//printf("%d ::  %f \n\n", flag_sequence, flag_in_data[flag_sequence][0]);
	}
	if(check_1 == true &&  check_2 == true) // 기문 둘다 들어올 경우
	{
		true_flag_0[0] = flag0[0];
		true_flag_0[1] = flag0[1];
		true_flag_1[0] = flag1[0];
		true_flag_1[1] = flag1[1];

		printf("Two Flag Data  X :: %f , Y :: %f \n", true_flag_0[0], true_flag_0[1]);
	}
	if(check_1 == true  ||  check_2 == true)// 한개의 기문만 들어올때
	{
		/*if(flag_sequence == 0) //안쪽 기문
		{*/
		true_flag_0[0] = flag0[0];
		true_flag_0[1] = flag0[1];

		/*		true_flag_1[0] = flag0[0];

		if(flag_in_data[0][1] > 0)
		{
			if(flag_sequence/2 == 0) // 우턴 시작
				true_flag_1[1] = flag0[1] - 5; //y 값 추정
			else
				true_flag_1[1] = flag0[1] + 5;
		}
		if(flag_in_data[0][1] < 0)
		{
			if(flag_sequence/2 == 0) // 좌턴 시작
				true_flag_1[1] = flag0[1] + 5;
			else
				true_flag_1[1] = flag0[1] - 5;
		}
		printf("in! flag!!! \n ");*/
		//}
		/*if(flag_sequence > 0)
		{
			if(fabs(flag_in_data[flag_sequence-1][1] - (flag0[1] + top_view_robot_position.y)) > 4)// 바깥 기문 인식.
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
				printf("out! flag!!! \n ");
			}
			else
			{
				true_flag_0[0] = flag0[0];
				true_flag_0[1] = flag0[1];

				true_flag_1[0] = flag0[0];

				if(flag_in_data[0][1] > 0)
				{
					if(flag_sequence/2 == 0) // 우턴 시작
						true_flag_1[1] = flag0[1] - 5; //y 값 추정
					else
						true_flag_1[1] = flag0[1] + 5;
				}
				if(flag_in_data[0][1] < 0)
				{
					if(flag_sequence/2 == 0) // 좌턴 시작
						true_flag_1[1] = flag0[1] + 5;
					else
						true_flag_1[1] = flag0[1] - 5;
				}
				printf("in! flag!!! \n ");
			}
		}*/

		flag_in_data[flag_sequence][0]  =  true_flag_0[0];
		flag_in_data[flag_sequence][1]  =  true_flag_0[1];
		flag_out_data[flag_sequence][0] =  true_flag_1[0];
		flag_out_data[flag_sequence][1] =  true_flag_1[1];

		printf("One Flag Data   X :: %f , Y :: %f \n", true_flag_0[0], true_flag_0[1]);
	}

	if(pre_flag_sequence != flag_sequence && flag_sequence != 0)
	{
		flag_direction = -flag_direction;
		flag_in_data[flag_sequence][0]  = true_flag_0[0];
		flag_in_data[flag_sequence][1]  = true_flag_0[1];
		flag_out_data[flag_sequence][0] = true_flag_1[0];
		flag_out_data[flag_sequence][1] = true_flag_1[1];

		printf("Flag Changed");
	}
	pre_temp_flag_0[0] = true_flag_0[0];
	pre_temp_flag_0[1] = true_flag_0[1];
	pre_temp_flag_1[0] = true_flag_1[0];
	pre_temp_flag_1[1] = true_flag_1[1];
}
void DecisionModule::decision_function(double flag0[3], double flag1[3])
{
	if(neutral_check == 1)
	{
		turn_direction = "center";
		return;
	}
	if(is_moving_check == false)
	{
		/*if(flag0[0]*flag0[1] < 0) // right turn
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
		}*/
		if(fabs(flag0[0]) < x_detect_margin  && y_detect_margin_min < fabs(flag0[1]) && y_detect_margin_max > fabs(flag0[1])) // dectect flag
		{
			if(flag_sequence == 0)
			{
				direction_command = initial_turn;
				if(initial_turn == -1)
					turn_direction = "right_turn";
				if(initial_turn == 1)
					turn_direction = "left_turn";

				pre_turn_direction = turn_direction;
			}
			else
			{
				if(!pre_turn_direction.compare("left_turn") || !pre_turn_direction.compare("right_turn"))
				{
					if(!pre_turn_direction.compare("left_turn"))
					{
						turn_direction = "right_turn";
					}
					if(!pre_turn_direction.compare("right_turn"))
					{
						turn_direction = "left_turn";
					}
					pre_turn_direction = turn_direction;
				}

				printf("command ::::  %s   \n",pre_turn_direction.c_str());
			}
		}
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

	x_detect_margin = doc["x"].as<double>();;
	y_detect_margin_min = doc["y_min"].as<double>();;
	y_detect_margin_max = doc["y_max"].as<double>();

	initial_turn = doc["initial_turn"].as<double>();
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
	if(flag_sequence == 0 && init_flag_check == true)
	{
		top_view_flag_position.x = flag_position[0]; // 고정된값
		top_view_flag_position.y = flag_position[1]; // 고정된값
		init_flag_check = false;

		pre_top_view_robot_position.x = top_view_flag_position.x;
		pre_top_view_robot_position.y = top_view_flag_position.y;
	}
	else
	{
		//flag_check = 0; // flag change
	}

	if(pre_flag_sequence != flag_sequence && flag_sequence != 0)
	{
		top_view_flag_position.x = pre_top_view_robot_position.x + flag_position[0]; // 고정된값
		top_view_flag_position.y = pre_top_view_robot_position.y + flag_position[1]; // 고정된값
		flag_check = 1; // flag change

		printf("Flag update :: X :: %f , X :: %f \n", top_view_flag_position.x, top_view_flag_position.y);
	}


	top_view_robot_position.x = -flag_position[0] + top_view_flag_position.x;
	top_view_robot_position.y = -flag_position[1] + top_view_flag_position.y;

	//	printf("ROBOT     :: X :: %f , Y :: %f \n", top_view_robot_position.x , top_view_robot_position.y);
	//	printf("Top View  :: X :: %f , Y :: %f \n", top_view_flag_position.x , top_view_flag_position.y);

	pre_flag_sequence = flag_sequence;
	pre_top_view_flag_position.x = top_view_flag_position.x;
	pre_top_view_flag_position.y = top_view_flag_position.y;

	pre_top_view_robot_position.x = top_view_robot_position.x;
	pre_top_view_robot_position.y = top_view_robot_position.y;

	//printf("flag  ::  X:: %f    Y :: %f \n", top_view_flag_position.x, top_view_flag_position.y);
	//printf("flag  ::  X:: %f    Y :: %f \n", top_view_robot_position.x, top_view_robot_position.y);
	//printf("flag_sequence ::  %d \n", flag_sequence);
}





