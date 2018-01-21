/*
 * center_change_lib_leg.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: robotemperor
 */
#include "decision_module/center_change_lib_leg.h"

using namespace diana_motion_leg;

CenterChangeLeg::CenterChangeLeg()
{
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/turn/initialize.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load LEG initialize yaml file!");
		return;
	}
	// motion data load initialize//
	YAML::Node pose_node = doc["motion_init"];// YAML 에 string "motion"을 읽어온다.
	//	 motion data load //
	for (YAML::iterator it = pose_node.begin(); it != pose_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		switch(standard)
		{
		case 0:
		{
			for(int i=0; i<3; i++)
			{
				middle_end_point_value_center[0][i] = it->second[i].as<double>();
				middle_end_point_value_center[1][i] = it->second[i+6].as<double>();
			}
			for(int i=3; i<6; i++)
			{
				middle_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
				middle_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
			}
		}
		break;
		case 1:
		{
			for(int i=0; i<3; i++)
			{
				right_end_point_value_center[0][i] = it->second[i].as<double>();
				right_end_point_value_center[1][i] = it->second[i+6].as<double>();
			}
			for(int i=3; i<6; i++)
			{
				right_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
				right_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
			}
		}
		break;
		case -1:
		{
			for(int i=0; i<3; i++)
			{
				left_end_point_value_center[0][i] = it->second[i].as<double>();
				left_end_point_value_center[1][i] = it->second[i+6].as<double>();
			}
			for(int i=3; i<6; i++)
			{
				left_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
				left_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
			}
		}
		break;
		}
	}

	for(int i=0; i<6; i++)
	{
		step_end_point_value[0][i] = middle_end_point_value_center[0][i];
		step_end_point_value[1][i] = middle_end_point_value_center[1][i];
	}


}
CenterChangeLeg::~CenterChangeLeg()
{}
void CenterChangeLeg::parseMotionData(std::string turn_type, std::string change_type)
{
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/turn/"+ turn_type + ".yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.
	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load LEG parse data yaml file!");
		return;
	}
	// motion data load initialize//
	YAML::Node pose_node = doc["motion_"+ change_type];// YAML 에 string "motion"을 읽어온다.
	//	 motion data load //
	if(!change_type.compare("center_change")) // 문자열이 같을때 0
	{
		for (YAML::iterator it = pose_node.begin(); it != pose_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
		{
			int standard;
			// 한 줄에서 int 와 double 을 분리한다.
			standard = it->first.as<int>();
			switch(standard)
			{
			case 0:
			{
				for(int i=0; i<3; i++)
				{
					middle_end_point_value_center[0][i] = it->second[i].as<double>();
					middle_end_point_value_center[1][i] = it->second[i+6].as<double>();
				}

				for(int i=3; i<6; i++)
				{
					middle_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					middle_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
				}
			}
			break;
			case 1:
			{
				for(int i=0; i<3; i++)
				{
					right_end_point_value_center[0][i] = it->second[i].as<double>();
					right_end_point_value_center[1][i] = it->second[i+6].as<double>();

				}
				for(int i=3; i<6; i++)
				{
					right_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					right_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
				}
			}
			break;
			case -1:
			{
				for(int i=0; i<3; i++)
				{
					left_end_point_value_center[0][i] = it->second[i].as<double>();
					left_end_point_value_center[1][i] = it->second[i+6].as<double>();
				}
				for(int i=3; i<6; i++)
				{
					left_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					left_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;


				}
			}
			break;
			}
		}
	}
	if(!change_type.compare("edge_change"))
	{
		for (YAML::iterator it = pose_node.begin(); it != pose_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
		{
			int standard;
			// 한 줄에서 int 와 double 을 분리한다.
			standard = it->first.as<int>();
			switch(standard)
			{
			case 0:
			{
				for(int i=3; i<6; i++)
				{
					middle_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					middle_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
				}
			}
			break;
			case 1:
			{
				for(int i=3; i<6; i++)
				{
					right_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					right_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
				}
			}
			break;
			case -1:
			{
				for(int i=3; i<6; i++)
				{
					left_end_point_value_center[0][i] = it->second[i].as<double>()*DEGREE2RADIAN;
					left_end_point_value_center[1][i] = it->second[i+6].as<double>()*DEGREE2RADIAN;
				}
			}
			break;
			}
		}
	}
	else
		return;

}
void CenterChangeLeg::calculateStepEndPointValue(double desired_value, double step_value, std::string change_type)
{
	if(desired_value == 0)
	{
		if(!change_type.compare("edge_change"))
		{
			for(int i=3; i<6; i++)
			{
				step_end_point_value[0][i] = middle_end_point_value_center[0][i];
				step_end_point_value[1][i] = middle_end_point_value_center[1][i];
			}
			return;
		}
		for(int i=0; i<6; i++)
		{
			step_end_point_value[0][i] = middle_end_point_value_center[0][i];
			step_end_point_value[1][i] = middle_end_point_value_center[1][i];
		}
	}
	else if (desired_value < 0 && desired_value >= -1)
	{
		if(!change_type.compare("edge_change"))
		{
			for(int i=3; i<6; i++)
			{
				step_end_point_value[0][i] = middle_end_point_value_center[0][i] + ((left_end_point_value_center[0][i] - middle_end_point_value_center[0][i])/step_value)*(-desired_value*step_value);
				step_end_point_value[1][i] = middle_end_point_value_center[1][i] + ((left_end_point_value_center[1][i] - middle_end_point_value_center[1][i])/step_value)*(-desired_value*step_value);
			}
			return;
		}
		for(int i=0; i<6; i++)
		{
			step_end_point_value[0][i] = middle_end_point_value_center[0][i] + ((left_end_point_value_center[0][i] - middle_end_point_value_center[0][i])/step_value)*(-desired_value*step_value);
			step_end_point_value[1][i] = middle_end_point_value_center[1][i] + ((left_end_point_value_center[1][i] - middle_end_point_value_center[1][i])/step_value)*(-desired_value*step_value);
		}
	}
	else if (desired_value > 0 && desired_value <= 1)
	{
		if(!change_type.compare("edge_change"))
		{
			for(int i=3; i<6; i++)
			{
				step_end_point_value[0][i] = middle_end_point_value_center[0][i] + ((right_end_point_value_center[0][i] - middle_end_point_value_center[0][i])/step_value)*(desired_value*step_value);
				step_end_point_value[1][i] = middle_end_point_value_center[1][i] + ((right_end_point_value_center[1][i] - middle_end_point_value_center[1][i])/step_value)*(desired_value*step_value);
			}
			return;
		}
		for(int i=0; i<6; i++)
		{
			step_end_point_value[0][i] = middle_end_point_value_center[0][i] + ((right_end_point_value_center[0][i] - middle_end_point_value_center[0][i])/step_value)*(desired_value*step_value);
			step_end_point_value[1][i] = middle_end_point_value_center[1][i] + ((right_end_point_value_center[1][i] - middle_end_point_value_center[1][i])/step_value)*(desired_value*step_value);
		}
	}
	else
		return;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MotionChange::MotionChange()
{
	for(int i = 0; i<11; i++)
	{
		motion_time[i] = 0;
	}
	for(int i = 0; i<12; i++)
	{
		motion_center_position[i] = 0;
		motion_waist_center_position[i] = 0;
		motion_arm_center_position[i] = 0;
	}
	for(int m=0; m<10 ; m++)
	{
		for(int var = 0; var<12; var++)
		{
			motion_left_position[m][var] = 0;
			motion_right_position[m][var] = 0;

		}
		for(int var = 0; var<2; var++)
		{
			motion_waist_left_position[m][var] = 0;
			motion_waist_right_position[m][var] = 0;

		}
		for(int var = 0; var<6; var++)
		{
			motion_arm_left_position[m][var] = 0;
			motion_arm_right_position[m][var] = 0;
		}
	}

	for(int m=0; m<11 ; m++)
	{
		for(int var = 0; var<12; var++)
		{
			motion_init_leg_left_vel[m][var] = 0;
			motion_final_leg_left_vel[m][var] = 0;

			motion_init_leg_right_vel[m][var] = 0;
			motion_final_leg_right_vel[m][var] = 0;

			motion_left_vel[m][var] = 0;
			motion_right_vel[m][var] = 0;
		}
		for(int var = 0; var<2; var++)
		{
			motion_init_waist_left_vel[m][var];
			motion_final_waist_left_vel[m][var];

			motion_init_waist_right_vel[m][var];
			motion_final_waist_right_vel[m][var];

			motion_waist_left_vel[m][var] = 0;
			motion_waist_right_vel[m][var] = 0;


		}
		for(int var = 0; var<6; var++)
		{
			motion_init_arm_left_vel[m][var] = 0;
			motion_final_arm_left_vel[m][var] = 0;

			motion_init_arm_right_vel[m][var] = 0;
			motion_final_arm_right_vel[m][var] = 0;

			motion_arm_left_vel[m][var] = 0;
			motion_arm_right_vel[m][var] = 0;


		}
	}
	parseMotionData("pflug_bogen_ver2");
}
MotionChange::~MotionChange()
{}

void MotionChange::parseMotionData(std::string trun_type)
{
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/turn/"+ trun_type +".yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.
	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load LEG parse data yaml file!");
		return;
	}
	// motion data load initialize//
	motion_time[0] = doc["motion_start_time_0"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[1] = doc["motion_time_1"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[2] = doc["motion_time_2"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[3] = doc["motion_time_3"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[4] = doc["motion_time_4"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[5] = doc["motion_time_5"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[6] = doc["motion_time_6"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[7] = doc["motion_time_7"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[8] = doc["motion_time_8"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[9] = doc["motion_back_9"].as<double>();// YAML 에 string "motion"을 읽어온다.
	motion_time[10] = doc["motion_back_10"].as<double>();// YAML 에 string "motion"을 읽어온다.


	YAML::Node motion_center_node = doc["motion_center"];

	for (YAML::iterator it = motion_center_node.begin(); it != motion_center_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		for(int i = 0; i < 3; i++)
		{
			motion_center_position[i] = it->second[i].as<double>();
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
		for(int i = 3; i < 6; i++)
		{
			motion_center_position[i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
		for(int i = 6; i < 9; i++)
		{
			motion_center_position[i] = it->second[i].as<double>();
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
		for(int i = 9; i < 12; i++)
		{
			motion_center_position[i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
	}
	YAML::Node motion_waist_center_node = doc["motion_waist_center"];

	for (YAML::iterator it = motion_waist_center_node.begin(); it != motion_waist_center_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		for(int i = 0; i < 2; i++)
		{
			motion_waist_center_position[i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
	}

	YAML::Node motion_arm_center_node = doc["motion_arm_center"];
	for (YAML::iterator it = motion_arm_center_node.begin(); it != motion_arm_center_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		for(int i = 0; i < 6; i++)
		{
			motion_arm_center_position[i] = it->second[i].as<double>();
			//printf("motion_center ::  %d    ::    %f\n", i, motion_center_position[i]);
		}
	}

	YAML::Node motion_left_node = doc["motion_leg_left"];

	for (YAML::iterator it = motion_left_node.begin(); it != motion_left_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 6; i++)
		{
			motion_left_position[standard][i] = it->second[i].as<double>();
			//printf("motion_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_left_position[standard][i]);
		}
		for(int i = 3; i < 6; i++)
		{
			motion_left_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_left_position[standard][i]);
		}

		for(int i = 6; i < 9; i++)
		{
			motion_left_position[standard][i] = it->second[i].as<double>();
			//printf("motion_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_left_position[standard][i]);
		}
		for(int i = 9; i < 12; i++)
		{
			motion_left_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_left_position[standard][i]);
		}
	}

	YAML::Node motion_right_node = doc["motion_leg_right"];

	for (YAML::iterator it = motion_right_node.begin(); it != motion_right_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 3; i++)
		{
			motion_right_position[standard][i] = it->second[i].as<double>();
			//printf("motion_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_right_position[standard][i]);
		}
		for(int i = 3; i < 6; i++)
		{
			motion_right_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_right_position[standard][i]);
		}

		for(int i = 6; i < 9; i++)
		{
			motion_right_position[standard][i] = it->second[i].as<double>();
			//printf("motion_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_right_position[standard][i]);
		}
		for(int i = 9; i < 12; i++)
		{
			motion_right_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_right_position[standard][i]);
		}

	}

	YAML::Node motion_waist_left_node = doc["motion_waist_left"];

	for (YAML::iterator it = motion_waist_left_node.begin(); it != motion_waist_left_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 2; i++)
		{
			motion_waist_left_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_waist_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_waist_left_position[standard][i]);
		}
	}

	YAML::Node motion_waist_right_node = doc["motion_waist_right"];

	for (YAML::iterator it = motion_waist_right_node.begin(); it != motion_waist_right_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 2; i++)
		{
			motion_waist_right_position[standard][i] = it->second[i].as<double>()*DEGREE2RADIAN;
			//printf("motion_waist_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_waist_right_position[standard][i]);
		}
	}

	YAML::Node motion_arm_left_node = doc["motion_arm_left"];

	for (YAML::iterator it = motion_arm_left_node.begin(); it != motion_arm_left_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 6; i++)
		{
			motion_arm_left_position[standard][i] = it->second[i].as<double>();
			//printf("motion_arm_left ::  %d    ::    %d   ::    %f\n", standard, i ,motion_arm_left_position[standard][i]);
		}
	}

	YAML::Node motion_arm_right_node = doc["motion_arm_right"];

	for (YAML::iterator it = motion_arm_right_node.begin(); it != motion_arm_right_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int standard;
		// 한 줄에서 int 와 double 을 분리한다.
		standard = it->first.as<int>();
		for(int i = 0; i < 6; i++)
		{
			motion_arm_right_position[standard][i] = it->second[i].as<double>();
			//printf("motion_arm_right ::  %d    ::    %d   ::    %f\n", standard, i ,motion_arm_right_position[standard][i]);
		}
	}
}
double MotionChange::calculate_velocity(double first_motion,double second_motion, double interval_time)
{
	return (second_motion - first_motion)/interval_time;
}
void MotionChange::calculate_init_final_velocity(int motion_number)
{
	for(int var = 0; var <12 ; var++)
	{
		motion_left_vel[0][var]  = calculate_velocity(motion_center_position[var],motion_left_position[0][var], motion_time[0]); // 처음
		motion_right_vel[0][var] = calculate_velocity(motion_center_position[var],motion_right_position[0][var], motion_time[0]); // 처음

		motion_init_leg_left_vel[0][var] = 0;
		motion_final_leg_left_vel[motion_number][var] = 0;

		motion_init_leg_right_vel[0][var] = 0;
		motion_final_leg_right_vel[motion_number][var] = 0;
	}

	for(int var = 0; var <2 ; var++)
	{
		motion_waist_left_vel[0][var]  = calculate_velocity(motion_waist_center_position[var],motion_waist_left_position[0][var], motion_time[0]); // 처음
		motion_waist_right_vel[0][var] = calculate_velocity(motion_waist_center_position[var],motion_waist_right_position[0][var], motion_time[0]); // 처음

		motion_init_waist_left_vel[0][var] = 0;
		motion_final_waist_left_vel[motion_number][var] = 0;

		motion_init_waist_right_vel[0][var] = 0;
		motion_final_waist_right_vel[motion_number][var] = 0;

	}

	for(int var = 0; var <6 ; var++)
	{
		motion_arm_left_vel[0][var]  = calculate_velocity(motion_arm_center_position[var],motion_arm_left_position[0][var], motion_time[0]); // 처음
		motion_arm_right_vel[0][var] = calculate_velocity(motion_arm_center_position[var],motion_arm_right_position[0][var], motion_time[0]); // 처음

		motion_init_arm_left_vel[0][var] = 0;
		motion_final_arm_left_vel[motion_number][var] = 0;

		motion_init_arm_right_vel[0][var] = 0;
		motion_final_arm_right_vel[motion_number][var] = 0;

	}
	// 속도 계산
	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		for(int var = 0; var<12 ; var++)
		{
			motion_left_vel[motion_num][var]  = calculate_velocity(motion_left_position[motion_num-1][var],motion_left_position[motion_num][var], motion_time[motion_num]);// 12
			motion_right_vel[motion_num][var] = calculate_velocity(motion_right_position[motion_num-1][var],motion_right_position[motion_num][var], motion_time[motion_num]);// 12
		}

		for(int var = 0; var <2 ; var++)
		{
			motion_waist_left_vel[motion_num][var]  = calculate_velocity(motion_waist_center_position[var],motion_waist_left_position[motion_num][var], motion_time[motion_num]); // 처음
			motion_waist_right_vel[motion_num][var] = calculate_velocity(motion_waist_center_position[var],motion_waist_right_position[motion_num][var], motion_time[motion_num]); // 처음
		}
		for(int var = 0; var <6 ; var++)
		{
			motion_arm_left_vel[motion_num][var]  = calculate_velocity(motion_arm_center_position[var],motion_arm_left_position[motion_num][var], motion_time[motion_num]); // 처음
			motion_arm_right_vel[motion_num][var] = calculate_velocity(motion_arm_center_position[var],motion_arm_right_position[motion_num][var], motion_time[motion_num]); // 처음
		}
	}
	for(int var = 0; var<12 ; var++)
	{
		motion_left_vel[motion_number][var] = calculate_velocity(motion_left_position[motion_number-1][var], motion_center_position[var], motion_time[motion_number]);// 12
		motion_right_vel[motion_number][var] = calculate_velocity(motion_right_position[motion_number-1][var], motion_center_position[var], motion_time[motion_number]);// 12
	}
	for(int var = 0; var <2 ; var++)
	{
		motion_waist_left_vel[motion_number][var]  = calculate_velocity(motion_waist_center_position[var],motion_waist_center_position[var], motion_time[motion_number]); // 처음
		motion_waist_right_vel[motion_number][var] = calculate_velocity(motion_waist_center_position[var],motion_waist_center_position[var], motion_time[motion_number]); // 처음
	}
	for(int var = 0; var <6 ; var++)
	{
		motion_arm_left_vel[motion_number][var]  = calculate_velocity(motion_arm_center_position[var],motion_waist_center_position[var], motion_time[motion_number]); // 처음
		motion_arm_right_vel[motion_number][var] = calculate_velocity(motion_arm_center_position[var],motion_waist_center_position[var], motion_time[motion_number]); // 처음
	}

	///////////////////////////////////////////////////////////////////////////////// 초기

	for(int var = 0; var < 12; var++)
	{
		if(motion_center_position[var] == motion_left_position[0][var])
			motion_final_leg_left_vel[0][var] = 0;
		else
			motion_final_leg_left_vel[0][var] = calculate_motion_velocity(motion_left_vel[0][var], motion_left_vel[1][var]);

		if(motion_center_position[var] == motion_right_position[0][var])
			motion_final_leg_right_vel[0][var] = 0;
		else
			motion_final_leg_right_vel[0][var] = calculate_motion_velocity(motion_right_vel[0][var], motion_right_vel[1][var]);
	}

	for(int var = 0; var < 2; var++)
	{
		if(motion_waist_center_position[var] == motion_waist_left_position[0][var])
			motion_final_waist_left_vel[0][var] = 0;
		else
			motion_final_waist_left_vel[0][var] = calculate_motion_velocity(motion_waist_left_vel[0][var], motion_waist_left_vel[1][var]);

		if(motion_waist_center_position[var] == motion_waist_right_position[0][var])
			motion_final_waist_right_vel[0][var] = 0;
		else
			motion_final_waist_right_vel[0][var] = calculate_motion_velocity(motion_waist_right_vel[0][var], motion_waist_right_vel[1][var]);
	}

	for(int var = 0; var < 6; var++)
	{
		if(motion_arm_center_position[var] == motion_arm_left_position[0][var])
			motion_final_arm_left_vel[0][var] = 0;
		else
			motion_final_arm_left_vel[0][var] = calculate_motion_velocity(motion_arm_left_vel[0][var], motion_arm_left_vel[1][var]);

		if(motion_arm_center_position[var] == motion_arm_right_position[0][var])
			motion_final_arm_right_vel[0][var] = 0;
		else
			motion_final_arm_right_vel[0][var] = calculate_motion_velocity(motion_arm_right_vel[0][var], motion_arm_right_vel[1][var]);
	}

	////////////////////////////////////////////////////////////////////////////////////////////// 모션개수에 따른 파이널 속도 계산
	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		for(int var = 0; var<12 ; var++)
		{
			if(motion_left_position[motion_num-1][var] == motion_left_position[motion_num][var])
				motion_final_leg_left_vel[motion_num][var] = 0;
			else
				motion_final_leg_left_vel[motion_num][var] = calculate_motion_velocity(motion_left_vel[motion_num][var], motion_left_vel[motion_num+1][var]);

			if(motion_right_position[motion_num-1][var] == motion_right_position[motion_num][var])
				motion_final_leg_right_vel[motion_num][var] = 0;
			else
				motion_final_leg_right_vel[motion_num][var] = calculate_motion_velocity(motion_right_vel[motion_num][var], motion_right_vel[motion_num+1][var]);
		}
		for(int var = 0; var<2 ; var++)
		{
			if(motion_waist_left_position[motion_num-1][var] == motion_waist_left_position[motion_num][var])
				motion_final_waist_left_vel[motion_num][var] = 0;
			else
				motion_final_waist_left_vel[motion_num][var] = calculate_motion_velocity(motion_waist_left_vel[motion_num][var], motion_waist_left_vel[motion_num+1][var]);

			if(motion_waist_right_position[motion_num-1][var] == motion_waist_right_position[motion_num][var])
				motion_final_waist_right_vel[motion_num][var] = 0;
			else
				motion_final_waist_right_vel[motion_num][var] = calculate_motion_velocity(motion_waist_right_vel[motion_num][var], motion_waist_right_vel[motion_num+1][var]);
		}
		for(int var = 0; var<6 ; var++)
		{
			if(motion_arm_left_position[motion_num-1][var] == motion_arm_left_position[motion_num][var])
				motion_final_arm_left_vel[motion_num][var] = 0;
			else
				motion_final_arm_left_vel[motion_num][var] = calculate_motion_velocity(motion_arm_left_vel[motion_num][var], motion_arm_left_vel[motion_num+1][var]);

			if(motion_arm_right_position[motion_num-1][var] == motion_arm_right_position[motion_num][var])
				motion_final_arm_right_vel[motion_num][var] = 0;
			else
				motion_final_arm_right_vel[motion_num][var] = calculate_motion_velocity(motion_arm_right_vel[motion_num][var], motion_arm_right_vel[motion_num+1][var]);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		for(int var = 0; var<12 ; var++)
		{
			motion_init_leg_left_vel[motion_num][var] = motion_final_leg_left_vel[motion_num-1][var];
			motion_init_leg_right_vel[motion_num][var] = motion_final_leg_right_vel[motion_num-1][var];
		}

		for(int var = 0; var<2 ; var++)
		{
			motion_init_waist_left_vel[motion_num][var] = motion_final_waist_left_vel[motion_num-1][var];
			motion_init_waist_right_vel[motion_num][var] = motion_final_waist_right_vel[motion_num-1][var];
		}

		for(int var = 0; var<6 ; var++)
		{
			motion_init_arm_left_vel[motion_num][var] = motion_final_arm_left_vel[motion_num-1][var];
			motion_init_arm_right_vel[motion_num][var] = motion_final_arm_right_vel[motion_num-1][var];
		}
	}

	for(int var = 0; var < 12; var++)
	{

		motion_init_leg_left_vel[motion_number][var] = motion_final_leg_left_vel[motion_number-1][var];
		motion_init_leg_right_vel[motion_number][var] = motion_final_leg_right_vel[motion_number-1][var];
	}
	for(int var = 0; var < 2; var++)
	{

		motion_init_waist_left_vel[motion_number][var] = motion_final_waist_left_vel[motion_number-1][var];
		motion_init_waist_right_vel[motion_number][var] = motion_final_waist_right_vel[motion_number-1][var];
	}
	for(int var = 0; var < 6; var++)
	{

		motion_init_arm_left_vel[motion_number][var] = motion_final_arm_left_vel[motion_number-1][var];
		motion_init_arm_right_vel[motion_number][var] = motion_final_arm_right_vel[motion_number-1][var];
	}
}
double MotionChange::calculate_motion_velocity(double first_v1, double second_v2)
{
	if(first_v1*second_v2 > 0)
		return second_v2;
	else
		return 0;
}






