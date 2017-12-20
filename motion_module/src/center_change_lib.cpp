/*
 * center_change_lib.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: robotemperor
 */
#include "motion_module/center_change_lib.h"

using namespace diana_motion;

CenterChange::CenterChange()
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
CenterChange::~CenterChange()
{}

void CenterChange::parseMotionData(std::string turn_type, std::string change_type)
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
			}
			break;
			case 1:
			{
				for(int i=0; i<3; i++)
				{
					right_end_point_value_center[0][i] = it->second[i].as<double>();
					right_end_point_value_center[1][i] = it->second[i+6].as<double>();
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
void CenterChange::calculateStepEndPointValue(double desired_value, double step_value, std::string change_type)
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
		for(int i=0; i<3; i++)
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
		for(int i=0; i<3; i++)
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
		for(int i=0; i<3; i++)
		{
			step_end_point_value[0][i] = middle_end_point_value_center[0][i] + ((right_end_point_value_center[0][i] - middle_end_point_value_center[0][i])/step_value)*(desired_value*step_value);
			step_end_point_value[1][i] = middle_end_point_value_center[1][i] + ((right_end_point_value_center[1][i] - middle_end_point_value_center[1][i])/step_value)*(desired_value*step_value);
		}
	}
	else
		return;
}











