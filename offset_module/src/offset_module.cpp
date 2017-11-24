/*
 * base_module.cpp
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */
#include "offset_module/offset_module.h"
using namespace offset_module;

OffsetModule::OffsetModule()
:control_cycle_msec_(8) // 제어주기 8ms
{
	// variables initialize ////
	joint_select_ = 1;

	for(int i=0; i<30;i++)
	{
		read_joint_value_[i]= 0;
		offset_joint_value_[i]= 0;
	}

	offset_start_ = false;
	save_onoff_ =false;
	////////////////////////////

	enable_       = false;
	module_name_  = "offset_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////

	result_["head"]        = new robotis_framework::DynamixelState(); // joint 1
	result_["waist_roll"]  = new robotis_framework::DynamixelState(); // joint 10

	result_["l_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 13

	result_["l_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 17

	result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 14

	result_["r_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 18

	result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 22




	///////////////////////////
	running_ = false;
	gazebo_check = false;
}

OffsetModule::~OffsetModule()
{
	queue_thread_.join();
}
void OffsetModule::stop() {};
bool OffsetModule::isRunning(){return running_;};

void OffsetModule::queueThread()
{
	ros::NodeHandle    ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	// subsscriber definition ////
	joint_select_sub_ = ros_node.subscribe("/joint_select",100, &OffsetModule::joint_select_sub_function, this);
	change_joint_value_sub_ = ros_node.subscribe("/change_joint_value",100, &OffsetModule::change_joint_value_sub_function, this);

	offset_joint_value_sub_ = ros_node.subscribe("/offset_joint_value",100, &OffsetModule::offset_joint_value_sub_function, this);
	save_onoff_sub_ = ros_node.subscribe("/save_onoff",100, &OffsetModule::save_onoff_sub_function, this);

	// publisher definition ////
	read_joint_value_srv = ros_node.advertiseService("/read_joint_value", &OffsetModule::read_joint_value_srv_function, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);


	while (ros_node.ok())
	{
		ros::spinOnce();
		callback_queue.callAvailable(duration);
		usleep(1000);
	}


}
void OffsetModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&OffsetModule::queueThread, this));

	new_count_ = 1;

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	ROS_INFO("< -------  Initialize Module : Offset Module !!  ------->");
}

void OffsetModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{

	if (enable_ == false)
		return;

		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			robotis_framework::Dynamixel* dxl_info = state_iter->second;

			joint_name_to_id_[joint_name] = dxl_info->id_;
			read_joint_value_[joint_name_to_id_[joint_name]] = dxls[joint_name]->dxl_state_->present_position_;

			if(offset_start_ == false)
			{
				change_joint_value_[joint_name_to_id_[joint_name]] = dxls[joint_name]->dxl_state_->present_position_;
				ROS_INFO("%d :: %f", joint_name_to_id_[joint_name], change_joint_value_[joint_name_to_id_[joint_name]]);
				result_[joint_id_to_name_[dxl_info->id_]]->goal_position_ = change_joint_value_[joint_name_to_id_[joint_name]]; // 지정된 조인트에 목표 위치 입력
			}
		} // 등록된 다이나믹셀의 위치값을 읽어옴

	result_[joint_id_to_name_[joint_select_]]->goal_position_ = change_joint_value_[joint_select_]; // 지정된 조인트에 목표 위치 입력

}

void OffsetModule::joint_select_sub_function(const std_msgs::Int8::ConstPtr& msg)
{
	joint_select_= msg->data; // GUI 에서 조인트 번호를 받아옴
}
void OffsetModule::change_joint_value_sub_function(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	change_joint_value_[joint_select_] = static_cast<double>(msg->data[0]*(0.088/3)*DEGREE2RADIAN); // GUI에서 변경할 조인트의 값을 받아옴.
	offset_start_ = true;
}
void OffsetModule::offset_joint_value_sub_function(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int i=0; i<30;i++)
	{
		offset_joint_value_[i] = msg->data[i];
		ROS_INFO("%f", offset_joint_value_[i]);
	}
}
bool OffsetModule::read_joint_value_srv_function(offset_module::command::Request  &req, offset_module::command::Response &res)
{
	for(int i =1; i<30; i++)
	{
		res.angle[i] = static_cast<int16_t>(((read_joint_value_[i]*RADIAN2DEGREE))/(0.088/3.0)); // GUI 에서 요청한 모든 조인트의 위치값을 저장함.
	}
	return true;
}
void OffsetModule::save_onoff_sub_function(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data == true)
	{
		// YAML SAVE
		ROS_INFO("YAML_SAVE");
		YAML::Emitter yaml_out;
		std::map<std::string, double> offset;
		std::map<std::string, double> init_pose;

		offset[joint_id_to_name_[1]] = offset_joint_value_[1];  // edit one of the nodes

		for(int i=10; i<23 ; i++)
		{
			offset[joint_id_to_name_[i]] = offset_joint_value_[i];  // edit one of the nodes
		}

		yaml_out << YAML::BeginMap;
		yaml_out << YAML::Key << "offset" << YAML::Value << offset;
		yaml_out << YAML::EndMap;
		std::string offset_file_path = ros::package::getPath("ski_main_manager") + "/config/offset.yaml";
		std::ofstream fout(offset_file_path.c_str());
		fout << yaml_out.c_str();  // dump it back into the file
	}
	else
		return;
}









