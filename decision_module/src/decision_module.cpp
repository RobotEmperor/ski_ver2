/*
 * decision_module.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#include "decision_module/decision_module.h"

void initialize()
{
	motion      = new MotionChange;
	motion_break = new MotionChange;
	motion_first = new MotionChange;
	decision_algorithm = new DecisionModule;

	turn_type = "basic";
	change_type = "basic";

	change_value_center = 0;
	time_center_change  = 0;
	time_edge_change    = 0;

	temp_turn_type = "basic";
	temp_change_type ="basic";

	temp_change_value_center = 0;
	temp_time_center_change  = 0;
	temp_time_edge_change    = 0;

	motion_time_count_carving = 0;

	//motion
	motion_seq = 0;
	motion_break_seq = 0;
	motion_time_count_break = 0;
	entire_motion_number_pflug = 4;
	entire_motion_number_carving = 2;
	entire_motion_number_break = 4;
	entire_motion_number_first = 2;

	first_turn_check = false;

	pre_command = "center";
	pre_direction_command = "center";
	mode = "auto";

	//motion
	for(int i = 0; i < 5; i++)
	{
		flag_position[i][0] = 0;
		flag_position[i][1] = 0;
		flag_position[i][2] = 0;
	}

	remote_update = 0;

	for(int i = 0; i<30 ; i++)
	{
		remote_command[i][0] = 0; // 시간
		remote_command[i][1] = 0; // right turn -1  netural 0  left turn 1
	}

	remote_count_time = 0;
	remote_count = 0;

	init_check = false;

	for(int i = 0; i < 6; i++)
	{
		time_neutral[i] = 0;
	}
	time_break = 0;
	time_count_break = 0;

	neutral_time_count = 0;
	flag_count = 0;
	lidar_check = false;
	pre_lidar_check = false;
	time_check = false;

	pre_direction_change = 0;
	pre_center_change = 0;
	direction_change = 0;
	center_check = false;
	n_check = false;
}
void neutralParseMotionData()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/turn/time_variables_neutral.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
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

	time_neutral[1] = doc["time_1"].as<double>();
	time_neutral[2] = doc["time_2"].as<double>();
	time_neutral[3] = doc["time_3"].as<double>();
	time_neutral[4] = doc["time_4"].as<double>();
	time_neutral[5] = doc["time_5"].as<double>();
	time_break = doc["time_break"].as<double>();
}

void remoteTimeMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	remote_update = msg->data;

	if(remote_update == 1)
	{
		remote_time_.clear();
		remote_command_.clear();
	}
	if(remote_update == 0)
	{
		// YAML SAVE
		ROS_INFO("YAML_time_SAVE");
		YAML::Emitter yaml_out;
		std::map<double, double> offset;


		for(int i=0; i< remote_time_.size() ; i++)
		{
			offset[remote_time_[i]] = remote_command_[i];
		}

		yaml_out << YAML::BeginMap;
		yaml_out << YAML::Key << "remote_time" << YAML::Value << offset;
		yaml_out << YAML::EndMap;

		std::string offset_file_path = ros::package::getPath("ski_main_manager") + "/data/remote_time.yaml";
		std::ofstream fout(offset_file_path.c_str());
		fout << yaml_out.c_str();  // dump it back into the file
	}
}

void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	ready_check = msg->data;
	change_value_center = 0;
}
void initCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	init_check = msg->data;
}
void lidarCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	if(decision_algorithm->is_moving_check == true)
	{
		if(msg->data == true)
			lidar_check = msg->data;
	}
}
void currentflagPosition1MsgCallback(const geometry_msgs::Vector3& msg)
{

	decision_algorithm ->temp_flag0[0]  = msg.x;
	decision_algorithm ->temp_flag0[1]  = msg.y;
	decision_algorithm ->temp_flag0[2]  = msg.z;

	decision_algorithm -> data_in_check_1 = true;
}
void currentflagPosition2MsgCallback(const geometry_msgs::Vector3& msg)
{

	decision_algorithm ->temp_flag1[0]  = msg.x;
	decision_algorithm ->temp_flag1[1]  = msg.y;
	decision_algorithm ->temp_flag1[2]  = msg.z;

	decision_algorithm -> data_in_check_2 = true;
}
void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	turn_type   = msg->turn_type;
	change_type = msg->change_type;
	time_center_change  = msg->time_change;
	time_edge_change    = msg->time_change_edge;


	if(!mode.compare("remote"))
	{
		change_value_center = msg->center_change;
	}

	if(change_value_center == 0)
	{
		remote_time_.push_back(remote_count_time);
		remote_command_.push_back(0);
	}
}
void updateMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!turn_type.compare("carving_turn"))
		motion->parseMotionData("carving_ver2");
	if(!turn_type.compare("pflug_bogen"))
		motion->parseMotionData("pflug_bogen_ver2");

	motion_break->parseMotionData("break");
	motion_first->parseMotionData("first_carving");
	decision_algorithm->parseMotionData();
	neutralParseMotionData();

	first_turn_check = false;

	change_value_center = 0;
	//init
	flag_count = 0;
	decision_algorithm->flag_sequence = flag_count;
	pre_command = "center";
	pre_direction_command = "center";
	decision_algorithm->turn_direction = "center";

	motion_break_seq = 0;
	motion_time_count_break = 0;
}
void modeChangeMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data == 0)
	{
		mode = "remote";
		change_value_center = 0;
	}
	else
		mode = "auto";
}

int main(int argc, char **argv)
{
	initialize();
	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;

	// publisher
	desired_pose_leg_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_leg",1);
	desired_pose_waist_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_waist",1);
	desired_pose_head_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_head",1);
	turn_command_pub = ros_node.advertise<std_msgs::String>("/turn_command",1);

	//map top view
	init_top_view_pub = ros_node.advertise<diana_msgs::FlagDataTop>("/init_top_view",1);
	top_view_robot_pub = ros_node.advertise<geometry_msgs::Vector3>("/top_view_robot",1);

	desired_pose_all_pub  = ros_node.advertise<diana_msgs::DesiredPoseCommand>("/desired_pose_all",1);

	// subcriber
	ros::Subscriber ready_check_sub = ros_node.subscribe("/ready_check", 5, readyCheckMsgCallBack);

	// init map
	ros::Subscriber init_check_sub = ros_node.subscribe("/init_check", 5, initCheckMsgCallBack);

	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, desiredCenterChangeMsgCallback);
	ros::Subscriber update_sub = ros_node.subscribe("/update", 5, updateMsgCallback);

	ros::Subscriber current_flag_position1_sub = ros_node.subscribe("/current_flag_position1", 5, currentflagPosition1MsgCallback);
	ros::Subscriber current_flag_position2_sub = ros_node.subscribe("/current_flag_position2", 5, currentflagPosition2MsgCallback);

	ros::Subscriber remote_time_sub = ros_node.subscribe("/remote_time", 5, remoteTimeMsgCallBack);

	ros::Subscriber mode_change_sub = ros_node.subscribe("/mode_change", 5, modeChangeMsgCallback);

	//lidar check
	ros::Subscriber lidar_check_sub = ros_node.subscribe("/gate_switch", 5, lidarCheckMsgCallBack);
	ros::Timer timer = ros_node.createTimer(ros::Duration(0.006), control_loop);

	ros::Rate loop_rate(1000);

	//carving_motion -> parseMotionData();

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
	{
		//////////////////// initialize function
		if(init_check)
		{
			decision_algorithm->initialize(0.006, 5);
			init_check = !decision_algorithm->init_complete_check;

			if(decision_algorithm->init_complete_check)
			{
				for(int num = 0; num < 5; num ++)
				{
					init_top_view_msg.flag_in_data_m_x[num] = decision_algorithm->flag_in_data[num][0];
					init_top_view_msg.flag_in_data_m_y[num] = decision_algorithm->flag_in_data[num][1];
					init_top_view_msg.flag_out_data_m_x[num] = decision_algorithm->flag_out_data[num][0];
					init_top_view_msg.flag_out_data_m_y[num] = decision_algorithm->flag_out_data[num][1];
				}
				init_top_view_pub.publish(init_top_view_msg);
			}
			return;
		}
		/////////////////////////////////////////

		if(!mode.compare("auto"))
		{
			if(!turn_type.compare("carving_turn") && change_value_center == 7)
			{
				decision_algorithm->turn_direction = "center";
				motion_center(entire_motion_number_carving); //remote control
				pre_command = decision_algorithm->turn_direction;
				return;
			}

			if(flag_count > 6)
			{
				flag_count = 6;
				decision_algorithm -> flag_sequence = flag_count;
				return;
			}
			else
			{
				decision_algorithm -> flag_sequence = flag_count;
			}

			//
			neutral_check_function();
			decision_algorithm->process();



			if(pre_command.compare(decision_algorithm->turn_direction) != 0 && decision_algorithm->turn_direction.compare("center") != 0)
			{
				if(decision_algorithm->is_moving_check == false && pre_direction_command.compare(decision_algorithm->turn_direction) != 0)
				{
					motion_seq = 0;
					motion_time_count_carving = 0;

					change_value_center = decision_algorithm->turn_command;
				}
			}
			pre_direction_command = decision_algorithm->turn_direction; // left right

			if(!decision_algorithm->turn_direction.compare("center"))
			{
				if(pre_command.compare(decision_algorithm->turn_direction) != 0)
				{
					motion_seq = -1;
					motion_time_count_carving = 0;

					change_value_center = decision_algorithm->turn_command;
				}
			}

			/*if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("first_left_turn"))
				motion_first_turn_left_fun(entire_motion_number_first);
			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("left_turn") && flag_count > 1)
				motion_left(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("right_turn"))
				motion_right(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("center"))
				motion_center(entire_motion_number_carving);*/



			decision_algorithm -> data_in_check_1 = false;
			decision_algorithm -> data_in_check_2 = false;

			if(decision_algorithm -> flag_check == 1)
			{
				init_top_view_msg.flag_in_data_m_x[decision_algorithm ->flag_sequence] = decision_algorithm ->top_view_flag_position.x;
				init_top_view_msg.flag_in_data_m_y[decision_algorithm ->flag_sequence] = decision_algorithm ->top_view_flag_position.y;
				init_top_view_msg.flag_out_data_m_x[decision_algorithm ->flag_sequence] = decision_algorithm ->top_view_flag_position.x;
				init_top_view_msg.flag_out_data_m_y[decision_algorithm ->flag_sequence] = decision_algorithm ->top_view_flag_position.y + (decision_algorithm ->flag_direction)*5;

				init_top_view_pub.publish(init_top_view_msg);
				decision_algorithm -> flag_check = 0;
			}

			if(flag_count == 0)// 첫번째 기문
			{
				desired_pose_head_msg.data.clear();
				desired_pose_head_msg.data.push_back(0.4);
				desired_pose_head_msg.data.push_back(-10*DEGREE2RADIAN);
				desired_pose_head_msg.data.push_back(0);
				desired_pose_head_msg.data.push_back(0.5);
				desired_pose_head_pub.publish(desired_pose_head_msg);
				desired_pose_head_msg.data.clear();
			}
			if(flag_count == 1)// 첫번째 기문
			{
				desired_pose_head_msg.data.clear();
				desired_pose_head_msg.data.push_back(0);
				desired_pose_head_msg.data.push_back(-10*DEGREE2RADIAN);
				desired_pose_head_msg.data.push_back(0);
				desired_pose_head_msg.data.push_back(0.5);
				desired_pose_head_pub.publish(desired_pose_head_msg);
				desired_pose_head_msg.data.clear();
			}

			turn_command_msg.data = decision_algorithm->turn_direction;
			turn_command_pub.publish(turn_command_msg);
		}

		if(!mode.compare("remote"))
		{
			remote_count_time = remote_count_time + 0.006;
		}
		/*
			if(!turn_type.compare("carving_turn") && change_value_center == 1 && flag_count > 1)
				motion_left(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && change_value_center == 1 &&  first_turn_check == false)
				motion_first_turn_left_fun(entire_motion_number_first);
			if(!turn_type.compare("carving_turn") && change_value_center == -1)
				motion_right(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && change_value_center == 0)
				motion_center(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && change_value_center == 2)
				motion_break_fun(entire_motion_number_break);//remote control
		 */
		static int status = 0;

		cout << decision_algorithm -> is_moving_check << endl;


		if(status == 0)
		{
			if(!decision_algorithm -> is_moving_check)
			{
				if(change_value_center == 1 || change_value_center == -1 || change_value_center == 5 || change_value_center == -5 || change_value_center == 2)
				{
					status = change_value_center;
					motion_time_count_carving = 0;
					motion_seq = 0;
				}
			}
		}
		else if(status == 1 || status == -1 || status == 5 || status == -5)
		{
			if(!decision_algorithm -> is_moving_check)
			{
				change_value_center = 0;
				status = 0;
				motion_time_count_carving = 0;
				motion_seq = 0;
			}
			else
			{
				if(change_value_center != status)
				{
					status = 0;
					motion_time_count_carving = 0;
					motion_seq = 0;
				}
			}
		}
		else if(status == 2)
		{
			if(change_value_center != status)
			{
				status = 0;
				motion_time_count_carving = 0;
				motion_seq = 0;
			}
		}
		decision_algorithm -> status = status;

		if(!turn_type.compare("carving_turn") && status == 1)
			motion_left(entire_motion_number_carving);
		else if(!turn_type.compare("carving_turn") && status == 5)
			motion_first_turn_left_fun(entire_motion_number_first);
		else if(!turn_type.compare("carving_turn") && status == -1)
			motion_right(entire_motion_number_carving);
		else if(!turn_type.compare("carving_turn") && status == 0)
			motion_center(entire_motion_number_carving);
		else if(!turn_type.compare("carving_turn") && status == 2)
			motion_break_fun(1);//remote control

		printf("is moving check :: %d  status ::  %d \n",decision_algorithm->is_moving_check, status);

		pre_command = decision_algorithm->turn_direction;

		top_view_robot_msg.x =  decision_algorithm->top_view_robot_position.x;
		top_view_robot_msg.y =  decision_algorithm->top_view_robot_position.y;
		top_view_robot_pub.publish(top_view_robot_msg);
	}
}
void neutral_check_function()
{
	if(lidar_check == true)
	{
		time_check = true;
		lidar_check = false;
		neutral_time_count = 0;
	}
	if(time_check)
	{
		neutral_time_count = neutral_time_count + 0.006;

		if(neutral_time_count > time_neutral[flag_count])
		{
			decision_algorithm->neutral_check = 1;
			time_check = false;
		}
	}
	else
		decision_algorithm->neutral_check = 0;
}

void motion_left(int motion_number)
{
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	if(motion_seq == 0)
	{
		flag_count++;

		if(!mode.compare("remote"))
		{
			remote_time_.push_back(remote_count_time);
			remote_command_.push_back(1);
		}

		for(int var = 0; var < 12 ; var++)
		{
			desired_pose_all_msg.leg_final_position[var] = motion->motion_left_position[0][var];
			desired_pose_all_msg.leg_init_vel[var]       = motion->motion_init_leg_left_vel[0][var];
			desired_pose_all_msg.leg_final_vel[var]      = motion->motion_final_leg_left_vel[0][var];
		}
		for(int var = 0; var < 2 ; var++)
		{
			desired_pose_all_msg.waist_final_position[var] = motion->motion_waist_left_position[0][var];
			desired_pose_all_msg.waist_init_vel[var]       = motion->motion_init_waist_left_vel[0][var];
			desired_pose_all_msg.waist_final_vel[var]      = motion->motion_final_waist_left_vel[0][var];

		}
		for(int var = 0; var < 6 ; var++)
		{
			desired_pose_all_msg.arm_final_position[var] = motion->motion_arm_left_position[0][var];
			desired_pose_all_msg.arm_init_vel[var]       = motion->motion_init_arm_left_vel[0][var];
			desired_pose_all_msg.arm_final_vel[var]      = motion->motion_final_arm_left_vel[0][var];
		}
		desired_pose_all_msg.time_leg = motion->motion_time[0];
		desired_pose_all_msg.time_waist = motion->motion_time[0];
		desired_pose_all_msg.time_arm = motion->motion_time[0];
		desired_pose_all_pub.publish(desired_pose_all_msg);
		motion_seq ++;
		motion_time_count_carving = 0;
		decision_algorithm->is_moving_check = true;
		decision_algorithm->turn_direction = "left_turn";
	}

	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		if(motion_time_count_carving > motion->motion_time[motion_num-1] && motion_seq == motion_num)
		{
			for(int var = 0; var <12 ; var++)
			{
				desired_pose_all_msg.leg_final_position[var] = motion->motion_left_position[motion_num][var];
				desired_pose_all_msg.leg_init_vel[var]       = motion->motion_init_leg_left_vel[motion_num][var];
				desired_pose_all_msg.leg_final_vel[var]      = motion->motion_final_leg_left_vel[motion_num][var];
			}

			for(int var = 0; var < 2 ; var++)
			{
				desired_pose_all_msg.waist_final_position[var] = motion->motion_waist_left_position[motion_num][var];
				desired_pose_all_msg.waist_init_vel[var]       = motion->motion_init_waist_left_vel[motion_num][var];
				desired_pose_all_msg.waist_final_vel[var]      = motion->motion_final_waist_left_vel[motion_num][var];

			}
			for(int var = 0; var < 6 ; var++)
			{
				desired_pose_all_msg.arm_final_position[var] = motion->motion_arm_left_position[motion_num][var];
				desired_pose_all_msg.arm_init_vel[var]       = motion->motion_init_arm_left_vel[motion_num][var];
				desired_pose_all_msg.arm_final_vel[var]      = motion->motion_final_arm_left_vel[motion_num][var];
			}
			desired_pose_all_msg.time_leg = motion->motion_time[motion_num];
			desired_pose_all_msg.time_waist = motion->motion_time[motion_num];
			desired_pose_all_msg.time_arm = motion->motion_time[motion_num];
			motion_seq ++;
			motion_time_count_carving = 0;
			desired_pose_all_pub.publish(desired_pose_all_msg);
			decision_algorithm->is_moving_check = true;
			decision_algorithm->turn_direction = "left_turn";
		}
	}

	if(motion_time_count_carving > motion->motion_time[motion_number-1] && motion_seq == motion_number)
	{
		motion_seq ++;
		decision_algorithm->is_moving_check = false;
	}
}

void motion_right(int motion_number)
{
	first_turn_check = true;

	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	if(motion_seq == 0)
	{
		flag_count++;

		if(!mode.compare("remote"))
		{
			remote_time_.push_back(remote_count_time);
			remote_command_.push_back(-1);
		}
		for(int var = 0; var < 12 ; var++)
		{
			desired_pose_all_msg.leg_final_position[var] = motion->motion_right_position[0][var];
			desired_pose_all_msg.leg_init_vel[var]       = motion->motion_init_leg_right_vel[0][var];
			desired_pose_all_msg.leg_final_vel[var]      = motion->motion_final_leg_right_vel[0][var];
		}
		for(int var = 0; var < 2 ; var++)
		{
			desired_pose_all_msg.waist_final_position[var] = motion->motion_waist_right_position[0][var];
			desired_pose_all_msg.waist_init_vel[var]       = motion->motion_init_waist_right_vel[0][var];
			desired_pose_all_msg.waist_final_vel[var]      = motion->motion_final_waist_right_vel[0][var];

		}
		for(int var = 0; var < 6 ; var++)
		{
			desired_pose_all_msg.arm_final_position[var] = motion->motion_arm_right_position[0][var];
			desired_pose_all_msg.arm_init_vel[var]       = motion->motion_init_arm_right_vel[0][var];
			desired_pose_all_msg.arm_final_vel[var]      = motion->motion_final_arm_right_vel[0][var];
		}
		desired_pose_all_msg.time_leg = motion->motion_time[0];
		desired_pose_all_msg.time_waist = motion->motion_time[0];
		desired_pose_all_msg.time_arm = motion->motion_time[0];
		desired_pose_all_pub.publish(desired_pose_all_msg);
		motion_seq ++;
		motion_time_count_carving = 0;
		decision_algorithm->is_moving_check = true;
		decision_algorithm->turn_direction = "right_turn";
	}

	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		if(motion_time_count_carving > motion->motion_time[motion_num-1] && motion_seq == motion_num)
		{
			for(int var = 0; var <12 ; var++)
			{
				desired_pose_all_msg.leg_final_position[var] = motion->motion_right_position[motion_num][var];
				desired_pose_all_msg.leg_init_vel[var]       = motion->motion_init_leg_right_vel[motion_num][var];
				desired_pose_all_msg.leg_final_vel[var]      = motion->motion_final_leg_right_vel[motion_num][var];
			}

			for(int var = 0; var < 2 ; var++)
			{
				desired_pose_all_msg.waist_final_position[var] = motion->motion_waist_right_position[motion_num][var];
				desired_pose_all_msg.waist_init_vel[var]       = motion->motion_init_waist_right_vel[motion_num][var];
				desired_pose_all_msg.waist_final_vel[var]      = motion->motion_final_waist_right_vel[motion_num][var];

			}
			for(int var = 0; var < 6 ; var++)
			{
				desired_pose_all_msg.arm_final_position[var] = motion->motion_arm_right_position[motion_num][var];
				desired_pose_all_msg.arm_init_vel[var]       = motion->motion_init_arm_right_vel[motion_num][var];
				desired_pose_all_msg.arm_final_vel[var]      = motion->motion_final_arm_right_vel[motion_num][var];
			}
			desired_pose_all_msg.time_leg = motion->motion_time[motion_num];
			desired_pose_all_msg.time_waist = motion->motion_time[motion_num];
			desired_pose_all_msg.time_arm = motion->motion_time[motion_num];
			motion_seq ++;
			motion_time_count_carving = 0;
			desired_pose_all_pub.publish(desired_pose_all_msg);
			decision_algorithm->is_moving_check = true;
			decision_algorithm->turn_direction = "right_turn";

		}
	}

	if(motion_time_count_carving > motion->motion_time[motion_number-1] && motion_seq == motion_number)
	{
		motion_seq ++;
		decision_algorithm->is_moving_check = false;
	}
}
void motion_center(int motion_number)
{
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	for(int var = 0; var <12 ; var++)
	{
		desired_pose_all_msg.leg_final_position[var] = motion->motion_center_position[var];
		desired_pose_all_msg.leg_init_vel[var]       = 0;
		desired_pose_all_msg.leg_final_vel[var]      = 0;
	}
	for(int var = 0; var < 2 ; var++)
	{
		desired_pose_all_msg.waist_final_position[var] = motion->motion_waist_center_position[var];
		desired_pose_all_msg.waist_init_vel[var]       = 0;
		desired_pose_all_msg.waist_final_vel[var]      = 0;
	}
	for(int var = 0; var < 6 ; var++)
	{
		desired_pose_all_msg.arm_final_position[var] = motion->motion_arm_center_position[var];
		desired_pose_all_msg.arm_init_vel[var]       = 0;
		desired_pose_all_msg.arm_final_vel[var]      = 0;
	}
	desired_pose_all_msg.time_leg = motion->motion_time[motion_number];
	desired_pose_all_msg.time_waist = motion->motion_time[motion_number];
	desired_pose_all_msg.time_arm = motion->motion_time[motion_number];
	desired_pose_all_pub.publish(desired_pose_all_msg);
	center_check = true;
	decision_algorithm->is_moving_check = true;

	if(motion_time_count_carving > motion->motion_time[motion_number])
	{
		decision_algorithm->is_moving_check = false;
		center_check = false;
	}
}

void motion_break_fun(int motion_number)
{
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	for(int var = 0; var <12 ; var++)
	{
		desired_pose_all_msg.leg_final_position[var] = motion_break->motion_center_position[var];
		desired_pose_all_msg.leg_init_vel[var]       = 0;
		desired_pose_all_msg.leg_final_vel[var]      = 0;
	}
	for(int var = 0; var < 2 ; var++)
	{
		desired_pose_all_msg.waist_final_position[var] = motion_break->motion_waist_center_position[var];
		desired_pose_all_msg.waist_init_vel[var]       = 0;
		desired_pose_all_msg.waist_final_vel[var]      = 0;
	}
	for(int var = 0; var < 6 ; var++)
	{
		desired_pose_all_msg.arm_final_position[var] = motion_break->motion_arm_center_position[var];
		desired_pose_all_msg.arm_init_vel[var]       = 0;
		desired_pose_all_msg.arm_final_vel[var]      = 0;
	}
	desired_pose_all_msg.time_leg = motion_break->motion_time[motion_number-1];
	desired_pose_all_msg.time_waist = motion_break->motion_time[motion_number-1];
	desired_pose_all_msg.time_arm = motion_break->motion_time[motion_number-1];
	desired_pose_all_pub.publish(desired_pose_all_msg);
	center_check = true;
	decision_algorithm->is_moving_check = true;

	if(motion_time_count_carving > motion_break->motion_time[motion_number-1])
	{
		decision_algorithm->is_moving_check = false;
	}

}
void motion_first_turn_left_fun(int motion_number)
{
	motion_first->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	if(motion_seq== 0)
	{
		flag_count ++;

		if(!mode.compare("remote"))
		{
			remote_time_.push_back(remote_count_time);
			remote_command_.push_back(2);
		}

		for(int var = 0; var < 12 ; var++)
		{
			desired_pose_all_msg.leg_final_position[var] = motion_first->motion_left_position[0][var];
			desired_pose_all_msg.leg_init_vel[var]       = motion_first->motion_init_leg_left_vel[0][var];
			desired_pose_all_msg.leg_final_vel[var]      = motion_first->motion_final_leg_left_vel[0][var];
		}
		for(int var = 0; var < 2 ; var++)
		{
			desired_pose_all_msg.waist_final_position[var] = motion_first->motion_waist_left_position[0][var];
			desired_pose_all_msg.waist_init_vel[var]       = motion_first->motion_init_waist_left_vel[0][var];
			desired_pose_all_msg.waist_final_vel[var]      = motion_first->motion_final_waist_left_vel[0][var];

		}
		for(int var = 0; var < 6 ; var++)
		{
			desired_pose_all_msg.arm_final_position[var] = motion_first->motion_arm_left_position[0][var];
			desired_pose_all_msg.arm_init_vel[var]       = motion_first->motion_init_arm_left_vel[0][var];
			desired_pose_all_msg.arm_final_vel[var]      = motion_first->motion_final_arm_left_vel[0][var];
		}
		desired_pose_all_msg.time_leg = motion_first->motion_time[0];
		desired_pose_all_msg.time_waist = motion_first->motion_time[0];
		desired_pose_all_msg.time_arm = motion_first->motion_time[0];
		desired_pose_all_pub.publish(desired_pose_all_msg);
		motion_seq ++;
		motion_time_count_carving = 0;
		decision_algorithm->is_moving_check = true;
		decision_algorithm->turn_direction = "first_left_turn";
	}

	for(int motion_num = 1; motion_num < motion_number; motion_num++)
	{
		if(motion_time_count_carving > motion_first->motion_time[motion_num-1] && motion_seq == motion_num)
		{
			for(int var = 0; var <12 ; var++)
			{
				desired_pose_all_msg.leg_final_position[var] = motion_first->motion_left_position[motion_num][var];
				desired_pose_all_msg.leg_init_vel[var]       = motion_first->motion_init_leg_left_vel[motion_num][var];
				desired_pose_all_msg.leg_final_vel[var]      = motion_first->motion_final_leg_left_vel[motion_num][var];
			}

			for(int var = 0; var < 2 ; var++)
			{
				desired_pose_all_msg.waist_final_position[var] = motion_first->motion_waist_left_position[motion_num][var];
				desired_pose_all_msg.waist_init_vel[var]       = motion_first->motion_init_waist_left_vel[motion_num][var];
				desired_pose_all_msg.waist_final_vel[var]      = motion_first->motion_final_waist_left_vel[motion_num][var];

			}
			for(int var = 0; var < 6 ; var++)
			{
				desired_pose_all_msg.arm_final_position[var] = motion_first->motion_arm_left_position[motion_num][var];
				desired_pose_all_msg.arm_init_vel[var]       = motion_first->motion_init_arm_left_vel[motion_num][var];
				desired_pose_all_msg.arm_final_vel[var]      = motion_first->motion_final_arm_left_vel[motion_num][var];
			}
			desired_pose_all_msg.time_leg = motion_first->motion_time[motion_num];
			desired_pose_all_msg.time_waist = motion_first->motion_time[motion_num];
			desired_pose_all_msg.time_arm = motion_first->motion_time[motion_num];
			motion_seq ++;
			motion_time_count_carving = 0;
			desired_pose_all_pub.publish(desired_pose_all_msg);
			decision_algorithm->is_moving_check = true;
			decision_algorithm->turn_direction = "first_left_turn";
		}
	}

	if(motion_time_count_carving > motion_first->motion_time[motion_number-1] && motion_seq == motion_number)
	{
		motion_seq ++;
		decision_algorithm->is_moving_check = false;
	}

}











