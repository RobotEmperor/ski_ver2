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
	motion_time_count = 0;
	motion_time_count_carving = 0;

	//motion
	motion_seq = 0;
	entire_motion_number_pflug = 4;
	entire_motion_number_carving = 4;

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

	for(int i = 0; i<20 ; i++)
	{
		remote_command[i][0] = 0; // 시간
		remote_command[i][1] = 0; // right turn -1  netural 0  left turn 1
	}

	remote_count_time = 0;
	remote_count = 0;

	init_check = false;


}
void remoteTimeMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	remote_update = msg->data;

	if(remote_update == 1)
	{
		remote_count = 0;
	}
	if(remote_update == 0)
	{
		// YAML SAVE
		ROS_INFO("YAML_time_SAVE");
		YAML::Emitter yaml_out;
		std::map<double, double> offset;

		for(int i=0; i<20 ; i++)
		{
			offset[remote_command[i][0]] = remote_command[i][1];
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
	printf("get \n");
}
void currentflagPosition1MsgCallback(const geometry_msgs::Vector3& msg)
{
	if(decision_algorithm->is_moving_check == false)
	{
		decision_algorithm ->temp_flag0[0]  = msg.x;
		decision_algorithm ->temp_flag0[1]  = msg.y;
		decision_algorithm ->temp_flag0[2]  = msg.z;
	}
}
void currentflagPosition2MsgCallback(const geometry_msgs::Vector3& msg)
{
	if(decision_algorithm->is_moving_check == false)
	{
		decision_algorithm ->temp_flag1[0]  = msg.x;
		decision_algorithm ->temp_flag1[1]  = msg.y;
		decision_algorithm ->temp_flag1[2]  = msg.z;
	}
}
void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	turn_type   = msg->turn_type;
	change_type = msg->change_type;

	change_value_center = msg->center_change;
	time_center_change  = msg->time_change;
	time_edge_change    = msg->time_change_edge;

	if(change_value_center == 0)
	{
		if(remote_count > 0 && remote_count < 20)
		{
			remote_count ++;
			remote_command[remote_count][0] = remote_count_time;
			remote_command[remote_count][1] = 0;
		}
	}

	if(!mode.compare("remote"))
	{
		motion_seq = 0;
		motion_time_count_carving = 0;
	}
	else
		return;

}
void updateMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!turn_type.compare("carving_turn"))
		motion->parseMotionData("carving_ver2");
	if(!turn_type.compare("pflug_bogen"))
		motion->parseMotionData("pflug_bogen_ver2");

	change_value_center = 0;
	decision_algorithm->parseMotionData();
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
			if(!turn_type.compare("carving_turn") && change_value_center == 5)
			{
				decision_algorithm->turn_direction = "center";
				motion_center(entire_motion_number_carving); //remote control
				pre_command = decision_algorithm->turn_direction;
				return;
			}

			decision_algorithm->process();

			if(pre_command.compare(decision_algorithm->turn_direction) != 0 && decision_algorithm->turn_direction.compare("center") != 0)
			{
				if(decision_algorithm->is_moving_check == false && pre_direction_command.compare(decision_algorithm->turn_direction) != 0)
				{
					motion_seq = 0;
					motion_time_count_carving = 0;
				}

				pre_direction_command = decision_algorithm->turn_direction; // left right
			}
			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("left_turn"))
				motion_left(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("right_turn"))
				motion_right(entire_motion_number_carving);

			/*			if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("center"))
				motion_center(entire_motion_number_carving);*/

			/*			desired_pose_head_msg.data.clear();
			desired_pose_head_msg.data.push_back(0);
			desired_pose_head_msg.data.push_back(-10*DEGREE2RADIAN);
			desired_pose_head_msg.data.push_back(0);
			desired_pose_head_msg.data.push_back(0.5);
			desired_pose_head_pub.publish(desired_pose_head_msg);
			desired_pose_head_msg.data.clear();*/
		}
		if(!mode.compare("remote"))
		{
			remote_count_time = remote_count_time + 0.006;

			if(!turn_type.compare("pflug_bogen") && change_value_center == -1)
				motion_left(entire_motion_number_pflug);
			if(!turn_type.compare("pflug_bogen") && change_value_center == 1)
				motion_right(entire_motion_number_pflug);
			if(!turn_type.compare("pflug_bogen") && change_value_center == 0)
				motion_center(entire_motion_number_pflug);

			if(!turn_type.compare("carving_turn") && change_value_center == 1)
				motion_left(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && change_value_center == -1)
				motion_right(entire_motion_number_carving);
			if(!turn_type.compare("carving_turn") && change_value_center == 0)
				motion_center(entire_motion_number_carving); //remote control
		}

		pre_command = decision_algorithm->turn_direction;

		if(decision_algorithm->flag_sequence > -1 && decision_algorithm->flag_sequence < 5)
		{
			flag_position[decision_algorithm->flag_sequence][0] = decision_algorithm->top_view_flag_position.x;
			flag_position[decision_algorithm->flag_sequence][1] = decision_algorithm->top_view_flag_position.y;
		}
		else
		{
			//ROS_INFO("Error algorithm!!!!\n");
			return;
		}

		top_view_robot_msg.x =  decision_algorithm->top_view_robot_position.x;
		top_view_robot_msg.y =  decision_algorithm->top_view_robot_position.y;
		top_view_robot_pub.publish(top_view_robot_msg);
		/*
		 * top_view_msg vector3
		top_view_msg.length = 5;
		for(int i = 0; i < 5; i++)
		{
			diana_msgs::FlagData temp;
			temp.position.x = flag_position[i][0];
			temp.position.y = flag_position[i][1];

			top_view_msg.data.push_back(temp);
		}
		top_view_pub.publish(top_view_msg);
		top_view_msg.data.clear();


		top_view_robot_msg.x =  decision_algorithm->top_view_robot_position.x;
		top_view_robot_msg.y =  decision_algorithm->top_view_robot_position.y;
		top_view_robot_pub.publish(top_view_robot_msg);*/

	}
	else
		return;
}
void motion_left(int motion_number)
{
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	if(motion_seq == 0)
	{

		if(remote_count > -1 && remote_count < 20)
		{
			remote_count ++;
			remote_command[remote_count][0] = remote_count_time;
			remote_command[remote_count][1] = 1;
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
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.006;

	if(motion_seq == 0)
	{
		if(remote_count > -1 && remote_count < 20)
		{
			remote_count ++;
			remote_command[remote_count][0] = remote_count_time;
			remote_command[remote_count][1] = -1;
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

	if(motion_time_count_carving < motion->motion_time[motion_number-1])
	{
		decision_algorithm->is_moving_check = true;
	}
	if(motion_time_count_carving > motion->motion_time[motion_number-1])
	{
		decision_algorithm->is_moving_check = false;
	}
}












