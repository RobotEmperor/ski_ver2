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
}

void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	ready_check = msg->data;
}
void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	turn_type   = msg->turn_type;
	change_type = msg->change_type;

	change_value_center = msg->center_change;
	time_center_change  = msg->time_change;
	time_edge_change    = msg->time_change_edge;

	motion_seq = 0;
	motion_time_count_carving = 0;
}
void currentFlagPosition1MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	decision_algorithm->temp_flag0[0] = msg->x;
	decision_algorithm->temp_flag0[1] = msg->y;
	decision_algorithm->temp_flag0[2] = msg->z; // flag 에서 로봇을 본 xyz
}
void currentFlagPosition2MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	decision_algorithm->temp_flag1[0] = msg->x;
	decision_algorithm->temp_flag1[1] = msg->y;
	decision_algorithm->temp_flag1[2] = msg->z; // flag 에서 로봇을 본 xyz
}
void currentFlagPosition3MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	decision_algorithm->temp_flag2[0] = msg->x;
	decision_algorithm->temp_flag2[1] = msg->y;
	decision_algorithm->temp_flag2[2] = msg->z; // flag 에서 로봇을 본 xyz
}
void currentFlagPosition4MsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	decision_algorithm->temp_flag3[0] = msg->x;
	decision_algorithm->temp_flag3[1] = msg->y;
	decision_algorithm->temp_flag3[2] = msg->z; // flag 에서 로봇을 본 xyz
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

int main(int argc, char **argv)
{
	initialize();
	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;

	// publisher
	desired_pose_leg_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_leg",1);
	desired_pose_waist_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_waist",1);

	desired_pose_all_pub  = ros_node.advertise<diana_msgs::DesiredPoseCommand>("/desired_pose_all",1);


	// subcriber
	ros::Subscriber ready_check_sub = ros_node.subscribe("/ready_check", 5, readyCheckMsgCallBack);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, desiredCenterChangeMsgCallback);
	ros::Subscriber update_sub = ros_node.subscribe("/update", 5, updateMsgCallback);

	ros::Subscriber current_flag_position_1 = ros_node.subscribe("/current_flag_position1", 5, currentFlagPosition1MsgCallback);
	ros::Subscriber current_flag_position_2 = ros_node.subscribe("/current_flag_position2", 5, currentFlagPosition2MsgCallback);
	ros::Subscriber current_flag_position_3 = ros_node.subscribe("/current_flag_position3", 5, currentFlagPosition3MsgCallback);
	ros::Subscriber current_flag_position_4 = ros_node.subscribe("/current_flag_position4", 5, currentFlagPosition4MsgCallback);

	ros::Timer timer = ros_node.createTimer(ros::Duration(0.008), control_loop);

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
		decision_algorithm->process();

		if(pre_command.compare(decision_algorithm->turn_direction) != 0)
		{
			motion_seq = 0;
			motion_time_count_carving = 0;
		}
		if(!turn_type.compare("pflug_bogen") && change_value_center == -1)
			motion_left(entire_motion_number_pflug);
		if(!turn_type.compare("pflug_bogen") && change_value_center == 1)
			motion_right(entire_motion_number_pflug);
		if(!turn_type.compare("pflug_bogen") && change_value_center == 0)
			motion_center(entire_motion_number_pflug);

		if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("left_turn"))
		{
			motion_left(entire_motion_number_carving);
		}
		if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("right_turn"))
		{
			motion_right(entire_motion_number_carving);
		}
		if(!turn_type.compare("carving_turn") && !decision_algorithm->turn_direction.compare("center"))
		{
			motion_center(entire_motion_number_carving);
		}

		pre_command = decision_algorithm->turn_direction;

		/*if(!turn_type.compare("carving_turn") && change_value_center == -1)
			motion_left(entire_motion_number_carving);
		if(!turn_type.compare("carving_turn") && change_value_center == 1)
			motion_right(entire_motion_number_carving);
		if(!turn_type.compare("carving_turn") && change_value_center == 0)
			motion_center(entire_motion_number_carving); //remote control*/

		if(!turn_type.compare("carving_turn") && change_value_center == -1)
			motion_center(entire_motion_number_carving); //remote control
	}
	else
		return;
}
void motion_left(int motion_number)
{
	motion->calculate_init_final_velocity(motion_number);
	motion_time_count_carving = motion_time_count_carving + 0.008;

	if(motion_seq == 0)
	{
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
	motion_time_count_carving = motion_time_count_carving + 0.008;

	if(motion_seq == 0)
	{
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
}












