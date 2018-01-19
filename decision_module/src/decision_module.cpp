/*
 * decision_module.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#include "decision_module/decision_module.h"

void initialize()
{
	center_change_leg = new CenterChangeLeg;
	center_change_waist = new CenterChangeWaist;
	//carving_motion      = new CarvingChange;

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

	leg_trj_time = 0;

	for(int i=0;i<6;i++)
	{
		leg_xyz_ypr_l[i] = 0;
		leg_xyz_ypr_r[i] = 0;
	}
	leg_trj_time = 0;

	waist_roll = 0;
	waist_yaw  = 0;
	waist_roll_time = 0;
	waist_yaw_time  = 0;

	for(int i=0;i<6;i++)
	{
		pre_leg_xyz_ypr_l[i] = 0;
		pre_leg_xyz_ypr_r[i] = 0;
	}
	pre_leg_trj_time = 0;

	pre_waist_roll = 0;
	pre_waist_yaw  = 0;
	pre_waist_roll_time = 0;
	pre_waist_yaw_time  = 0;
}




void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	ready_check = msg->data;
}
int main(int argc, char **argv)
{
	initialize();
	ros::init(argc, argv, "decision_module");
	ros::NodeHandle ros_node;

	// publisher
	desired_pose_leg_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_leg",1);
	desired_pose_waist_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/desired_pose_waist",1);


	// subcriber
	ros::Subscriber ready_check_sub = ros_node.subscribe("/ready_check", 5, readyCheckMsgCallBack);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, desiredCenterChangeMsgCallback);


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
		desired_leg_pose_pflug();
		desired_leg_pose_carving();
	}
	else
		return;
}

void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	turn_type   = msg->turn_type;
	change_type = msg->change_type;

	change_value_center = msg->center_change;
	time_center_change  = msg->time_change;
	time_edge_change    = msg->time_change_edge;

}

void desired_leg_pose_pflug()
{
	if(!turn_type.compare("pflug_bogen"))
	{
		if (temp_change_value_center != change_value_center || temp_turn_type.compare(turn_type) || temp_change_type.compare(change_type))
		{
			center_change_waist->parseMotionData("pflug_bogen", "center_change");
			center_change_waist->calculateStepEndPointValue(change_value_center,100,"center_change");

			center_change_leg->parseMotionData("pflug_bogen", "center_change");
			center_change_leg->calculateStepEndPointValue(change_value_center,100,"center_change"); // 0.01 단위로 조정 가능.

			if(change_value_center == 0)
			{
				waist_yaw = center_change_waist->step_end_point_value[0];
				waist_yaw_time = time_center_change;
			}
			waist_roll = center_change_waist->step_end_point_value[1];
			waist_roll_time =  time_center_change;

			for(int m = 0 ; m<6 ; m++)
			{
				leg_xyz_ypr_l[m] = center_change_leg->step_end_point_value[0][m];
				leg_xyz_ypr_r[m] = center_change_leg->step_end_point_value[1][m];
				leg_trj_time = time_center_change;
			}
			center_change_moving_check = true;
			motion_time_count = 0;

			temp_turn_type   = turn_type;
			temp_change_type = change_type;
			temp_change_value_center = change_value_center;
			temp_time_center_change  = time_center_change;
			temp_time_edge_change    = time_edge_change;
			ROS_INFO("Turn !!  Change");
		} // 변한 것이 있으면 값을 계산

		if(center_change_moving_check == true && change_value_center !=0)
		{
			motion_time_count  = motion_time_count  + 0.008;

			if(motion_time_count > time_center_change)
			{
				center_change_leg->parseMotionData("pflug_bogen", "edge_change");
				center_change_waist->parseMotionData("pflug_bogen", "center_change");


				if(change_value_center > 0)
				{
					center_change_leg->calculateStepEndPointValue(1,100,"edge_change"); // 0.01 단위로 조정 가능.
					center_change_waist->calculateStepEndPointValue(1,100,"center_change");
				}
				if(change_value_center < 0)
				{
					center_change_leg->calculateStepEndPointValue(-1,100,"edge_change"); // 0.01 단위로 조정 가능.
					center_change_waist->calculateStepEndPointValue(-1,100,"center_change");
				}
				for(int m = 3 ; m<6 ; m++)
				{
					leg_xyz_ypr_l[m] = center_change_leg->step_end_point_value[0][m];
					leg_xyz_ypr_r[m] = center_change_leg->step_end_point_value[1][m];
					leg_trj_time = time_edge_change;
				}

				waist_yaw = center_change_waist->step_end_point_value[0];
				waist_yaw_time = time_edge_change;

				center_change_moving_check = false;
				motion_time_count = 0;
			}
		}
		if(pre_leg_xyz_ypr_l[0] != leg_xyz_ypr_l[0] || pre_leg_xyz_ypr_l[1] != leg_xyz_ypr_l[1] || pre_leg_xyz_ypr_l[2] != leg_xyz_ypr_l[2] || pre_leg_xyz_ypr_l[3] != leg_xyz_ypr_l[3] ||
				pre_leg_xyz_ypr_l[4] != leg_xyz_ypr_l[4] || pre_leg_xyz_ypr_l[5] != leg_xyz_ypr_l[5] || pre_leg_xyz_ypr_r[0] != leg_xyz_ypr_r[0] || pre_leg_xyz_ypr_r[1] != leg_xyz_ypr_r[1] ||
				pre_leg_xyz_ypr_r[2] != leg_xyz_ypr_r[2] || pre_leg_xyz_ypr_r[3] != leg_xyz_ypr_r[3] || pre_leg_xyz_ypr_r[4] != leg_xyz_ypr_r[4] || pre_leg_xyz_ypr_r[5] != leg_xyz_ypr_r[5] ||
				pre_waist_roll != waist_roll             || pre_waist_yaw != waist_yaw               || pre_leg_trj_time != leg_trj_time         || pre_waist_roll_time != waist_roll_time   ||
				pre_waist_yaw_time != waist_yaw_time)
		{

			desired_pose_waist_msg.data.push_back(waist_yaw);
			desired_pose_waist_msg.data.push_back(waist_roll);
			desired_pose_waist_msg.data.push_back(waist_yaw_time);
			desired_pose_waist_msg.data.push_back(waist_roll_time);
			desired_pose_waist_pub.publish(desired_pose_waist_msg);
			desired_pose_waist_msg.data.clear();

			for(int m = 0 ; m<6 ; m++)
			{
				desired_pose_leg_msg.data.push_back(leg_xyz_ypr_l[m]);
			}
			for(int m = 0 ; m<6 ; m++)
			{
				desired_pose_leg_msg.data.push_back(leg_xyz_ypr_r[m]);
			}
			desired_pose_leg_msg.data.push_back(leg_trj_time);
			desired_pose_leg_pub.publish(desired_pose_leg_msg);
			desired_pose_leg_msg.data.clear();
		}
		for(int i=0;i<6;i++)
		{
			pre_leg_xyz_ypr_l[i] = leg_xyz_ypr_l[i];
			pre_leg_xyz_ypr_r[i] = leg_xyz_ypr_r[i];
		}
		pre_leg_trj_time = leg_trj_time;

		pre_waist_roll = waist_roll;
		pre_waist_yaw  = waist_yaw;
		pre_waist_roll_time = waist_roll_time;
		pre_waist_yaw_time  = waist_yaw_time;
	}
	else
		return;
}

void desired_leg_pose_carving()
{
	if(!turn_type.compare("carving_turn"))
	{
		if ((temp_change_value_center != change_value_center || temp_turn_type.compare(turn_type) || temp_change_type.compare(change_type)) && change_value_center !=0)
		{
			center_change_leg->parseMotionData("carving", "edge_change");
			center_change_waist->parseMotionData("carving", "center_change");

			if(change_value_center > 0)
			{
				center_change_leg->calculateStepEndPointValue(1,100,"edge_change"); // 0.01 단위로 조정 가능.
				center_change_waist->calculateStepEndPointValue(1,100,"center_change");
			}
			if(change_value_center < 0)
			{
				center_change_leg->calculateStepEndPointValue(-1,100,"edge_change"); // 0.01 단위로 조정 가능.
				center_change_waist->calculateStepEndPointValue(-1,100,"center_change");
			}
			for(int m = 3 ; m<6 ; m++)
			{
				leg_xyz_ypr_l[m] = center_change_leg->step_end_point_value[0][m];
				leg_xyz_ypr_r[m] = center_change_leg->step_end_point_value[1][m];
				leg_trj_time = time_edge_change;
			}

			waist_roll = center_change_waist->step_end_point_value[1];
			waist_roll_time =  time_edge_change;

			center_change_moving_check = true;
			motion_time_count_carving = 0;


			temp_turn_type   = turn_type;
			temp_change_type = change_type;
			temp_change_value_center = change_value_center;
			temp_time_center_change  = time_center_change;
			temp_time_edge_change    = time_edge_change;
			ROS_INFO("Turn !!  Change");
		} // 변한 것이 있으면 값을 계산

		if(center_change_moving_check == true && change_value_center !=0)
		{
			motion_time_count_carving  = motion_time_count_carving  + 0.008;

			if(motion_time_count_carving > time_edge_change)
			{

				center_change_leg->parseMotionData("carving", "center_change");
				center_change_leg->calculateStepEndPointValue(change_value_center,100,"center_change"); // 0.01 단위로 조정 가능.

				for(int m = 0 ; m<6 ; m++)
				{
					leg_xyz_ypr_l[m] = center_change_leg->step_end_point_value[0][m];
					leg_xyz_ypr_r[m] = center_change_leg->step_end_point_value[1][m];
					leg_trj_time = time_center_change;
				}
				center_change_moving_check = true;
				motion_time_count_carving = 0;

			}
		}

		if(change_value_center == 0)
		{

			center_change_waist->parseMotionData("carving", "center_change");
			center_change_waist->calculateStepEndPointValue(change_value_center,100,"center_change");

			center_change_leg->parseMotionData("carving", "center_change");
			center_change_leg->calculateStepEndPointValue(change_value_center,100,"center_change"); // 0.01 단위로 조정 가능.

			waist_roll = center_change_waist->step_end_point_value[1];
			waist_roll_time =  time_center_change;

			for(int m = 0 ; m<6 ; m++)
			{
				leg_xyz_ypr_l[m] = center_change_leg->step_end_point_value[0][m];
				leg_xyz_ypr_r[m] = center_change_leg->step_end_point_value[1][m];
				leg_trj_time = time_center_change;
			}
			center_change_moving_check = true;
			motion_time_count_carving = 0;
		}

		if(pre_leg_xyz_ypr_l[0] != leg_xyz_ypr_l[0] || pre_leg_xyz_ypr_l[1] != leg_xyz_ypr_l[1] || pre_leg_xyz_ypr_l[2] != leg_xyz_ypr_l[2] || pre_leg_xyz_ypr_l[3] != leg_xyz_ypr_l[3] ||
				pre_leg_xyz_ypr_l[4] != leg_xyz_ypr_l[4] || pre_leg_xyz_ypr_l[5] != leg_xyz_ypr_l[5] || pre_leg_xyz_ypr_r[0] != leg_xyz_ypr_r[0] || pre_leg_xyz_ypr_r[1] != leg_xyz_ypr_r[1] ||
				pre_leg_xyz_ypr_r[2] != leg_xyz_ypr_r[2] || pre_leg_xyz_ypr_r[3] != leg_xyz_ypr_r[3] || pre_leg_xyz_ypr_r[4] != leg_xyz_ypr_r[4] || pre_leg_xyz_ypr_r[5] != leg_xyz_ypr_r[5] ||
				pre_waist_roll != waist_roll             || pre_waist_yaw != waist_yaw               || pre_leg_trj_time != leg_trj_time         || pre_waist_roll_time != waist_roll_time   ||
				pre_waist_yaw_time != waist_yaw_time)
		{

			desired_pose_waist_msg.data.push_back(waist_yaw);
			desired_pose_waist_msg.data.push_back(waist_roll);
			desired_pose_waist_msg.data.push_back(waist_yaw_time);
			desired_pose_waist_msg.data.push_back(waist_roll_time);
			desired_pose_waist_pub.publish(desired_pose_waist_msg);
			desired_pose_waist_msg.data.clear();

			for(int m = 0 ; m<6 ; m++)
			{
				desired_pose_leg_msg.data.push_back(leg_xyz_ypr_l[m]);
			}
			for(int m = 0 ; m<6 ; m++)
			{
				desired_pose_leg_msg.data.push_back(leg_xyz_ypr_r[m]);
			}
			desired_pose_leg_msg.data.push_back(leg_trj_time);
			desired_pose_leg_pub.publish(desired_pose_leg_msg);
			desired_pose_leg_msg.data.clear();
		}
		for(int i=0;i<6;i++)
		{
			pre_leg_xyz_ypr_l[i] = leg_xyz_ypr_l[i];
			pre_leg_xyz_ypr_r[i] = leg_xyz_ypr_r[i];
		}
		pre_leg_trj_time = leg_trj_time;

		pre_waist_roll = waist_roll;
		pre_waist_yaw  = waist_yaw;
		pre_waist_roll_time = waist_roll_time;
		pre_waist_yaw_time  = waist_yaw_time;

	}
	else
		return;
}

void carving_motion_generate()
{

}










