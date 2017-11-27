/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/atest_gui/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace atest_gui {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	    		init_argc(argc),
					init_argv(argv)
{}

QNode::~QNode() {
	if(ros::isStarted()) {

		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}

	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"atest_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;


	start();

	// Add your ros communications here.

	module_on_off = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 100);


	// off set moudle topic definition //

	joint_select_pub = n.advertise<std_msgs::Int8>("/joint_select",100);
	change_joint_value_pub =n.advertise<std_msgs::Int16MultiArray>("/change_joint_value",100);

	read_joint_value_client = n.serviceClient<atest_gui::command>("/read_joint_value");

	save_onoff_pub = n.advertise<std_msgs::Bool>("/save_onoff",100);
	offset_joint_value_pub =n.advertise<std_msgs::Float64MultiArray>("/offset_joint_value",100);

	//base module //
	init_pose_pub = n.advertise<std_msgs::String>("/init_pose",100);

	//pose module //
	desired_pose_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose",100);
	read_p_gain_value_client = n.serviceClient<atest_gui::command>("/read_p_gain_value");

	//motion_module //
	pattern_pub = n.advertise<std_msgs::Int32>("/motion_num",100);

	//balance on off
	diana_balance_parameter_pub = n.advertise<diana_msgs::BalanceParam>("/diana/balance_parameter",100);

	//center change
	center_change_pub = n.advertise<diana_msgs::CenterChange>("/diana/center_change",100);

	/////////////////////////////////////

	// sensor data //
	imu_data_sub = n.subscribe("/imu/data", 100, &QNode::imuDataMsgCallback, this);
	diana_force_torque_data_sub = n.subscribe("/diana/force_torque_data", 100, &QNode::forceTorqueDataMsgCallback, this);



	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
	return true;
}
void QNode::run() {
	while ( ros::ok() ) {

		ros::spinOnce();

	}
}

void QNode::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	currentGyroX_gui = (double) msg->angular_velocity.x;
	currentGyroY_gui = (double) -msg->angular_velocity.y;
	currentGyroZ_gui = (double) msg->angular_velocity.z;
}

void QNode::forceTorqueDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)
{
	currentForceX_l_gui = (double) msg->force_x_raw_l;
	currentForceY_l_gui = (double) msg->force_y_raw_l;
	currentForceZ_l_gui = (double) msg->force_z_raw_l;

	currentForceX_r_gui = (double) msg->force_x_raw_r;
	currentForceY_r_gui = (double) msg->force_y_raw_r;
	currentForceZ_r_gui = (double) msg->force_z_raw_r;

	currentTorqueX_l_gui = (double) msg->torque_x_raw_l;
	currentTorqueY_l_gui = (double) msg->torque_y_raw_l;
	currentTorqueZ_l_gui = (double) msg->torque_z_raw_l;

	currentTorqueX_r_gui = (double) msg->torque_x_raw_r;
	currentTorqueY_r_gui = (double) msg->torque_y_raw_r;
	currentTorqueZ_r_gui = (double) msg->torque_z_raw_r;
}

}  // namespace atest_gui
