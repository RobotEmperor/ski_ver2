/*
 * pose_module.h
 *
 *  Created on: Oct 24, 2017
 *      Author: robotemperor
 */
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <pose_module/command.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"



#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"
#include "heroehs_math/end_point_to_rad_cal.h"
#include "diana_balance_control/cop_calculation_function.h"



namespace pose_module
{

class PoseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<PoseModule>
{
public:
	PoseModule();
	virtual ~PoseModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;

	/* ROS Topic Callback Functions */
	void desiredPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseArmMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void gainAdjustmentMsgCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
	void finalGainSaveMsgCallback(const std_msgs::Bool::ConstPtr& msg);
	bool readPgainSrvFunction(pose_module::command::Request  &req, pose_module::command::Response &res);
	void parsePgainValue(std::string joint_name_);
	void savePgainValue();
	ros::ServiceServer read_p_gain_value_srv;


private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	int new_count_;

	bool is_moving_l_;
	bool is_moving_r_;
	bool is_moving_waist;
	bool is_moving_head;
	bool is_moving_l_arm;
	bool is_moving_r_arm;

	double traj_time_;

	// leg kinematics
	Eigen::MatrixXd leg_end_point_l_;
	Eigen::MatrixXd leg_end_point_r_;
	Eigen::MatrixXd one_joint_ctrl_;

	heroehs_math::CalRad *end_to_rad_l_;
	heroehs_math::CalRad *end_to_rad_r_;


	heroehs_math::Kinematics *l_kinematics_;
	heroehs_math::Kinematics *r_kinematics_;

	Eigen::MatrixXd result_end_l_;
	Eigen::MatrixXd result_end_r_;

	//waist kinematics
	heroehs_math::KinematicsEulerAngle *waist_kinematics_;
	heroehs_math::CalRad *end_to_rad_waist_;
	Eigen::MatrixXd waist_end_point_;
	Eigen::MatrixXd result_end_waist_;

	//head kinematics
	heroehs_math::KinematicsEulerAngle *head_kinematics_;
	heroehs_math::CalRad *end_to_rad_head_;
	Eigen::MatrixXd head_end_point_;
	Eigen::MatrixXd result_end_head_;

	//Arm kinematics end point control
	heroehs_math::KinematicsArm *l_arm_kinematics_;
	heroehs_math::CalRad *end_to_rad_l_arm_;
	Eigen::MatrixXd l_arm_end_point_;
	Eigen::MatrixXd result_end_l_arm_;

	heroehs_math::KinematicsArm *r_arm_kinematics_;
	heroehs_math::CalRad *end_to_rad_r_arm_;
	Eigen::MatrixXd r_arm_end_point_;
	Eigen::MatrixXd result_end_r_arm_;

	//Arm kinematics joint control
	//one joint control
	//heroehs_math::CalRad *one_joint_;
	//double result_rad_one_joint_;

	// dxl gain
	int p_gain_data_[30];
	int id_select;
	bool p_gain_adjust_check;

	//cop experiment
	diana::CopCalculationFunc cop_cal;
};

}
