/*
 * motion_module.h
 *
 *  Created on: Oct 27, 2017
 *      Author: robotemperor
 */
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"
//library
#include "robotis_math/robotis_math.h"
#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"
#include "heroehs_math/end_point_to_rad_cal.h"
#include "diana_balance_control/diana_balance_control.h"
#include "motion_module/center_change_lib.h"
#include "diana_balance_control/cop_calculation_function.h"

//message
//m - standard
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

//m - personal
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/ForceTorque.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "diana_msgs/CenterChange.h"

using namespace diana;

namespace motion_module
{

class MotionModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<MotionModule>
{
public:
	MotionModule();
	virtual ~MotionModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;

	// paper messages
	ros::Publisher state_end_point_pose_pub;
	ros::Publisher state_end_point_orientation_pub;
	ros::Publisher cop_point_Fz_pub;
	ros::Publisher cop_point_Fy_pub;
	ros::Publisher cop_point_Fx_pub;

	// sensor data & balance on off
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber get_ft_data_sub_;
  ros::Subscriber set_balance_param_sub_;

	/* ROS Topic Callback Functions */
	void desiredMotionMsgCallback(const std_msgs::Int32::ConstPtr& msg);
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);
  void setBalanceParameterCallback(const diana_msgs::BalanceParam::ConstPtr& msg);
  void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	int new_count_;

	geometry_msgs::Vector3 state_end_point_pose_msg_;
	geometry_msgs::Vector3 state_end_point_orientation_msg_;
	geometry_msgs::PointStamped cop_point_Fz_msg_;
	geometry_msgs::PointStamped cop_point_Fy_msg_;
	geometry_msgs::PointStamped cop_point_Fx_msg_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	void parse_motion_data_(int motion_num_); // 모션 데이터 읽어온다.
	void motion_vel_cal_leg_(int pose_num_ , int node_num_); // 속도 계산
	void motion_generater_(); // 모션 생성
	int motion_command_, pre_motion_command_ , motion_seq_;
	int pose_;
	double current_time_;
	Eigen::MatrixXd change_desired_pose_;
	Eigen::MatrixXd change_desired_final_vel_;
	Eigen::MatrixXd change_desired_initial_vel_;
	Eigen::MatrixXd change_desired_time_;

	double traj_time_;

	bool is_moving_l_;
	bool is_moving_r_;
	bool is_moving_one_joint_;

	Eigen::MatrixXd leg_end_point_l_;
	Eigen::MatrixXd leg_end_point_r_;
	Eigen::MatrixXd one_joint_ctrl_;

	heroehs_math::Kinematics *l_kinematics_;
	heroehs_math::Kinematics *r_kinematics_;
	heroehs_math::CalRad *end_to_rad_l_;
	heroehs_math::CalRad *end_to_rad_r_;
	heroehs_math::CalRad *one_joint_;

	Eigen::MatrixXd result_end_l_;
	Eigen::MatrixXd result_end_r_;
	Eigen::Matrix4d result_mat_cob_, result_mat_cob_modified_;
	Eigen::Matrix4d result_mat_l_, result_mat_l_modified_;
  Eigen::Matrix4d result_mat_r_, result_mat_r_modified_;
  robotis_framework::Pose3D result_pose_l_modified_;
  robotis_framework::Pose3D result_pose_r_modified_;

	double result_rad_one_joint_;
  diana::BalanceControlUsingPDController balance_ctrl_;
  double currentGyroX,currentGyroY,currentGyroZ;

  // balance gyro
  void updateBalanceParameter();
  diana_msgs::BalanceParam previous_balance_param_, desired_balance_param_;
  robotis_framework::FifthOrderPolynomialTrajectory balance_param_update_coeff_;
  double balance_updating_duration_sec_;
  double balance_updating_sys_time_sec_;
  bool balance_update_;

  // zmp
  diana::CopCalculationFunc *cop_cal;
  double currentFX_l,currentFY_l,currentFZ_l,currentTX_l,currentTY_l,currentTZ_l;
  double currentFX_r,currentFY_r,currentFZ_r,currentTX_r,currentTY_r,currentTZ_r;

  //center change lib
  diana_motion::CenterChange *center_change_;
  double temp_change_value_edge, temp_change_value_center;
  std::string temp_turn_type;
  std::string temp_change_type;
};

}



