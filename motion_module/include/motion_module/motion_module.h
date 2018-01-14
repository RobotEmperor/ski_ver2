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
	ros::Publisher current_leg_pose_pub;
	ros::Publisher l_leg_point_xyz_pub;
	ros::Publisher l_leg_point_rpy_pub;
	ros::Publisher r_leg_point_xyz_pub;
	ros::Publisher r_leg_point_rpy_pub;

	ros::Publisher l_compensation_xyz_pub;
	ros::Publisher l_compensation_rpy_pub;
	ros::Publisher r_compensation_xyz_pub;
	ros::Publisher r_compensation_rpy_pub;

	ros::Publisher cop_fz_pub;


	// sensor data & balance on off
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber get_ft_data_sub_;
	ros::Subscriber set_balance_param_sub_;

	/* ROS Topic Callback Functions */
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);
	void setBalanceParameterCallback(const diana_msgs::BalanceParam::ConstPtr& msg);
	void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	int new_count_;


	// leg state
	std_msgs::Float64MultiArray current_leg_pose_msg_;
	geometry_msgs::Vector3 l_leg_point_xyz_msg_;
	geometry_msgs::Vector3 l_leg_point_rpy_msg_;
	geometry_msgs::Vector3 r_leg_point_xyz_msg_;
	geometry_msgs::Vector3 r_leg_point_rpy_msg_;

	geometry_msgs::Vector3 l_compensation_xyz_msg_;
	geometry_msgs::Vector3 l_compensation_rpy_msg_;
	geometry_msgs::Vector3 r_compensation_xyz_msg_;
	geometry_msgs::Vector3 r_compensation_rpy_msg_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	double traj_time_;

	bool is_moving_l_;
	bool is_moving_r_;
	bool is_moving_one_joint_;

	Eigen::MatrixXd leg_end_point_l_;
	Eigen::MatrixXd leg_end_point_r_;

	heroehs_math::Kinematics *l_kinematics_;
	heroehs_math::Kinematics *r_kinematics_;
	heroehs_math::CalRad *end_to_rad_l_;
	heroehs_math::CalRad *end_to_rad_r_;

	// balance gyro
	Eigen::MatrixXd result_end_l_;
	Eigen::MatrixXd result_end_r_;
	Eigen::Matrix4d result_mat_cob_, result_mat_cob_modified_;
	Eigen::Matrix4d result_mat_l_, result_mat_l_modified_;
	Eigen::Matrix4d result_mat_r_, result_mat_r_modified_;
	robotis_framework::Pose3D result_pose_l_modified_;
	robotis_framework::Pose3D result_pose_r_modified_;

	diana::BalanceControlUsingPDController balance_ctrl_;
	void updateBalanceParameter();
	void gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x);
	double currentGyroX,currentGyroY,currentGyroZ;
	double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;
	diana_msgs::BalanceParam previous_balance_param_, desired_balance_param_;
	robotis_framework::FifthOrderPolynomialTrajectory balance_param_update_coeff_;
	double balance_updating_duration_sec_;
	double balance_updating_sys_time_sec_;
	bool balance_update_;

	// cop
	diana::CopCalculationFunc *cop_cal;
	double currentFX_l,currentFY_l,currentFZ_l,currentTX_l,currentTY_l,currentTZ_l;
	double currentFX_r,currentFY_r,currentFZ_r,currentTX_r,currentTY_r,currentTZ_r;
	std_msgs::Float64MultiArray cop_fz_msg_;

	//cop compensation
	diana::CopCompensationFunc *cop_compensation;
	heroehs_math::FifthOrderTrajectory *gain_copFz_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_copFz_d_adjustment;
	double updating_duration_cop;
	double copFz_p_gain;
	double copFz_d_gain;

	//center change lib
	diana_motion::CenterChange *center_change_;
	double temp_change_value_edge, temp_change_value_center;
	std::string temp_turn_type;
	std::string temp_change_type;

	void motion();
	double change_value_center;
	double change_value_edge;
	double time_center;
	double time_edge;
	std::string turn_type;
	std::string change_type;
	double motion_time_count_center;
	double motion_time_count_edge;
	double motion_count;

	double pattern_count;

	bool read_data;
};

}



