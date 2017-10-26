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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

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



private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	bool is_moving_l_;
	bool is_moving_r_;

	double desired_waist_roll_;
	double traj_time_;

	Eigen::MatrixXd leg_end_point_l_;
	Eigen::MatrixXd leg_end_point_r_;

	heroehs_math::CalRad *end_to_rad_l_;
	heroehs_math::CalRad *end_to_rad_r_;

	Eigen::MatrixXd result_rad_l_;
	Eigen::MatrixXd result_rad_r_;



};

}
