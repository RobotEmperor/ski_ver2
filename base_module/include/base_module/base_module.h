/*
 * base_module.h
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
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

#include "base_module/base_module_state.h"

#include "heroehs_math/fifth_order_trajectory_generate.h"

#include "heroehs_math/kinematics.h"

using namespace base_module_state;
using namespace heroehs_math;

namespace base_module
{

class BaseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  /* ROS Topic Callback Functions */

  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);


  BaseModuleState *base_module_state;

  FifthOrderTrajectory *motion_trajectory_11;
  FifthOrderTrajectory *motion_trajectory_13;


  //*motion_trajectory;


private:
  void queueThread();
  void parse_init_pose_data_(const std::string &path);
  void init_pose_trajectory_();
  bool running_;

  int control_cycle_msec_;

  boost::thread queue_thread_;

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;




};

}


