/*
 * heroehs_manager1_node.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: robotemperor
 */

#include "robotis_controller/robotis_controller.h"
#include "offset_module/offset_module.h"
#include "base_module/base_module.h"
#include "pose_module/pose_module.h"
#include "motion_module/motion_module.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Heroehs_ski_main_manager_ver2");
  ros::NodeHandle nh;
  ROS_INFO("ski_main_manager_ver2->init !! start!");
  robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

  controller->DEBUG_PRINT = true;
  /* Load ROS Parameter */
  std::string offset_file = nh.param<std::string>("offset_table", "");
  std::string robot_file  = nh.param<std::string>("robot_file_path", "");
  std::string init_file   = nh.param<std::string>("init_file_path", "");

  /* gazebo simulation */
  controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
  if (controller->gazebo_mode_ == true)
  {
    std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;

    ROS_WARN("GAZEBO_MODE!!!!!!!!!!!!!!!");
  }

  base_module::BaseModule::getInstance()->gazebo_check = controller->gazebo_mode_;
  pose_module::PoseModule::getInstance()->gazebo_check = controller->gazebo_mode_;
  motion_module::MotionModule::getInstance()->gazebo_check = controller->gazebo_mode_;

  if (robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  if (controller->initialize(robot_file, init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  if (offset_file != "")
    controller->loadOffset(offset_file);

  sleep(1);

  /* Add Motion Module */
  controller->addMotionModule((robotis_framework::MotionModule*) offset_module::OffsetModule::getInstance());
  controller->addMotionModule((robotis_framework::MotionModule*) base_module::BaseModule::getInstance());
  controller->addMotionModule((robotis_framework::MotionModule*) pose_module::PoseModule::getInstance());
  controller->addMotionModule((robotis_framework::MotionModule*) motion_module::MotionModule::getInstance());

  controller->startTimer();



  while (ros::ok())
  {
    usleep(1000*1000);
  }

  return 0;


}
