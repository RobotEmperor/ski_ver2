/*
 * base_module_state.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "base_module/base_module_state.h"

using namespace base_module_state;

BaseModuleState::BaseModuleState()
{
  is_moving_state= false;
  mov_time_state =0.0;
  MAX_JOINT_ID_STATE = 0;

  joint_ini_pose_state  = Eigen::MatrixXd::Zero( MAX_JOINT_ID_STATE + 1, 1);
  joint_ini_pose_goal  = Eigen::MatrixXd::Zero( MAX_JOINT_ID_STATE + 1, 1);

}

BaseModuleState::~BaseModuleState()
{
}

