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

  cnt_state = 0;
  MAX_JOINT_ID_STATE = 0;

  mov_time_state  = 1.0;
  smp_time_state  = 0.008;
  all_time_steps_state  = int(mov_time_state / smp_time_state);

  joint_ini_pose_state  = Eigen::MatrixXd::Zero( MAX_JOINT_ID_STATE + 1, 1);

}

BaseModuleState::~BaseModuleState()
{
}

