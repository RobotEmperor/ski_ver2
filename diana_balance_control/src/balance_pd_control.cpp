/*
 * balance_pd_control.cpp
 *
 *  Created on: 2017. 10. 29.
 *      Author: Crowban
 */

#include "diana_balance_control/balance_pd_control.h"


using namespace diana;


BalancePDController::BalancePDController()
{
  control_cycle_sec_ = 0.008;

  desired_ = 0;

  p_gain_ = 0;
  d_gain_ = 0;

  curr_err_ = 0;
  prev_err_ = 0;
}

BalancePDController::~BalancePDController()
{  }


double BalancePDController::getFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;

  return (p_gain_*curr_err_ + d_gain_*(curr_err_ - prev_err_)/control_cycle_sec_);
}


