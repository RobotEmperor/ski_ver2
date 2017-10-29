/*
 * balance_pid_control.h
 *
 *  Created on: 2017. 10. 29.
 *      Author: Crowban
 */

#ifndef DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PID_CONTROL_H_
#define DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PID_CONTROL_H_

namespace diana
{

class BalancePIDController
{
public:
  BalancePIDController();
  ~BalancePIDController();

  double control_cycle_sec_;

  double desired_;

  double p_gain_;
  double i_gain_;
  double d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double curr_err_;
  double prev_err_;
  double sum_err_;
};


}


#endif /* DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PID_CONTROL_H_ */
