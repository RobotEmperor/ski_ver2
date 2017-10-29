/*
 * balance_pd_control.h
 *
 *  Created on: 2017. 10. 29.
 *      Author: Crowban
 */

#ifndef DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_
#define DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_

namespace diana
{

class BalancePDController
{
public:
  BalancePDController();
  ~BalancePDController();

  double control_cycle_sec_;

  double desired_;

  double p_gain_;
  double d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double curr_err_;
  double prev_err_;
};

}



#endif /* DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_BALANCE_PD_CONTROL_H_ */
