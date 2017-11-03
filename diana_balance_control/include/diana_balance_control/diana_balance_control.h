/*
 * diana_balance_control.h
 *
 *  Created on: 2017. 10. 29.
 *      Author: Crowban
 */

#ifndef DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_DIANA_BALANCE_CONTROL_H_
#define DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_DIANA_BALANCE_CONTROL_H_

#include "robotis_math/robotis_math.h"

#include "balance_pd_control.h"
#include "balance_pid_control.h"

namespace diana
{

class BalanceControlError
{
public:
  static const int NoError = 0;
  static const int BalanceLimit = 2;
};


class BalanceControlUsingPDController
{
public:
  BalanceControlUsingPDController();
  ~BalanceControlUsingPDController();

  void initialize(const int control_cycle_msec);

  void setGyroBalanceEnable(bool enable);

  void process(int *balance_error, Eigen::Matrix4d *robot_to_cob_modified, Eigen::Matrix4d *robot_to_right_foot_modified, Eigen::Matrix4d *robot_to_left_foot_modified);

  void calcGyroBalance();

  void sumBalanceResults();

  void checkBalanceLimit();

  // all arguments are with respect to robot coordinate.
  void setDesiredPose(const Eigen::Matrix4d &robot_to_cob, const Eigen::Matrix4d &robot_to_right_foot, const Eigen::Matrix4d &robot_to_left_foot);
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);

  // with respect to robot coordinate.
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);


  // set maximum adjustment
  void setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
      double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
      double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
      double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  //Manual Adjustment
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

private:
  BalancePDController foot_roll_gyro_ctrl_;
  BalancePDController foot_pitch_gyro_ctrl_;

  int balance_control_error_;
  double control_cycle_sec_;

  // balance enable
  double gyro_enable_;

  // desired pose
  Eigen::Matrix4d desired_robot_to_cob_;
  Eigen::Matrix4d desired_robot_to_right_foot_;
  Eigen::Matrix4d desired_robot_to_left_foot_;

  // sensed values
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  // manual cob adjustment
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // result of balance control
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;


  // sum of results of balance control
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::Matrix4d mat_robot_to_cob_modified_;
  Eigen::Matrix4d mat_robot_to_right_foot_modified_;
  Eigen::Matrix4d mat_robot_to_left_foot_modified_;

  // maximum adjustment
  Eigen::VectorXd cob_adjustment_abs_max_m_rad_;
  Eigen::VectorXd foot_adjustment_abs_max_m_rad_;

};

}

#endif /* DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_DIANA_BALANCE_CONTROL_H_ */
