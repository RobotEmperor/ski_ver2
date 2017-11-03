/*
 * diana_balance_control.cpp
 *
 *  Created on: 2017. 10. 29.
 *      Author: Crowban
 */


#include "diana_balance_control/diana_balance_control.h"

using namespace diana;


BalanceControlUsingPDController::BalanceControlUsingPDController()
{
  balance_control_error_ = BalanceControlError::NoError;
  control_cycle_sec_ = 0.008;

  // balance enable
  gyro_enable_ = 1.0;

  // desired pose
  desired_robot_to_cob_        = Eigen::Matrix4d::Identity();
  desired_robot_to_right_foot_ = Eigen::Matrix4d::Identity();
  desired_robot_to_left_foot_  = Eigen::Matrix4d::Identity();

  // sensed values
  current_gyro_roll_rad_per_sec_  = 0;
  current_gyro_pitch_rad_per_sec_ = 0;

  // manual cob adjustment
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // result of balance control
  foot_roll_adjustment_by_gyro_roll_   = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;


  // sum of results of balance control
  pose_cob_adjustment_        = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_ = Eigen::VectorXd::Zero(6);
  pose_left_foot_adjustment_  = Eigen::VectorXd::Zero(6);

  mat_robot_to_cob_modified_        = Eigen::Matrix4d::Identity();
  mat_robot_to_right_foot_modified_ = Eigen::Matrix4d::Identity();
  mat_robot_to_left_foot_modified_  = Eigen::Matrix4d::Identity();

  // maximum adjustment
  cob_adjustment_abs_max_m_rad_  = Eigen::VectorXd::Zero(6);
  foot_adjustment_abs_max_m_rad_ = Eigen::VectorXd::Zero(6);

  cob_adjustment_abs_max_m_rad_.coeffRef(0) = foot_adjustment_abs_max_m_rad_.coeffRef(0) = 0.05;
  cob_adjustment_abs_max_m_rad_.coeffRef(1) = foot_adjustment_abs_max_m_rad_.coeffRef(1) = 0.05;
  cob_adjustment_abs_max_m_rad_.coeffRef(2) = foot_adjustment_abs_max_m_rad_.coeffRef(2) = 0.05;
  cob_adjustment_abs_max_m_rad_.coeffRef(3) = foot_adjustment_abs_max_m_rad_.coeffRef(3) = 15.0*M_PI/180.0;
  cob_adjustment_abs_max_m_rad_.coeffRef(4) = foot_adjustment_abs_max_m_rad_.coeffRef(4) = 15.0*M_PI/180.0;
  cob_adjustment_abs_max_m_rad_.coeffRef(5) = foot_adjustment_abs_max_m_rad_.coeffRef(5) = 15.0*M_PI/180.0;
}

BalanceControlUsingPDController::~BalanceControlUsingPDController()
{  }

void BalanceControlUsingPDController::initialize(const int control_cycle_msec)
{
  balance_control_error_ = BalanceControlError::NoError;

  control_cycle_sec_ = control_cycle_msec * 0.001;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  foot_roll_gyro_ctrl_.control_cycle_sec_  = control_cycle_sec_;
  foot_pitch_gyro_ctrl_.control_cycle_sec_ = control_cycle_sec_;
}

void BalanceControlUsingPDController::setGyroBalanceEnable(bool enable)
{
  if(enable)
    gyro_enable_ = 1.0;
  else
    gyro_enable_ = 0.0;
}

void BalanceControlUsingPDController::process(int *balance_error, Eigen::Matrix4d *robot_to_cob_modified, Eigen::Matrix4d *robot_to_right_foot_modified, Eigen::Matrix4d *robot_to_left_foot_modified)
{
  balance_control_error_ = BalanceControlError::NoError;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  calcGyroBalance();

  sumBalanceResults();

  checkBalanceLimit();

  if(balance_error != 0)
    *balance_error = balance_control_error_;

  Eigen::Matrix3d cob_rotation_adj = robotis_framework::getRotationZ(pose_cob_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_cob_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_cob_adjustment_.coeff(3));
  Eigen::Matrix3d rf_rotation_adj = robotis_framework::getRotationZ(pose_right_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_right_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_right_foot_adjustment_.coeff(3));
  Eigen::Matrix3d lf_rotation_adj = robotis_framework::getRotationZ(pose_left_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_left_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_left_foot_adjustment_.coeff(3));
  mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
  mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
  mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

  mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
  mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
  mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

  mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
  mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
  mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

  mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
  mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
  mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

  *robot_to_cob_modified        = mat_robot_to_cob_modified_;
  *robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
  *robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}

void BalanceControlUsingPDController::calcGyroBalance()
{
  foot_roll_adjustment_by_gyro_roll_   = -1.0*gyro_enable_*foot_roll_gyro_ctrl_.getFeedBack(current_gyro_roll_rad_per_sec_);
  foot_pitch_adjustment_by_gyro_pitch_ = -1.0*gyro_enable_*foot_pitch_gyro_ctrl_.getFeedBack(current_gyro_pitch_rad_per_sec_);
}

void BalanceControlUsingPDController::sumBalanceResults()
{
  //sum balcance results
  pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

  pose_right_foot_adjustment_.coeffRef(0) = 0;
  pose_right_foot_adjustment_.coeffRef(1) = 0;
  pose_right_foot_adjustment_.coeffRef(2) = 0;
  pose_right_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_gyro_roll_;
  pose_right_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_gyro_pitch_;

  pose_left_foot_adjustment_.coeffRef(0) = 0;
  pose_left_foot_adjustment_.coeffRef(1) = 0;
  pose_left_foot_adjustment_.coeffRef(2) = 0;
  pose_left_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_gyro_roll_;
  pose_left_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_gyro_pitch_;
}

void BalanceControlUsingPDController::checkBalanceLimit()
{
  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_adjustment_abs_max_m_rad_.coeff(0)), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_adjustment_abs_max_m_rad_.coeff(1)), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_adjustment_abs_max_m_rad_.coeff(2)), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_adjustment_abs_max_m_rad_.coeff(3)), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_adjustment_abs_max_m_rad_.coeff(4)), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5) = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_adjustment_abs_max_m_rad_.coeff(0)), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_adjustment_abs_max_m_rad_.coeff(1)), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_adjustment_abs_max_m_rad_.coeff(2)), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_adjustment_abs_max_m_rad_.coeff(3)), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_adjustment_abs_max_m_rad_.coeff(4)), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5) = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_adjustment_abs_max_m_rad_.coeff(0)), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_adjustment_abs_max_m_rad_.coeff(1)), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_adjustment_abs_max_m_rad_.coeff(2)), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_adjustment_abs_max_m_rad_.coeff(3)), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_adjustment_abs_max_m_rad_.coeff(4)), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5) = 0;

  // check limitation
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_adjustment_abs_max_m_rad_.coeff(0)) ||
      (fabs(pose_cob_adjustment_.coeff(1)) == cob_adjustment_abs_max_m_rad_.coeff(1)) ||
      (fabs(pose_cob_adjustment_.coeff(2)) == cob_adjustment_abs_max_m_rad_.coeff(2)) ||
      (fabs(pose_cob_adjustment_.coeff(3)) == cob_adjustment_abs_max_m_rad_.coeff(3)) ||
      (fabs(pose_cob_adjustment_.coeff(4)) == cob_adjustment_abs_max_m_rad_.coeff(4)) ||
      (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_adjustment_abs_max_m_rad_.coeff(0)) ||
      (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_adjustment_abs_max_m_rad_.coeff(1)) ||
      (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_adjustment_abs_max_m_rad_.coeff(2)) ||
      (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_adjustment_abs_max_m_rad_.coeff(3)) ||
      (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_adjustment_abs_max_m_rad_.coeff(4)) ||
      (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_adjustment_abs_max_m_rad_.coeff(0)) ||
      (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_adjustment_abs_max_m_rad_.coeff(1)) ||
      (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_adjustment_abs_max_m_rad_.coeff(2)) ||
      (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_adjustment_abs_max_m_rad_.coeff(3)) ||
      (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_adjustment_abs_max_m_rad_.coeff(4)))
    balance_control_error_ &= BalanceControlError::BalanceLimit;
}

void BalanceControlUsingPDController::setDesiredPose(const Eigen::Matrix4d &robot_to_cob, const Eigen::Matrix4d &robot_to_right_foot, const Eigen::Matrix4d &robot_to_left_foot)
{
  desired_robot_to_cob_        = robot_to_cob;
  desired_robot_to_right_foot_ = robot_to_right_foot;
  desired_robot_to_left_foot_  = robot_to_left_foot;
}

void BalanceControlUsingPDController::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)
{
  foot_roll_gyro_ctrl_.desired_  = gyro_roll;
  foot_pitch_gyro_ctrl_.desired_ = gyro_pitch;
}

// with respect to robot coordinate.
void BalanceControlUsingPDController::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}


// set maximum adjustment
void BalanceControlUsingPDController::setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
    double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
    double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
    double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  if((cob_x_max_adjustment_m       > 0) && (foot_x_max_adjustment_m       > 0) &&
      (cob_y_max_adjustment_m       > 0) && (foot_y_max_adjustment_m       > 0) &&
      (cob_z_max_adjustment_m       > 0) && (foot_z_max_adjustment_m       > 0) &&
      (cob_roll_max_adjustment_rad  > 0) && (foot_roll_max_adjustment_rad  > 0) &&
      (cob_pitch_max_adjustment_rad > 0) && (foot_pitch_max_adjustment_rad > 0) &&
      (cob_yaw_max_adjustment_rad   > 0) && (foot_yaw_max_adjustment_rad   > 0))
  {
    cob_adjustment_abs_max_m_rad_.coeffRef(0) = cob_x_max_adjustment_m;
    cob_adjustment_abs_max_m_rad_.coeffRef(1) = cob_y_max_adjustment_m;
    cob_adjustment_abs_max_m_rad_.coeffRef(2) = cob_z_max_adjustment_m;
    cob_adjustment_abs_max_m_rad_.coeffRef(3) = cob_roll_max_adjustment_rad;
    cob_adjustment_abs_max_m_rad_.coeffRef(4) = cob_pitch_max_adjustment_rad;
    cob_adjustment_abs_max_m_rad_.coeffRef(5) = cob_yaw_max_adjustment_rad;

    foot_adjustment_abs_max_m_rad_.coeffRef(0) = foot_x_max_adjustment_m;
    foot_adjustment_abs_max_m_rad_.coeffRef(1) = foot_y_max_adjustment_m;
    foot_adjustment_abs_max_m_rad_.coeffRef(2) = foot_z_max_adjustment_m;
    foot_adjustment_abs_max_m_rad_.coeffRef(3) = foot_roll_max_adjustment_rad;
    foot_adjustment_abs_max_m_rad_.coeffRef(4) = foot_pitch_max_adjustment_rad;
    foot_adjustment_abs_max_m_rad_.coeffRef(5) = foot_yaw_max_adjustment_rad;
  }
}

//Manual Adjustment
void BalanceControlUsingPDController::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}
