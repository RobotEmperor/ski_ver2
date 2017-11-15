/*
 * zmp_calculation_function.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: robotemperor
 */

#include "diana_balance_control/zmp_calculation_function.h"

using namespace diana;


ZmpCalculationFunc::ZmpCalculationFunc(){

	gf_zmp_point.resize(4,1);
	gf_zmp_point.fill(0);

	cf_zmp_point.resize(4,1);
	cf_zmp_point.fill(0);

	force_data_l.resize(4,1);
	force_data_l.fill(0);

	torque_data_l.resize(4,1);
	torque_data_l.fill(0);

	force_data_r.resize(4,1);
	force_data_r.fill(0);

	torque_data_r.resize(4,1);
	torque_data_r.fill(0);

	gf_px_l = 0;
	gf_py_l = 0;
	gf_pz_l = 0;

	gf_fx_l = 0;
	gf_fy_l = 0;
	gf_fz_l = 0;

	gf_tx_l = 0;
	gf_ty_l = 0;
	gf_tz_l = 0;

	gf_px_r = 0;
	gf_py_r = 0;
	gf_pz_r = 0;

	gf_fx_r = 0;
	gf_fy_r = 0;
	gf_fz_r = 0;

	gf_tx_r = 0;
	gf_ty_r = 0;
	gf_tz_r = 0;

}

ZmpCalculationFunc::~ZmpCalculationFunc()
{}

void ZmpCalculationFunc::ftSensorDataLeftGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z)
{

	force_data_l<< force_sensor_data_x, force_sensor_data_y, force_sensor_data_z, 1;
	torque_data_l<< torque_sensor_data_x, torque_sensor_data_y, torque_sensor_data_z, 1;

}
void ZmpCalculationFunc::ftSensorDataRightGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z)
{

	force_data_r<< force_sensor_data_x, force_sensor_data_y, force_sensor_data_z, 1;
	torque_data_r<< torque_sensor_data_x, torque_sensor_data_y, torque_sensor_data_z, 1;

}


void ZmpCalculationFunc::jointStateGetForTransForm(Eigen::MatrixXd joint_state_l, Eigen::MatrixXd joint_state_r)
{

	double joint_l[7] = {0,joint_state_l(1,0),joint_state_l(2,0),joint_state_l(3,0),joint_state_l(4,0),joint_state_l(5,0),joint_state_l(6,0)};
	double joint_r[7] = {0,joint_state_r(1,0),joint_state_r(2,0),joint_state_r(3,0),joint_state_r(4,0),joint_state_r(5,0),joint_state_r(6,0)};

	transformation_.FowardKnematicsGroundToSensorLeft(joint_l);
	transformation_.FowardKnematicsGroundToSensorRight(joint_r);

	force_data_l = transformation_.ground_to_sensor_transform_left*force_data_l;
	torque_data_l = transformation_.ground_to_sensor_transform_left*torque_data_l;

	gf_px_l = transformation_.ground_to_sensor_transform_left(0,3);
	gf_py_l = transformation_.ground_to_sensor_transform_left(1,3);
	gf_pz_l = transformation_.ground_to_sensor_transform_left(2,3);

	gf_fx_l = force_data_l(0,0);
	gf_fy_l = force_data_l(1,0);
	gf_fz_l = force_data_l(2,0);

	gf_tx_l = torque_data_l(0,0);
	gf_ty_l = torque_data_l(1,0);
	gf_tz_l = torque_data_l(2,0);

	force_data_r = transformation_.ground_to_sensor_transform_right*force_data_r;
	torque_data_r = transformation_.ground_to_sensor_transform_right*torque_data_r;

	gf_px_r = transformation_.ground_to_sensor_transform_right(0,3);
	gf_py_r = transformation_.ground_to_sensor_transform_right(1,3);
	gf_pz_r = transformation_.ground_to_sensor_transform_right(2,3);

	gf_fx_r = force_data_r(0,0);
	gf_fy_r = force_data_r(1,0);
	gf_fz_r = force_data_r(2,0);

	gf_tx_r = torque_data_r(0,0);
	gf_ty_r = torque_data_r(1,0);
	gf_tz_r = torque_data_r(2,0);
}

void ZmpCalculationFunc::zmpCalculationResult()
{
	gf_zmp_point(0,0) = (-gf_ty_r - gf_pz_r*gf_fx_r + gf_px_r*gf_tz_r -gf_ty_l - gf_pz_l*gf_fx_l + gf_px_l*gf_tz_l)/(gf_tz_r + gf_tz_l);
	gf_zmp_point(1,0) = (-gf_ty_r - gf_pz_r*gf_fx_r + gf_px_r*gf_tz_r -gf_ty_l - gf_pz_l*gf_fx_l + gf_px_l*gf_tz_l)/(gf_tz_r + gf_tz_l);
	gf_zmp_point(2,0) = 0;
	gf_zmp_point(3,0) = 1;

	cf_zmp_point = transformation_.CenterToGroundTransformation(gf_zmp_point);

}
