/*
 * zmp_calculation_function.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: robotemperor
 */

#include "diana_balance_control/cop_calculation_function.h"

using namespace diana;


CopCalculationFunc::CopCalculationFunc(){

	force_data_l.resize(4,1);
	force_data_l.fill(0);

	torque_data_l.resize(4,1);
	torque_data_l.fill(0);

	force_data_r.resize(4,1);
	force_data_r.fill(0);

	torque_data_r.resize(4,1);
	torque_data_r.fill(0);

	cf_px_l = 0;
	cf_py_l = 0;
	cf_pz_l = 0;

	cf_fx_l = 0;
	cf_fy_l = 0;
	cf_fz_l = 0;

	cf_tx_l = 0;
	cf_ty_l = 0;
	cf_tz_l = 0;

	cf_px_r = 0;
	cf_py_r = 0;
	cf_pz_r = 0;

	cf_fx_r = 0;
	cf_fy_r = 0;
	cf_fz_r = 0;

	cf_tx_r = 0;
	cf_ty_r = 0;
	cf_tz_r = 0;

	cop_fz_point_x = 0;
	cop_fz_point_y = 0;

	cop_fy_point_x = 0;
	cop_fy_point_z = 0;

	cop_fx_point_y = 0;
	cop_fx_point_z = 0;


}

CopCalculationFunc::~CopCalculationFunc()
{}

void CopCalculationFunc::ftSensorDataLeftGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z)
{

	force_data_l << force_sensor_data_x, force_sensor_data_y, force_sensor_data_z, 1;
	torque_data_l << torque_sensor_data_x, torque_sensor_data_y, torque_sensor_data_z, 1;

}
void CopCalculationFunc::ftSensorDataRightGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z)
{

	force_data_r << force_sensor_data_x, force_sensor_data_y, force_sensor_data_z, 1;
	torque_data_r << torque_sensor_data_x, torque_sensor_data_y, torque_sensor_data_z, 1;

}


void CopCalculationFunc::jointStateGetForTransForm(Eigen::MatrixXd joint_state_l, Eigen::MatrixXd joint_state_r)
{

	double joint_l[7] = {0,joint_state_l(1,0),joint_state_l(2,0),joint_state_l(3,0),joint_state_l(4,0),joint_state_l(5,0),joint_state_l(6,0)};
	double joint_r[7] = {0,joint_state_r(1,0),joint_state_r(2,0),joint_state_r(3,0),joint_state_r(4,0),joint_state_r(5,0),joint_state_r(6,0)};

	transformation_.FowardKnematicsCenterToSensorLeft(joint_l);
	transformation_.FowardKnematicsCenterToSensorRight(joint_r);

	cf_px_l = transformation_.center_to_sensor_transform_left(0,3);
	cf_py_l = transformation_.center_to_sensor_transform_left(1,3);
	cf_pz_l = transformation_.center_to_sensor_transform_left(2,3);

	cf_fx_l = force_data_l(0,0);
	cf_fy_l = force_data_l(1,0);
	cf_fz_l = force_data_l(2,0);

	cf_tx_l = torque_data_l(0,0);
	cf_ty_l = torque_data_l(1,0);
	cf_tz_l = torque_data_l(2,0);

	cf_px_r = transformation_.center_to_sensor_transform_right(0,3);
	cf_py_r = transformation_.center_to_sensor_transform_right(1,3);
	cf_pz_r = transformation_.center_to_sensor_transform_right(2,3);

	cf_fx_r = force_data_r(0,0);
	cf_fy_r = force_data_r(1,0);
	cf_fz_r = force_data_r(2,0);

	cf_tx_r = torque_data_r(0,0);
	cf_ty_r = torque_data_r(1,0);
	cf_tz_r = torque_data_r(2,0);


}

void CopCalculationFunc::copCalculationResult()
{
	if(cf_fz_l + cf_fz_r == 0 || cf_fy_l + cf_fy_r == 0 || cf_fx_l + cf_fx_r == 0)
		return;

	cop_fz_point_x = (- cf_px_l*cf_fz_l - cf_px_r*cf_fz_r + cf_ty_l + cf_ty_r )/(cf_fz_l + cf_fz_r);
	cop_fz_point_y = (  cf_py_l*cf_fz_l + cf_py_r*cf_fz_r + cf_tx_l + cf_tx_r )/(cf_fz_l + cf_fz_r);

	cop_fy_point_x = (  cf_px_l*cf_fy_l + cf_px_r*cf_fy_r + cf_tz_l + cf_tz_r )/(cf_fy_l + cf_fy_r);
	cop_fy_point_z = (  cf_pz_l*cf_fy_l + cf_pz_r*cf_fy_r + cf_tx_l + cf_tx_r )/(cf_fy_l + cf_fy_r);

	cop_fx_point_y = (- cf_py_l*cf_fx_l - cf_py_r*cf_fx_r + cf_tz_l + cf_tz_r )/(cf_fx_l + cf_fx_r);
	cop_fx_point_z = (  cf_pz_l*cf_fx_l + cf_pz_r*cf_fx_r + cf_ty_l + cf_ty_r )/(cf_fx_l + cf_fx_r);




	printf("Fz  X  %f ::Y  %f  \n\n", cop_fz_point_x, cop_fz_point_x);
	printf("Fy  X  %f ::z  %f  \n\n", cop_fy_point_x, cop_fy_point_z);
	printf("Fx  Y  %f ::z  %f  \n\n", cop_fx_point_y, cop_fx_point_z);


}
