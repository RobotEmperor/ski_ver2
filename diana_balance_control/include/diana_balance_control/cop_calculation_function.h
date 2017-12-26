/*
 * zmp_calculation_function.h
 *
 *  Created on: Nov 15, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_ZMP_CALCULATION_FUNCTION_H_
#define SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_ZMP_CALCULATION_FUNCTION_H_


#include <ros/ros.h>
#include <Eigen/Dense>
#include "robotis_math/robotis_math.h"
#include "heroehs_math/kinematics.h"
#include "diana_balance_control/control_function.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

namespace diana
{

class CopCalculationFunc
{

public:
	CopCalculationFunc();
	~CopCalculationFunc();
	// cop calculation variables
	void ftSensorDataLeftGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void ftSensorDataRightGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void jointStateGetForTransForm(Eigen::MatrixXd joint_state_l, Eigen::MatrixXd joint_state_r);
	void copCalculationResult();
	double cop_fz_point_x, cop_fz_point_y, cop_fy_point_x, cop_fy_point_z, cop_fx_point_y, cop_fx_point_z;

private:
	// cop calculation variables
	Eigen::MatrixXd force_data_l, torque_data_l;
	Eigen::MatrixXd force_data_r, torque_data_r;
	heroehs_math::Kinematics transformation_;
	double cf_fx_l,cf_fy_l,cf_fz_l,cf_tx_l,cf_ty_l,cf_tz_l; // center frame cf
	double cf_fx_r,cf_fy_r,cf_fz_r,cf_tx_r,cf_ty_r,cf_tz_r; // center frame cf
	double cf_px_l,cf_py_l,cf_pz_l;
	double cf_px_r,cf_py_r,cf_pz_r;





};
class CopCompensationFunc
{
public:
	CopCompensationFunc();
	~CopCompensationFunc();

	//cop compensation function
	void   parse_margin_data();
	void   centerOfPressureReferencePoint(std::string turn_type, double cur_l_point_x, double cur_l_point_y, double cur_l_point_z, double cur_r_point_x, double cur_r_point_y, double cur_r_point_z, double current_control_value);
	void centerOfPressureCompensationFz(double current_point_x, double current_point_y, double current_control_value);
	void centerOfPressureCompensationFy(double current_point_x, double current_point_z, double current_control_value);
	void centerOfPressureCompensationFx(double current_point_y, double current_point_z, double current_control_value);

	double reference_point_Fz_x , reference_point_Fz_y;
	double reference_point_Fy_x , reference_point_Fy_z;
	double reference_point_Fx_y , reference_point_Fx_z;

private:
	// cop compensation variables
	control_function::PID_function *pidControllerFz_x;
	control_function::PID_function *pidControllerFz_y;
	control_function::PID_function *pidControllerFx;
	control_function::PID_function *pidControllerFy;

	double margin_pflug_bogen_l_fz_x,  margin_pflug_bogen_r_fz_x,  margin_pflug_bogen_l_fy_x,  margin_pflug_bogen_r_fy_x,  margin_pflug_bogen_l_fx_z,  margin_pflug_bogen_r_fx_z;
	double margin_pflug_bogen_l_fz_y,  margin_pflug_bogen_r_fz_y,  margin_pflug_bogen_l_fy_z,  margin_pflug_bogen_r_fy_z,  margin_pflug_bogen_l_fx_y,  margin_pflug_bogen_r_fx_y;
	double margin_carving_turn_l_fz_x, margin_carving_turn_r_fz_x, margin_carving_turn_l_fy_x, margin_carving_turn_r_fy_x, margin_carving_turn_l_fx_z, margin_carving_turn_r_fx_z;
	double margin_carving_turn_l_fz_y, margin_carving_turn_r_fz_y, margin_carving_turn_l_fy_z, margin_carving_turn_r_fy_z, margin_carving_turn_l_fx_y, margin_carving_turn_r_fx_y;
};


}



#endif /* SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_ZMP_CALCULATION_FUNCTION_H_ */
