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

using namespace Eigen;
using namespace std;


namespace diana
{

class CopCalculationFunc
{

public:
	CopCalculationFunc();
	~CopCalculationFunc();
	void ftSensorDataLeftGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void ftSensorDataRightGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void jointStateGetForTransForm(Eigen::MatrixXd joint_state_l, Eigen::MatrixXd joint_state_r);
	void copCalculationResult();

	double cop_fz_point_x, cop_fz_point_y, cop_fy_point_x, cop_fy_point_z, cop_fx_point_y, cop_fx_point_z;

private:
	Eigen::MatrixXd force_data_l, torque_data_l;
	Eigen::MatrixXd force_data_r, torque_data_r;
	heroehs_math::Kinematics transformation_;
	double cf_fx_l,cf_fy_l,cf_fz_l,cf_tx_l,cf_ty_l,cf_tz_l; // center frame cf
	double cf_fx_r,cf_fy_r,cf_fz_r,cf_tx_r,cf_ty_r,cf_tz_r; // center frame cf
	double cf_px_l,cf_py_l,cf_pz_l;
	double cf_px_r,cf_py_r,cf_pz_r;


};


}



#endif /* SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_ZMP_CALCULATION_FUNCTION_H_ */
