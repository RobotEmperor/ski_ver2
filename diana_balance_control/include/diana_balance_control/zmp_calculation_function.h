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

class ZmpCalculationFunc
{

public:
	ZmpCalculationFunc();
	~ZmpCalculationFunc();
	void ftSensorDataLeftGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void ftSensorDataRightGet(double force_sensor_data_x, double force_sensor_data_y , double force_sensor_data_z , double torque_sensor_data_x, double torque_sensor_data_y, double torque_sensor_data_z);
	void jointStateGetForTransForm(Eigen::MatrixXd joint_state_l, Eigen::MatrixXd joint_state_r);
	void zmpCalculationResult();


	Eigen::MatrixXd gf_zmp_point;
	Eigen::MatrixXd cf_zmp_point;

private:
	Eigen::MatrixXd force_data_l, torque_data_l;
	Eigen::MatrixXd force_data_r, torque_data_r;
	heroehs_math::Kinematics transformation_;
	double gf_fx_l,gf_fy_l,gf_fz_l,gf_tx_l,gf_ty_l,gf_tz_l; // ground frame gf
	double gf_fx_r,gf_fy_r,gf_fz_r,gf_tx_r,gf_ty_r,gf_tz_r; // ground frame gf
	double gf_px_l,gf_py_l,gf_pz_l;
	double gf_px_r,gf_py_r,gf_pz_r;

};


}



#endif /* SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_ZMP_CALCULATION_FUNCTION_H_ */
