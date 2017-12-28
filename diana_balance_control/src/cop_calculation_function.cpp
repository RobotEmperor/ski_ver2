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

	cf_fx_l = (transformation_.center_to_sensor_transform_left * force_data_l)(0,0);
	cf_fy_l = (transformation_.center_to_sensor_transform_left * force_data_l)(1,0);
	cf_fz_l = (transformation_.center_to_sensor_transform_left * force_data_l)(2,0);

	cf_tx_l = (transformation_.center_to_sensor_transform_left * torque_data_l)(0,0);
	cf_ty_l = (transformation_.center_to_sensor_transform_left * torque_data_l)(1,0);
	cf_tz_l = (transformation_.center_to_sensor_transform_left * torque_data_l)(2,0);

	cf_px_r = transformation_.center_to_sensor_transform_right(0,3);
	cf_py_r = transformation_.center_to_sensor_transform_right(1,3);
	cf_pz_r = transformation_.center_to_sensor_transform_right(2,3);

	cf_fx_r = (transformation_.center_to_sensor_transform_right * force_data_r)(0,0);
	cf_fy_r = (transformation_.center_to_sensor_transform_right * force_data_r)(1,0);
	cf_fz_r = (transformation_.center_to_sensor_transform_right * force_data_r)(2,0);

	cf_tx_r = (transformation_.center_to_sensor_transform_right * torque_data_r)(0,0);
	cf_ty_r = (transformation_.center_to_sensor_transform_right * torque_data_r)(1,0);
	cf_tz_r = (transformation_.center_to_sensor_transform_right * torque_data_r)(2,0);


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

	printf("cop_fz_x  :: %f \n", cop_fz_point_x);

}
CopCompensationFunc::CopCompensationFunc()
{
	reference_point_Fz_x = 0;
	reference_point_Fz_y = 0;

	reference_point_Fy_x = 0;
	reference_point_Fy_z = 0;

	reference_point_Fx_y = 0;
	reference_point_Fx_z = 0;

	margin_pflug_bogen_l_fz_x = 0;
	margin_pflug_bogen_r_fz_x = 0;
	margin_pflug_bogen_l_fz_y = 0;
	margin_pflug_bogen_r_fz_y = 0;

	margin_pflug_bogen_l_fy_x = 0;
	margin_pflug_bogen_r_fy_x = 0;
	margin_pflug_bogen_l_fy_z = 0;
	margin_pflug_bogen_r_fy_z = 0;

	margin_pflug_bogen_l_fx_y = 0;
	margin_pflug_bogen_r_fx_y = 0;
	margin_pflug_bogen_l_fx_z = 0;
	margin_pflug_bogen_r_fx_z = 0;

	margin_carving_turn_l_fz_x = 0;
	margin_carving_turn_r_fz_x = 0;
	margin_carving_turn_l_fz_y = 0;
	margin_carving_turn_r_fz_y = 0;

	margin_carving_turn_l_fy_x = 0;
	margin_carving_turn_r_fy_x = 0;
	margin_carving_turn_l_fy_z = 0;
	margin_carving_turn_r_fy_z = 0;

	margin_carving_turn_l_fx_z = 0;
	margin_carving_turn_r_fx_z = 0;
	margin_carving_turn_l_fx_y = 0;
	margin_carving_turn_r_fx_y = 0;

	pid_control_value_fz_x = 0;
	pid_control_value_fz_y = 0;
	pid_control_value_fy_x = 0;
	pid_control_value_fy_z = 0;
	pid_control_value_fx_y = 0;
	pid_control_value_fx_z = 0;

	control_value_Fz_x = 0;
	control_value_Fz_y = 0;

	control_value_Fy_x = 0;
	control_value_Fy_z = 0;

	control_value_Fx_y = 0;
	control_value_Fx_z = 0;

	pidControllerFz_x = new control_function::PID_function(0.008,0.1,-0.1,0,0,0);
	pidControllerFz_y = new control_function::PID_function(0.008,0.1,-0.1,0,0,0);
	pidControllerFx = new control_function::PID_function(0.008,0.1,-0.1,0,0,0);
	pidControllerFy = new control_function::PID_function(0.008,0.1,-0.1,0,0,0);
}
CopCompensationFunc::~CopCompensationFunc()
{
}
void CopCompensationFunc::parse_margin_data()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("ski_main_manager") + "/data/cop/control_value.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	//margin load //
	margin_pflug_bogen_l_fz_x = doc["pflug_bogen_l_fz"][0].as<double>();
	margin_pflug_bogen_l_fz_y = doc["pflug_bogen_l_fz"][1].as<double>();

	margin_pflug_bogen_r_fz_x = doc["pflug_bogen_r_fz"][0].as<double>();
	margin_pflug_bogen_r_fz_y = doc["pflug_bogen_r_fz"][1].as<double>();
}
void CopCompensationFunc::centerOfPressureReferencePoint(std::string turn_type, double cur_l_point_x, double cur_l_point_y, double cur_l_point_z, double cur_r_point_x, double cur_r_point_y, double cur_r_point_z, double current_control_value)
{
	parse_margin_data();
	if(!turn_type.compare("pflug_bogen"))
	{
		//reference_point_Fz_x = cur_l_point_x - cur_l_point_x*(-current_control_value+1);
		//reference_point_Fz_y = cur_l_point_y - cur_l_point_y*current_control_value*margin_pflug_bogen_l_fz_y;
		if(current_control_value > 0)//right
		{
			reference_point_Fz_x = cur_l_point_x;
			reference_point_Fz_y = (cur_l_point_y - cur_l_point_y*(fabs(-current_control_value+1)))*margin_pflug_bogen_l_fz_y;

			reference_point_Fy_x = cur_l_point_x - cur_l_point_x*margin_pflug_bogen_l_fy_x;
			reference_point_Fy_z = cur_l_point_z - cur_l_point_z*margin_pflug_bogen_l_fy_z;

			reference_point_Fx_y = cur_l_point_y - cur_l_point_y*margin_pflug_bogen_l_fx_y;
			reference_point_Fx_z = cur_l_point_z - cur_l_point_z*margin_pflug_bogen_l_fx_z;
		}
		else if(current_control_value < 0)//left
		{
			reference_point_Fz_x = cur_r_point_x;
			reference_point_Fz_y = (cur_r_point_y - cur_r_point_y*(fabs(-current_control_value-1)))*margin_pflug_bogen_r_fz_y;

			reference_point_Fy_x = cur_r_point_x - cur_r_point_x*margin_pflug_bogen_r_fy_x;
			reference_point_Fy_z = cur_r_point_z - cur_r_point_z*margin_pflug_bogen_r_fy_z;

			reference_point_Fx_y = cur_r_point_y - cur_r_point_y*margin_pflug_bogen_r_fx_y;
			reference_point_Fx_z = cur_r_point_z - cur_r_point_z*margin_pflug_bogen_r_fx_z;
		}
		else // zero position
		{
			reference_point_Fz_x = (cur_r_point_x + cur_l_point_x)/2;
			reference_point_Fz_y = (cur_r_point_y + cur_l_point_y)/2;

			reference_point_Fy_x = (cur_r_point_x + cur_l_point_x)/2;
			reference_point_Fy_z = (cur_r_point_z + cur_l_point_z)/2;

			reference_point_Fx_y = (cur_r_point_y + cur_l_point_y)/2;
			reference_point_Fx_z = (cur_r_point_z + cur_l_point_z)/2;
		}
		// retain center position (reference point is center)
	}
	if(!turn_type.compare("carving_turn"))
	{
		return; // Don't write until carving turn algorithm is completed
	}
	else
		return;

}
void CopCompensationFunc::centerOfPressureCompensationFz(double current_point_x, double current_point_y)
{
	pid_control_value_fz_x = pidControllerFz_x->PID_calculate(reference_point_Fz_x, current_point_x);
	pid_control_value_fz_y = pidControllerFz_y->PID_calculate(reference_point_Fz_y, current_point_y);

	control_value_Fz_x = pid_control_value_fz_x;
	control_value_Fz_y = pid_control_value_fz_y;
	//control_value_Fz_x = pid_control_value_fz_x + control_value_Fz_x;
	//control_value_Fz_y = pid_control_value_fz_y + control_value_Fz_y; // 최종적으로 들어갈때 - 로 들어가야함 상대적 운동임
}
void CopCompensationFunc::centerOfPressureCompensationFy(double current_point_x, double current_point_z)
{
	pid_control_value_fy_x = pidControllerFz_x->PID_calculate(reference_point_Fy_x, current_point_x);
	pid_control_value_fy_z = pidControllerFz_y->PID_calculate(reference_point_Fy_z, current_point_z);

	control_value_Fy_x = pid_control_value_fy_x + control_value_Fy_x;
	control_value_Fy_z = pid_control_value_fy_z + control_value_Fy_z;
}
void CopCompensationFunc::centerOfPressureCompensationFx(double current_point_y, double current_point_z)
{
	pid_control_value_fx_y = pidControllerFz_x->PID_calculate(reference_point_Fx_y, current_point_y);
	pid_control_value_fx_z = pidControllerFz_y->PID_calculate(reference_point_Fx_z, current_point_z);

	control_value_Fx_y = pid_control_value_fx_y + control_value_Fx_y;
	control_value_Fx_z = pid_control_value_fx_z + control_value_Fx_z;
}
