/*
 * control_function.h
 *
 *  Created on: Dec 21, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_CONTROL_FUNCTION_H_
#define SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_CONTROL_FUNCTION_H_

namespace control_function
{

class PID_function
{
public:
	PID_function(double dt, double max, double min, double kp, double kd, double ki);
	~PID_function();
	double PID_calculate(double ref_value, double current_value);
	double kp_;
	double kd_;
	double ki_;

private:
	double dt_;
	double max_;
	double min_;
	double pre_error_;
	double integral_;

};
}




#endif /* SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_CONTROL_FUNCTION_H_ */
