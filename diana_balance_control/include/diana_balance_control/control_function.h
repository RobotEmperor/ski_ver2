/*
 * control_function.h
 *
 *  Created on: Dec 26, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_CONTROL_FUNCTION_H_
#define SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_CONTROL_FUNCTION_H_

#include <stdio.h>

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
	double max_;
	double min_;

private:
	double dt_;
	double pre_error_;
	double integral_;
};

class Filter
{
public:
	Filter();
	~Filter();
	double lowPassFilter(double value, double pre_value, double weight_factor);
};
}

#endif /* SKI_VER2_DIANA_BALANCE_CONTROL_INCLUDE_DIANA_BALANCE_CONTROL_CONTROL_FUNCTION_H_ */
