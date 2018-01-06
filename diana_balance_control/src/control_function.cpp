/*
 * control_function.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: robotemperor
 */
#include "diana_balance_control/control_function.h"

using namespace control_function;

/////// PID function /////////////////////////////////////////////////////////////////////////
PID_function::PID_function(double dt, double max, double min, double kp, double kd, double ki) :
	dt_(dt),
	max_(max),
	min_(min),
	kp_(kp),
	ki_(ki),
	kd_(kd),
	pre_error_(0),
	integral_(0)
{
}
double PID_function::PID_calculate(double ref_value, double current_value)
{
		// calculate error
			double error = ref_value - current_value;

			// P gain
			double p_control_value = kp_ * error;

			// I gain
			integral_ += error * dt_;
			double i_control_value = ki_ * integral_;

			//D gain
			double derivate = (error - pre_error_)/dt_;
			double d_control_value = kd_ * derivate;

			// calculate control value

			double output = p_control_value + i_control_value + d_control_value;

			if(output > max_)
				output = max_;
			else if (output < min_)
				output = min_;

			pre_error_ = error;

			if(output < 0.0005 && output > -0.0005)
			output = 0;

			return output;
}

PID_function::~PID_function()
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////
Filter::Filter()
{

}
Filter::~Filter()
{

}
double Filter::lowPassFilter(double value, double pre_value, double weight_factor, double sampling_time)
{
	printf("cutoff_frequency  ::  %f \n", 1/(2*M_PI*weight_factor));
	return (weight_factor*pre_value + sampling_time*value)/(weight_factor + sampling_time);
}
