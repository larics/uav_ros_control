/*
 * NonlinearFilters.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <iostream>
#include <math.h>

double nonlinear_filters::saturation(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value > highLimit) { return highLimit; }
	else if (value < lowLimit) { return lowLimit; }
	else { return value; }
}

double nonlinear_filters::deadzone(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value < highLimit && value > lowLimit)  
		return 0; 
	else if (value >= highLimit) 
		return value - highLimit;
	else if (value <= lowLimit)
		return value - lowLimit;
}

double nonlinear_filters::filterPT1(
		double previousValue,
		double currentValue,
		double T,
		double Ts,
		double K)
{
	double a = T / (T + Ts);
	double b = K * Ts / (T + Ts);

	return (a * previousValue + b * currentValue);
}


double wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}

double util::wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}

double util::calculateYaw(double qx, double qy, double qz, double qw)
{
	return atan2(
		2 * (qw * qz + qx * qy),
		qw * qw + qx * qx - qy * qy - qz * qz);
}


