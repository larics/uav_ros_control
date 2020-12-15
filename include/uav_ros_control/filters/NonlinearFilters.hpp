/*
 * NonlinearFilters.hpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#ifndef NONLINEAR_FILTERS_H
#define NONLINEAR_FILTERS_H

namespace nonlinear_filters {
/**
 * Perform saturation filter on the given value;
 *
 * @param value
 * @param lowLimit
 * @param highLimit
 *
 * @return saturated value
 */
double saturation(double value, double lowLimit, double highLimit);

/**
 * Perform deadzone filter on given value.
 *
 * @param value
 * @param lowLimit
 * @param highLimit
 */
double deadzone(double value, double lowLimit, double highLimit);

/**
 * Concrete PT1 filter implementation.
 *
 * @param previousValue
 * @param currentValue
 * @param T - time constant
 * @param Ts - discretization step
 * @param K - Filter gain
 */
double
  filterPT1(double previousValue, double currentValue, double T, double Ts, double K);
}// namespace nonlinear_filters

namespace util {
/**
 *  wrap x -> [min,max)
 */
double wrapMinMax(double x, double min, double max);

/**
 * Calculates yaw angle from given quaternion components.
 */
double calculateYaw(double qx, double qy, double qz, double qw);
}// namespace util

#endif /* NONLINEAR_FILTERS_H */
