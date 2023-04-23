#include "pid.hpp"

namespace zspinlab::math::modules {

/**
 * @brief Initialize the PID controller general parameters
 * @param[in] kP        Proportional gain
 * @param[in] kI        Integral gain
 * @param[in] kD        Derivative gain
 * @param[in] outMin    Minimum controller output
 * @param[in] outMax    Maximum controller output
 **/
PID::PID(float kP, float kI, float kD, float outMin, float outMax)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->outMin = outMin;
    this->outMax = outMax;
}

/**
 * @brief Initialize the low pass filter parameters for the PID controller
 * @param[in] a1 The denominator filter coefficient value for z^(-1)
 * @param[in] b0 The numerator filter coefficient value for z^0
 * @param[in] b1 The numerator filter coefficient value for z^(-1)
 * @param[in] x1 The input value at time sample n=-1
 * @param[in] y1 The output value at time sample n=-1
 * 
 * @return None
 **/
void PID::set_lpf_parameter(float a1, float b0, float b1, float x1, float y1)
{
    filter.set_denominator_coefficient(a1);
    filter.set_denominator_coefficient_b0(b0);
    filter.set_denominator_coefficient_b1(b1);
    filter.set_initial_condition(x1, y1);
}

} // namespace zspinlab::math::modules