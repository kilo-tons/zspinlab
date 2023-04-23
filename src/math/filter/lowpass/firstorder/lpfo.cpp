#include "lpfo.hpp"

namespace zspinlab::math::modules {

/** 
 * @brief Constructor, set initial denominator and numerator coefficients
 * @param[in] a1 The denominator filter coefficient value for z^(-1)
 * @param[in] b0 The numerator filter coefficient value for z^0
 * @param[in] b1 The numerator filter coefficient value for z^(-1)
 **/
LowPassFirstOrder::LowPassFirstOrder(float a1, float b0, float b1) 
{
    this->a1 = a1;
    this->b0 = b0;
    this->b1 = b1;

    // Default the initial condition to zero
    this->x1 = 0.0f;
    this->y1 = 0.0f;
}

/**
 * @brief Set the initial input and output of the filter
 * @param[in] x1 The input value at time sample n=-1
 * @param[in] y1 The output value at time sample n=-1
 * 
 * @return None
 **/
void LowPassFirstOrder::set_initial_condition(float x1, float y1)
{
    this->x1 = x1;
    this->y1 = y1;
}

} // namespace zspinlab::math::modules