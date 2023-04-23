#include "current_controller.hpp"

namespace zspinlab::controller{

/**
 * @brief Set the Id PID controller general parameters
 * @param[in] kP        Proportional gain
 * @param[in] kI        Integral gain
 * @param[in] kD        Derivative gain
 * @param[in] min       Minimum controller output
 * @param[in] max       Maximum controller output
 **/
void CurrentController::set_Id_pid_params(float kP, float kI, float kD, float min, float max)
{
    PID_id.set_kp(kP);
    PID_id.set_ki(kI);
    PID_id.set_kd(kD);
    PID_id.set_outMax(max);
    PID_id.set_outMin(min);
}

/**
 * @brief Config the low pass filter parameters for the Id PID controller, optional. LPF is ignored when using only PI
 * @param[in] a1 The denominator filter coefficient value for z^(-1)
 * @param[in] b0 The numerator filter coefficient value for z^0
 * @param[in] b1 The numerator filter coefficient value for z^(-1)
 * @param[in] x1 The input value at time sample n=-1
 * @param[in] y1 The output value at time sample n=-1
 * 
 * @return None
 **/
void CurrentController::set_Id_lpfilter_coeffs(float a1, float b0, float b1, float x1, float y1)
{
    PID_id.set_lpf_parameter(a1, b0, b1, x1, y1);
}

/**
 * @brief Set the Iq PID controller general parameters
 * @param[in] kP        Proportional gain
 * @param[in] kI        Integral gain
 * @param[in] kD        Derivative gain
 * @param[in] min       Minimum controller output
 * @param[in] max       Maximum controller output
 **/
void CurrentController::set_Iq_pid_params(float kP, float kI, float kD, float min, float max)
{
    PID_iq.set_kp(kP);
    PID_iq.set_ki(kI);
    PID_iq.set_kd(kD);
    PID_iq.set_outMax(max);
    PID_iq.set_outMin(min);
}

/**
 * @brief Config the low pass filter parameters for the Iq PID controller, optional. LPF is ignored when using only PI
 * @param[in] a1 The denominator filter coefficient value for z^(-1)
 * @param[in] b0 The numerator filter coefficient value for z^0
 * @param[in] b1 The numerator filter coefficient value for z^(-1)
 * @param[in] x1 The input value at time sample n=-1
 * @param[in] y1 The output value at time sample n=-1
 * 
 * @return None
 **/
void CurrentController::set_Iq_lpfilter_coeffs(float a1, float b0, float b1, float x1, float y1)
{
    PID_iq.set_lpf_parameter(a1, b0, b1, x1, y1);
}

} // namespace zspinner::controller


