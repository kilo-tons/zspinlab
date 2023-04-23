#pragma once

#include <zephyr/sys/util.h>
#include "math/math_core.hpp"

namespace zspinlab::math::modules {

// Create a generic PID controller
class PID {
public:
    PID(float kP = 0.0f, float kI = 0.0f, float kD = 0.0f, float outMin = 0.0f, float outMax = 0.0f);

    void set_lpf_parameter(float a1, float b0, float b1, float x1, float y1);

    float run(float sp, float pv, float ffwd);
    float reset_state(void);

    float get_kp(void) { return kP; }
    float get_ki(void) { return kI; }
    float get_kd(void) { return kD; }

    void set_kp(float kP) { this->kP = kP; }
    void set_ki(float kI) { this->kI = kI; }
    void set_kd(float kD) { this->kD = kD; }

    float get_outMin(void) { return outMin; }
    float get_outMax(void) { return outMax; }

    void set_outMin(float outMin) { this->outMin = outMin; }
    void set_outMax(float outMax) { this->outMax = outMax; }

private:
    // Filter unit
    LowPassFirstOrder filter{0.0f, 1.0f, 0.0f}; // Default to non-filtering mode
    
    float kP, kI, kD;
    float outMin, outMax;

    float prev_i_term;      // Previous integrator term
    
    float prev_sp;          // Previous desired setpoint
    float prev_pv;          // Previous measured process variable
    float prev_ffwd;        // Previous feed-forward variable     
};

/**
 * @brief Run the PID controller
 * @param[in] sp    Desired setpoint
 * @param[in] pv    Measured process variable
 * @param[in] ffwd  Feed-forward variable
 * 
 * @return Processed output sample
 **/
inline float PID::run(float sp, float pv, float ffwd)
{
    float error;
    float p_term, i_term, d_term;

    error   = sp - pv;

    p_term  = kP*error;
    i_term  = (kI == 0.0f ? 0.0f : CLAMP(prev_i_term + kI * error, outMin, outMax));     // Only bother when kI is used
    d_term  = (kI == 0.0f ? 0.0f : filter.run(kD * error));     // Only bother when kD is used

    // Store previous state
    prev_i_term = i_term;
    prev_sp = sp;
    prev_pv = pv;
    prev_ffwd = ffwd;

    return CLAMP(p_term + i_term + d_term + ffwd, outMin, outMax);
}

/**
 * @brief Reset the PID controller to default state (zero)
 *
 * @return None
 **/
inline float PID::reset_state(void)
{
    prev_i_term = 0;
    prev_sp = 0;
    prev_pv = 0;
    prev_ffwd = 0;
}

} // zspinlab::math::modules