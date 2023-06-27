#pragma once

#include <zephyr/sys/util.h>
#include "math/math_core.hpp"

namespace zspinlab::math::modules
{
    // Create a generic PI controller
    class PI
    {
    public:
        /**
         * @brief Initialize the PID controller general parameters
         * @param[in] kP        Proportional gain
         * @param[in] kI        Integral gain
         * @param[in] outMin    Minimum controller output
         * @param[in] outMax    Maximum controller output
         **/
        PI(float kP = 0.0f, float kI = 0.0f, float outMin = 0.0f, float outMax = 0.0f);

        float run(float sp, float pv, float ffwd);
        float reset_state(void);

        float get_kp(void) { return kP; }
        float get_ki(void) { return kI; }

        void set_kp(float kP) { this->kP = kP; }
        void set_ki(float kI) { this->kI = kI; }

        float get_outMin(void) { return outMin; }
        float get_outMax(void) { return outMax; }

        void set_outMin(float outMin) { this->outMin = outMin; }
        void set_outMax(float outMax) { this->outMax = outMax; }

    private:
        float kP, kI;
        float outMin, outMax;

        float prev_i_term; // Previous integrator term

        float prev_sp;   // Previous desired setpoint
        float prev_pv;   // Previous measured process variable
        float prev_ffwd; // Previous feed-forward variable
    };

    /**
     * @brief Run the PI controller
     * @param[in] sp    Desired setpoint
     * @param[in] pv    Measured process variable
     * @param[in] ffwd  Feed-forward variable
     *
     * @return Processed output sample
     **/
    inline float PI::run(float sp, float pv, float ffwd)
    {
        float error;
        float p_term, i_term, d_term;

        error = sp - pv;

        p_term = kP * error;
        i_term = CLAMP(prev_i_term + kI * error, outMin, outMax);

        // Store previous state
        prev_i_term = i_term;
        prev_sp = sp;
        prev_pv = pv;
        prev_ffwd = ffwd;

        return CLAMP(p_term + i_term + ffwd, outMin, outMax);
    }

    /**
     * @brief Reset the PI controller to default state (zero)
     *
     * @return None
     **/
    inline float PI::reset_state(void)
    {
        prev_i_term = 0;
        prev_sp = 0;
        prev_pv = 0;
        prev_ffwd = 0;
    }

} // zspinlab::math::modules