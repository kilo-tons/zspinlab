#pragma once

#include <zephyr/sys/util.h>
#include "math/math_core.hpp"

namespace zspinlab::controller {
// Classical current (torque) controller implementation
class CurrentController {
public:
    CurrentController() {};

    void set_Id_pid_params(float kP, float kI, float kD, float min, float max);     
    void set_Id_lpfilter_coeffs(float a1, float b0, float b1, float x1, float y1);  // Optional

    void set_Iq_pid_params(float kP, float kI, float kD, float min, float max);
    void set_Iq_lpfilter_coeffs(float a1, float b0, float b1, float x1, float y1);  // Optional

    // Set Iq reference current
    void set_Iq_ref(float Iq_ref) { this->Iq_ref = Iq_ref; }
    // Get Iq reference current
    float get_Iq_ref(void) { return Iq_ref; }

    // Set Id reference current
    void set_Id_ref(float Id_ref) { this->Id_ref = Id_ref; }
    // Get Id reference current
    float get_Id_ref(void) { return Id_ref; };

    void run(float Id, float Iq, float theta);
    
    // Obtain the calculated alpha voltage vector
    float get_va(void) { return v_a; }
    // Obtain the calculated beta voltage vector
    float get_vb(void) { return v_b; }

private:
    // PID units
    zspinlab::math::modules::PID PID_id, PID_iq;

    float Iq_ref, Id_ref;
    float v_a, v_b;
};

/**
 * @brief Run the current controller module
 * @param[in] Id Input Iq current
 * @param[in] Iq Input Iq current
 * @param[in] theta Input electrical angle in degree
 * 
 * @return None
 **/
inline void CurrentController::run(float Id, float Iq, float theta)
{
    float v_q, v_d;
    float sin_theta, cos_theta;

    // Currently not implementing feed-forward variable
    v_q = PID_iq.run(Iq_ref, Iq, 0.0f);
    v_d = PID_id.run(Id_ref, Id, 0.0f);

    // Get theta sin-cos value
    zspinlab::math::basic::fsincosf(theta, sin_theta, cos_theta);

    // Get alpha and beta voltage
    zspinlab::math::function::inverse_park_transform(v_d, v_q, sin_theta, cos_theta, v_a, v_b);
}

} // namespace zspinner::controller