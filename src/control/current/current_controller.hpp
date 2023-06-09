#pragma once

#include "math/math_core.hpp"

namespace zspinlab::controller {
// Classical current (torque) controller implementation
class CurrentController {
public:
    CurrentController() {};

    void set_Id_pi_params(float kP, float kI, float min, float max);     
    void set_Iq_pi_params(float kP, float kI, float min, float max);

    // Set Iq reference current
    void set_Iq_ref(float Iq_ref) { this->Iq_ref = Iq_ref; }
    // Get Iq reference current
    float get_Iq_ref(void) { return Iq_ref; }

    // Set Id reference current
    void set_Id_ref(float Id_ref) { this->Id_ref = Id_ref; }
    // Get Id reference current
    float get_Id_ref(void) { return Id_ref; };

    void run(float Id, float Iq, float sin_theta, float cos_theta);
    
    // Obtain the calculated alpha voltage vector
    float get_va(void) { return v_a; }
    // Obtain the calculated beta voltage vector
    float get_vb(void) { return v_b; }

private:
    // PI units
    zspinlab::math::modules::PI PI_id, PI_iq;

    float Iq_ref, Id_ref;
    float v_a, v_b;
};

/**
 * @brief Run the current controller module
 * @param[in] Id Input Iq current
 * @param[in] Iq Input Iq current
 * @param[in] sin_theta Input Sine value of electrical angle
 * @param[in] cos_theta Input Cosine value of electrical angle
 * 
 * @return None
 **/
inline void CurrentController::run(float Id, float Iq, float sin_theta, float cos_theta)
{
    float v_q, v_d;
    float sin_theta, cos_theta;

    // Currently not implementing feed-forward variable
    v_q = PI_iq.run(Iq_ref, Iq, 0.0f);
    v_d = PI_id.run(Id_ref, Id, 0.0f);

    // Get alpha and beta voltage
    zspinlab::math::function::inverse_park_transform(v_d, v_q, sin_theta, cos_theta, v_a, v_b);
}

} // namespace zspinner::controller