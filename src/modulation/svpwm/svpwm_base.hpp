#pragma once

#include <cstdint>
#include "../../math_const.hpp"
#include "../../math_core.hpp"

namespace zspinlab::modulation {

template <class Derived>
class SVPWM_Base
{
public:
    // Constructor, do not allow over-modulation by default
    SVPWM_Base(void) { overmodulate = false; }

    // De-constructor
    virtual ~SVPWM_Base(void) {}

    // Set the <alpha, beta> vectors
    void set_vref_ab(float v_a, float v_b);

    // Obtain the calculated phase duty cycle for A channel   
    float get_phase_duty_a(void) { return dA; }

    // Obtain the calculated phase duty cycle for B channel   
    float get_phase_duty_b(void) { return dB; }

    // Obtain the calculated phase duty cycle for C channel   
    float get_phase_duty_c(void) { return dC; }

    // Allow/disallow over-modulation mode
    void allow_overmodulation(bool overmodulate) { this->overmodulate = overmodulate; }

    // Custom virtual methods, should be implemented in child classes
    void init(void) { static_cast<Derived*>this->init(); }      // Initialize any remaining required parameters   
    void run(void) { static_cast<Derived*>this->run(); }        // Main method, run the algorithm

protected:
    // Phase duty cycle 
    float dA, dB, dC;

    // Inverse Clark reference voltage vectors (alpha and beta), should be normalized to [-1, 1]
    float va, vb;

    // Flag to indicate if we want phase over-modulation to increase the output voltage to motor
    bool overmodulate; 

    // Limit alpha-beta maximum amplitude to avoid distortions when phase over-modulation is not supported
    void limit_vref_ab(void);
};

/**
 * @brief Set alpha and beta voltage vector before running SVPWM
 * @param[in] v_a alpha voltage component
 * @param[in] v_b beta voltage component
 * @return None
 */
template <class Derived>
inline void SVPWM_Base<Derived>::set_vref_ab(float v_a, float v_b)
{
	va = v_a;
	vb = v_b;
}

/**
 * @brief Limit alpha and beta voltage maximum amplitude if over-modulation is not supported
 * 
 * @return None
 */
template <class Derived>
inline void SVPWM_Base<Derived>::limit_vref_ab(void)
{
    if (overmodulate) {
        return;
    }

    float mod;
    zspinlab::math::basic::fsqrtf(va*va + vb*vb, mod);

    if (mod > MATH_SQRT_3_BY_2) {
        va   = va / mod * MATH_SQRT_3_BY_2;
        vb   = vb / mod * MATH_SQRT_3_BY_2;
    }
}

} // namespace zspinlab::modulation::SpaceVectorPWM
