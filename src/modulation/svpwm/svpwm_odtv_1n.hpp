#pragma once

#include "svpwm_base.hpp"
#include <zephyr/sys/util.h>


namespace zspinlab::modulation {
/*
 * This implement the optimized SVPWM algorithm called ODTV 1-norm from 
 * "A Simplified Space Vector Pulse Width Modulation Algorithm of a High-Speed Permanent Magnet Synchronous Machine Drive for a Flywheel Energy Storage System"
 * 
 * Reference: https://www.mdpi.com/1996-1073/15/11/4065
 */  
class SVPWM_ODTV_1N : public SVPWM_Base<SVPWM_ODTV_1N>
{
public:
    
    // Constructor
    using SVPWM_Base::SVPWM_Base;

    void init(void) {};
    void run(void);
};


/**
 * @brief Run the SVPWM ODTV 1-norm algorithm.
 * 
 * @note Modifed the dA calculation formula when (|b| >= |c| && |b| > |a|) (either this is a small mistake from the paper or
 * we are using a different sector order)
 * 
 * @return None
 */
inline void SVPWM_ODTV_1N::run(void)
{
	float a, b, c;
	float abs_a, abs_b, abs_c;
	float half_a, half_b, half_c;

    // Limit alpha and beta if required
    limit_vref_ab();

	a = va + MATH_1_BY_SQRT_3 * vb;
	b = MATH_2_BY_SQRT_3 * vb;
	c = a - b;

	// Pre-compute everything possible
	abs_a = zspinlab::math::basic::ffabsf(a);
	abs_b = zspinlab::math::basic::ffabsf(b);
	abs_c = zspinlab::math::basic::ffabsf(c);

	half_a = a * 0.5f;
	half_b = b * 0.5f;
	half_c = c * 0.5f;

	if ((abs_c >= abs_a) && (abs_c > abs_b)) {
		dA = half_c + 0.5f;
		dB = -half_c + 0.5f;
		dC = -half_a - half_b + 0.5f;

	} else if ((abs_a >= abs_b) && (abs_a > abs_c)) {
		dA = half_a + 0.5f;
		dB = half_b - half_c + 0.5f;
		dC = -half_a + 0.5f;

	} else {
		dA = half_a + half_c + 0.5f;
		dB = half_b + 0.5f;
		dC = -half_b + 0.5f;

	}

		// Clamp
	dA = CLAMP(dA, 0.0f, 1.0f);
	dB = CLAMP(dB, 0.0f, 1.0f);
	dC = CLAMP(dC, 0.0f, 1.0f);
}

} // namespace zspinlab::modulation::SpaceVectorPWM