#pragma once

#include "svpwm_base.hpp"
#include <zephyr/sys/util.h>

namespace zspinlab::modulation {

// This implement the classical Alternating Reverse Sequencing SVPWM algorithm - the de-facto industry stardard in FOC motor control  
class SVPWM_ARS : public SVPWM_Base<SVPWM_ARS>
{
public:
    
    // Constructor
    using SVPWM_Base::SVPWM_Base;

    void init(void) {}
    void run(void);
};

/**
 * @brief Run the classical Alternating Reverse Sequencing SVPWM algorithm, originally written for ODrive ESCs
 * 
 * @return None
 */
inline void SVPWM_ARS::run(void)
{
	float t1, t2, t3, t4, t5, t6;
    
    uint8_t sector;

    // Limit alpha and beta if required
    limit_vref_ab();

	// Get sector
	if (vb >= 0.0f) {
        sector = (va >= 0.0f)  
                    ? (MATH_1_BY_SQRT_3 * vb > va) ? 2U : 1U
                    : (-MATH_1_BY_SQRT_3 * vb > va) ? 3U : 2U;
	} else {
        sector = (va >= 0.0f)
                    ? (-MATH_1_BY_SQRT_3 * vb > va) ? 5U : 6U
                    : (MATH_1_BY_SQRT_3 * vb > va) ? 4U : 5U;
	}

	switch (sector) {
        case 1:
            t1 = va - MATH_1_BY_SQRT_3 * vb;
            t2 = MATH_2_BY_SQRT_3 * vb;

            dA = (1.0f - t1 - t2) * 0.5f;
            dB = dA + t1;
            dC = dB + t2;

			break;

        case 2: 
            t2 = va + MATH_1_BY_SQRT_3 * vb;
            t3 = -va + MATH_1_BY_SQRT_3 * vb;

            dB = (1.0f - t2 - t3) * 0.5f;
            dA = dB + t3;
            dC = dA + t2;

			break;

        case 3:
            t3 = MATH_2_BY_SQRT_3 * vb;
            t4 = -va - MATH_1_BY_SQRT_3 * vb;

            dB = (1.0f - t3 - t4) * 0.5f;
            dC = dB + t3;
            dA = dC + t4;

			break;

        case 4:
            t4 = -va + MATH_1_BY_SQRT_3 * vb;
            t5 = -MATH_2_BY_SQRT_3 * vb;

            dC = (1.0f - t4 - t5) * 0.5f;
            dB = dC + t5;
            dA = dB + t4;

			break;

        case 5: 
            t5 = -va - MATH_1_BY_SQRT_3 * vb;
            t6 = va - MATH_1_BY_SQRT_3 * vb;

            dC = (1.0f - t5 - t6) * 0.5f;
            dA = dC + t5;
            dB = dA + t6;

			break;

        case 6:
            t6 = -MATH_2_BY_SQRT_3 * vb;
            t1 = va + MATH_1_BY_SQRT_3 * vb;

            dA = (1.0f - t6 - t1) * 0.5f;
            dC = dA + t1;
            dB = dC + t6;

			break;
	}
	
    // Clamp
	dA = CLAMP(dA, 0.0f, 1.0f);
	dB = CLAMP(dB, 0.0f, 1.0f);
	dC = CLAMP(dC, 0.0f, 1.0f);
}

} // namspace zspinlab::modulation::SpaceVectorPWM