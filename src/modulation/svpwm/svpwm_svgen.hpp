#pragma once

#include "svpwm_base.hpp"
#include <zephyr/sys/util.h>

namespace zspinlab::modulation {

// This implement the optimized SVPWM algorithm generated from MATLAB Space Vector Generator Simulink Toolbox  
class SVPWM_SVGen : public SVPWM_Base<SVPWM_SVGen>
{
public:
    
    // Constructor
    SVPWM_SVGen(void) = default;

    void init(void) {}
    void run(void);
};

/**
 * @brief Run the SVPWM algorithm generated from the MATLAB Simulink SVGEN block.
 * 
 * @return None
 */
inline void SVPWM_SVGen::run(void)
{
	float a, b, c;

    c = 0.5f * va;
    b = MATH_SQRT_3_BY_2 * vb;
    a = b - c;
    c = -c - b;
    b = (MAX(MAX(va, a), c) + MIN(MIN(va, a), c)) * -0.5f;

    dA = (va + b) * MATH_2_BY_SQRT_3;
    dB = (a + b) * MATH_2_BY_SQRT_3;
    dC = (b + c) * MATH_2_BY_SQRT_3;

    // Clamp
	dA = CLAMP(dA, 0.0f, 1.0f);
	dB = CLAMP(dB, 0.0f, 1.0f);
	dC = CLAMP(dC, 0.0f, 1.0f);
}

} // namespace zspinlab::modulation::SpaceVectorPWM