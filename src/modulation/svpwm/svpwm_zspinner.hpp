#pragma once

#include "svpwm_base.hpp"
#include <zephyr/sys/util.h>


namespace zspinlab::modulation {

// This implement the SVPWM algorithm from the Zephyr Spinner project, work in progress  
class SVPWM_ZSpinner :  public SVPWM_Base<SVPWM_ZSpinner>
{
public:
    
    // Constructor
    using SVPWM_Base::SVPWM_Base;

    void init(void) {}
    void run(void);

private:
    uint8_t get_sector(float a, float b, float c);
};


/**
 * @brief Run the SVM algorithm implemented in the Zephyr Spinner project
 * 
 * @return None
 */
inline void SVPWM_ZSpinner::run(void)
{
	float a, b, c;
	float x, y, z;

	uint8_t sector;

    // Limit alpha and beta if required
    limit_vref_ab();

	a = va - MATH_1_BY_SQRT_3 * vb;
	b = MATH_2_BY_SQRT_3 * vb;
	c = -(va + vb);

    // Find sector
    sector = get_sector(a, b, c);

	switch (sector) {
	case 1U:
		x = a;
		y = b;
		z = 1.0f - (x + y);

		dA = x + y + z * 0.5f;
		dB = y + z * 0.5f;
		dC = z * 0.5f;

		break;
	case 2U:
		x = -c;
		y = -a;
		z = 1.0f - (x + y);

		dA = x + z * 0.5f;
		dB = x + y + z * 0.5f;
		dC = z * 0.5f;

		break;
	case 3U:
		x = b;
		y = c;
		z = 1.0f - (x + y);

		dA = z * 0.5f;
		dB = x + y + z * 0.5f;
		dC = y + z * 0.5f;

		break;
	case 4U:
		x = -a;
		y = -b;
		z = 1.0f - (x + y);

		dA = z * 0.5f;
		dB = x + z * 0.5f;
		dC = x + y + z * 0.5f;

		break;
	case 5U:
		x = c;
		y = a;
		z = 1.0f - (x + y);

		dA = y + z * 0.5f;
		dB = z * 0.5f;
		dC = x + y + z * 0.5f;

		break;
	case 6U:
		x = -b;
		y = -c;
		z = 1.0f - (x + y);

		dA = x + y + z * 0.5f;
		dB = z * 0.5f;
		dC = x + z * 0.5f;

		break;
	default:
        // Invalid state
        dA = 0;
        dB = 0;
        dC = 0;

		break;
	}

	// Clamp
	dA = CLAMP(dA, 0.0f, 1.0f);
	dB = CLAMP(dB, 0.0f, 1.0f);
	dC = CLAMP(dC, 0.0f, 1.0f);
}


/**
 * @brief Obtain sector based on a, b, c vector values.
 *
 * @param[in] a a component value.
 * @param[in] b b component value.
 * @param[in] c c component value.
 * @return Sector (1...6).
 */
inline uint8_t SVPWM_ZSpinner::get_sector(float a, float b, float c)
{
	if (c < 0.0f) {
		if (a < 0.0f) {
			return 2U;
		} else {
            return (b < 0.0f ? 6U : 1U);
		}
	} else {
		if (a < 0.0f) {
            return (b <= 0.0f ? 4U : 3U);
		} else {
			return 5U;
		}
	}
}

}// namespace zspinlab::modulation::SpaceVectorPWM