#pragma once

#include <cstdint>
#include <math.h>

#include "math_const.hpp"
#include "pid/pid.hpp"
#include "filter/lowpass/firstorder/lpfo.hpp"

#if defined(CONFIG_CMSIS_DSP) && defined(CONFIG_ARM)
#include <arm_math.h>
#endif

namespace zspinlab::math::type {

} // namespace zspinlab::math::type

// Namespaces for basic math operations that could be performed by specialized instructions or hardwares
namespace zspinlab::math::basic {

inline void fcosf(float rad, float &out) 
{
#if defined(CONFIG_CMSIS_DSP) && defined(CONFIG_ARM)
    // Currently only support ARM with DSP functions
    out = arm_cos_f32(rad);
#else
    // Generic newlib implementation
    out = cosf(rad);
#endif
}

inline void fsinf(const float rad, float &out) 
{
#if defined(CONFIG_CMSIS_DSP) && defined(CONFIG_ARM)
    // Currently only support ARM with DSP functions
    out = arm_sin_f32(rad);
#else
    // Generic newlib implementation
    out = sinf(rad);
#endif
}

inline void fsqrtf(const float square, float &out) 
{
#if defined(CONFIG_CMSIS_DSP) && defined(CONFIG_ARM)
    // Currently only support ARM with DSP functions
    (void)arm_sqrt_f32(square, out);
#else
    // Generic newlib implementation
    out = sqrtf(square);
#endif
}

inline void fsincosf(const float angle_deg, float &sin_out, float &cos_out) {
#if defined(CONFIG_CMSIS_DSP) && defined(CONFIG_ARM)
    // Currently only support ARM with DSP functions
    arm_sin_cos_f32(angle_deg, sin_out, cos_out);
#else
    // Generic newlib implementation
    const angle_rad = angle_deg * 180.0f * M_1_PI;
    sin_out = sinf(angle_rad);
    cos_out = cosf(angle_rad);
#endif
}

} // namespace zspinlab::math::basic


// Namespace for simple, "direct" motor control algorithm functions
namespace zspinlab::math::function {

/**
 * @brief Vector Clarke transform.
 * @param[in] use_all_phase Using all the input phase coordinates. This is statically defined 
 * @param[in] iA            Input three phase coordinate A
 * @param[in] iB            Input three phase coordinate B
 * @param[in] iC            (Optional) Input three phase coordinate C, ignored if \p use_all_phase is set to false
 * @param[out] i_alpha      Output two-phase vector coordinate alpha
 * @param[out] i_beta       Output two-phase vector coordinate beta
 *  
 * @return None      
 **/
template <bool use_all_phase>
inline void clarke_transform(float iA,
                            float iB,
                            float iC,
                            float &i_alpha,
                            float &i_beta)
{
    // Statically check if we are using all the three phases
    if constexpr(use_all_phase) {
        (void)iC;
        i_alpha = MATH_2_BY_3*iA - MATH_1_BY_3*(iB+iC);
        i_beta  = MATH_1_BY_SQRT_3 * (iB-iC);
    } else {
        i_alpha = iA;
        i_beta  = MATH_1_BY_SQRT_3*iA + MATH_2_BY_SQRT_3*iB; 
    }
}

/**
 * @brief Vector Inverse Clarke transform
 * 
 * @param[in] i_alpha  Output two-phase vector coordinate alpha
 * @param[in] i_beta   Output two-phase vector coordinate beta
 * @param[out] iA      Output three phase coordinate A
 * @param[out] iB      Output three phase coordinate B
 * @param[out] iC      Output three phase coordinate C
 *  
 * @return None      
 **/
inline void inverse_clarke_transform(float i_alpha,
                                    float i_beta,
                                    float &iA,
                                    float &iB,
                                    float &iC)
{
    iA = i_alpha;
    iB = -0.5*i_alpha + MATH_SQRT_3_BY_2*i_beta;
    iC = -0.5*i_alpha - MATH_SQRT_3_BY_2*i_beta;    // Optional
}


/**
 * @brief Vector Park transform
 * @param[in] i_alpha   Input two-phase vector coordinate alpha
 * @param[in] i_beta    Input two-phase vector coordinate beta
 * @param[in] sin_theta Sine value of rotation angle theta
 * @param[in] cos_theta Cosine value of rotation angle theta
 * @param[out] id       Output coordinate rotor reference frame d
 * @param[out] iq       Output coordinate rotor reference frame q
 * 
 * @return None      
 **/
inline void park_transform(float i_alpha,
                        float i_beta,
                        float sin_theta,
                        float cos_theta,
                        float &id,
                        float &iq)
{
    id = i_alpha * cos_theta + i_beta * sin_theta;
    iq = -i_alpha * sin_theta + i_beta * cos_theta;
}

/**
 * @brief Vector Inverse Park transform
 * @param[in] id        Input coordinate of rotor reference frame d
 * @param[in] iq        Input coordinate rotor reference frame q
 * @param[in] sin_theta Sine value of rotation angle theta
 * @param[in] cos_theta Cosine value of rotation angle theta
 * @param[out] i_alpha  Output two-phase vector coordinate alpha
 * @param[out] i_beta   Output two-phase vector coordinate beta
 *  
 * @return None      
 **/
inline void inverse_park_transform(float id,
                                float iq,
                                float sin_theta,
                                float cos_theta,
                                float &i_alpha,
                                float &i_beta)
{
    i_alpha = id * cos_theta - iq * sin_theta;
    i_beta  = id * sin_theta + iq * cos_theta;
}

} // namespace zspinlab::math::function


// Namespace for complex motor control algorithm classes
namespace zspinlab::math::modules {
    class LowPassFirstOrder;
    class PID;
} // namespace zspinlab::math::modules