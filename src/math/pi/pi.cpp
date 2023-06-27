#include "pi.hpp"

namespace zspinlab::math::modules
{
    /**
     * @brief Initialize the PI controller general parameters
     * @param[in] kP        Proportional gain
     * @param[in] kI        Integral gain
     * @param[in] outMin    Minimum controller output
     * @param[in] outMax    Maximum controller output
     **/
    PI::PI(float kP, float kI, float outMin, float outMax)
    {
        this->kP = kP;
        this->kI = kI;

        this->outMin = outMin;
        this->outMax = outMax;
    }

} // namespace zspinlab::math::modules