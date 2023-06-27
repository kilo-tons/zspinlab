#include "current_controller.hpp"

namespace zspinlab::controller
{
    /**
     * @brief Set the Id PI controller general parameters
     * @param[in] kP        Proportional gain
     * @param[in] kI        Integral gain
     * @param[in] min       Minimum controller output
     * @param[in] max       Maximum controller output
     **/
    void CurrentController::set_Id_pi_params(float kP, float kI, float min, float max)
    {
        PI_id.set_kp(kP);
        PI_id.set_ki(kI);
        PI_id.set_outMax(max);
        PI_id.set_outMin(min);
    }

    /**
     * @brief Set the Iq PI controller general parameters
     * @param[in] kP        Proportional gain
     * @param[in] kI        Integral gain
     * @param[in] min       Minimum controller output
     * @param[in] max       Maximum controller output
     **/
    void CurrentController::set_Iq_pi_params(float kP, float kI, float min, float max)
    {
        PI_iq.set_kp(kP);
        PI_iq.set_ki(kI);
        PI_iq.set_outMax(max);
        PI_iq.set_outMin(min);
    }

} // namespace zspinner::controller
