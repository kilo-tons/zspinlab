#include "lpso.hpp"

namespace zspinlab::math::modules
{

    /**
     * @brief Constructor, set initial denominator and numerator coefficients
     * @param[in] a1 The denominator filter coefficient value for z^(-1)
     * @param[in] a2 The denominator filter coefficient value for z^(-2)
     * @param[in] b0 The numerator filter coefficient value for z^0
     * @param[in] b1 The numerator filter coefficient value for z^(-1)
     * @param[in] b2 The numerator filter coefficient value for z^(-2)
     **/
    LowPassSecondOrder::LowPassSecondOrder(float a1, float a2, float b0, float b1, float b2)
    {
        this->a1 = a1;
        this->a2 = a2;

        this->b0 = b0;
        this->b1 = b1;
        this->b2 = b2;

        // Default the initial condition to zero
        this->x1 = 0.0f;
        this->y1 = 0.0f;

        this->x2 = 0.0f;
        this->y2 = 0.0f;
    }

    /**
     * @brief Set the initial inputs and outputs of the filter
     * @param[in] x1 The input value at time sample n=-1
     * @param[in] x2 The input value at time sample n=-2
     * @param[in] y1 The output value at time sample n=-1
     * @param[in] y2 The output value at time sample n=-2
     *
     * @return None
     **/
    void LowPassSecondOrder::set_initial_condition(float x1, float x2, float y1, float y2)
    {
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
    }

    /**
     * @brief Get the filter denominator coefficients
     * @param[out] a1 the denominator filter coefficient value for z^(-1)
     * @param[out] a2 the denominator filter coefficient value for z^(-2)
     *
     * @return None
     **/
    void LowPassSecondOrder::get_denominator_coefficients(float &a1, float &a2)
    {
        a1 = this->a1;
        a2 = this->a2;
    }

    /**
     * @brief Set the filter denominator coefficients
     * @param[in] a1 the denominator filter coefficient value for z^(-1)
     * @param[in] a2 the denominator filter coefficient value for z^(-2)
     *
     * @return None
     **/
    void LowPassSecondOrder::set_denominator_coefficients(float a1, float a2)
    {
        this->a1 = a1;
        this->a2 = a2;
    }

    /**
     * @brief Get the filter numerator coefficients
     * @param[out] b0 the numerator filter coefficient value for z^(0)
     * @param[out] b1 the numerator filter coefficient value for z^(-1)
     * @param[out] b2 the numerator filter coefficient value for z^(-2)
     *
     * @return None
     **/
    void LowPassSecondOrder::get_numerator_coefficients(float &b0, float &b1, float &b2)
    {
        b0 = this->b0;
        b1 = this->b1;
        b2 = this->b2;
    }

    /**
     * @brief Set the filter numerator coefficients
     * @param[in] b0 the numerator filter coefficient value for z^(0)
     * @param[in] b1 the numerator filter coefficient value for z^(-1)
     * @param[in] b2 the numerator filter coefficient value for z^(-2)
     *
     * @return None
     **/
    void LowPassSecondOrder::set_numerator_coefficients(float b0, float b1, float b2)
    {
        this->b0 = b0;
        this->b1 = b1;
        this->b2 = b2;
    }

} // namespace zspinlab::math::modules