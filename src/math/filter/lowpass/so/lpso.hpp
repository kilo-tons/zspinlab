#pragma once

namespace zspinlab::math::modules
{

    // Create a simple Second-order low-pass filter
    class LowPassSecondOrder
    {
    public:
        LowPassSecondOrder(float a1, float a2, float b0, float b1, float b2);

        // Set the initial inputs and outputs of the filter
        void set_initial_condition(float x1, float x2, float y1, float y2);

        // Get the filter denominator coefficients
        void get_denominator_coefficients(float &a1, float &a2);
        // Set the filter denominator coefficients
        void set_denominator_coefficients(float a1, float a2);

        // Get the filter numerator coefficients
        void get_numerator_coefficients(float &b0, float &b1, float &b2);
        // Set the filter numerator coefficients
        void set_numerator_coefficients(float b0, float b1, float b2);

        // Get the filter denominator coefficient for z^(-1)
        float get_denominator_coefficient_a1(void) { return a1; }
        // Set the filter denominator coefficient for z^(-1)
        void get_denominator_coefficient_a1(float a1) { this->a1 = a1; }

        // Get the filter denominator coefficient for z^(-2)
        float get_denominator_coefficient_a2(void) { return a2; }
        // Set the filter denominator coefficient for z^(-2)
        void get_denominator_coefficient_a2(float a2) { this->a2 = a2; }

        // Get the filter numerator coefficient for z^0
        float get_numerator_coefficient_b0(void) { return b0; }
        // Set the filter numerator coefficient for z^0
        void set_denominator_coefficient_b0(float b0) { this->b0 = b0; }

        // Get the filter numerator coefficient for z^(-1)
        float get_numerator_coefficient_b1(void) { return b1; }
        // Set the filter numerator coefficient for z^(-1)
        void set_denominator_coefficient_b1(float b1) { this->b1 = b1; }

        // Get the filter numerator coefficient for z^(-2)
        float get_numerator_coefficient_b2(void) { return b2; }
        // Set the filter numerator coefficient for z^(-2)
        void set_denominator_coefficient_b2(float b2) { this->b2 = b2; }

        float run(float input);

    private:
        float a1; // the denominator filter coefficient value for z^(-1)
        float a2; // the denominator filter coefficient value for z^(-2)

        float b0; // the numerator filter coefficient value for z^0
        float b1; // the numerator filter coefficient value for z^(-1)
        float b2; // the numerator filter coefficient value for z^(-2)

        float x1; // the input value at time sample n=-1
        float x2; // the input value at time sample n=-2

        float y1; // the output value at time sample n=-1
        float y2; // the output value at time sample n=-2
    };

    /**
     * @brief Run the filter (y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1])
     * @param[in] input Input raw value x[n]
     *
     * @return Output filter value
     **/
    inline float LowPassSecondOrder::run(float input)
    {
        float y0 = (b0 * input) + (b1 * x1) + (b2 * x2) - (a1 * y1) - (a2 * y2);

        // Store new value into previous input and output value
        x1 = input;
        x2 = x1;
        y1 = y0;
        y2 = y1;

        return y0;
    }

} // namespace zspinlab::math::modules