#pragma once

// Create a simple first-order low-pass filter
class LowPassFirstOrder {
public:
    LowPassFirstOrder(float a1, float b0, float b1);

    void set_initial_condition(float x1, float y1);

    // Get the filter denominator coefficient
    float get_denominator_coefficient(void) { return a1; }
    // Set the filter denominator coefficient
    void set_denominator_coefficient(float a1) { this->a1 = a1; }

    // Get the filter numerator coefficient for z^0
    float get_numerator_coefficient_b0(void) { return b0; }
    // Set the filter numerator coefficient for z^0
    void set_denominator_coefficient_b0(float b0) { this->b0 = b0; }

    // Get the filter numerator coefficient for z^1
    float get_numerator_coefficient_b1(void) { return b1; }
    // Set the filter numerator coefficient for z^1
    void set_denominator_coefficient_b1(float b0) { this->b1 = b1; }

    float run(float input);


private:
    float  a1;          // the denominator filter coefficient value for z^(-1)

    float  b0;          // the numerator filter coefficient value for z^0
    float  b1;          // the numerator filter coefficient value for z^(-1)

    float  x1;          // the input value at time sample n=-1
    float  y1;          // the output value at time sample n=-1
};

/**
 * @brief Run the filter (y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1])
 * @param[in] input Input raw value x[n]
 * 
 * @return Output filter value
 **/
inline float LowPassFirstOrder::run(float input) {
    float y0 = b0*input + b1*x1 - a1*y1;

    // Store new value into previous input and output value
    x1 = input;
    y1 = y0;

    return y0;
}