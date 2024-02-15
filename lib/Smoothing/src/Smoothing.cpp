#include "Smoothing.hpp"
#include <Arduino.h>

using namespace Smoothing;

Smoother::Smoother(SmoothedTime half_life, SmoothedType initial_value) {
    this->half_life = half_life;
    this->smoothed_val = initial_value;
}

SmoothedType Smoother::updateSmoothedValue(
    SmoothedType new_val,
    SmoothedTime time_in_state
) {
    this->smoothed_val = (this->smoothed_val - new_val) *
                         pow(0.5, time_in_state / this->half_life)
                         + new_val;

    if (abs(this->smoothed_val) > 10000)
        this->smoothed_val = 0;

    return this->smoothed_val;
}