#include "QuadratureEncoder.hpp"
#include <Arduino.h>

#define EDGES_PER_PULSE 4

using QuadratureEncoder::impl::QuadratureEncoderClass;

/**
 * \brief Constructor for the EncoderClass class.
 * 
 * @param pin_a The digital pin number of the pin connected to wire A of the
 * encoder. Must be interrupt-enabled and belonging to the same port as pin_b.
 * @param pin_b The digital pin number of the pin connected to wire B of the
 * encoder. Must be interrupt-enabled and beloning to the same port as pin_a.
 * @param pulses_per_revolution The number of pulses generated on a single
 * channel of the encoder per revolution. If anyone exceeds the a uint16_t 
 * bounds using this... why are you using this library I wrote for myself
 * @param invert_direction Whether or not to invert the reported direction of
 * the motor.
*/
QuadratureEncoderClass::QuadratureEncoderClass(
    uint8_t pin_a,
    uint8_t pin_b,
    uint16_t pulses_per_revolution,
    bool invert_direction = false
) {
    this->ppr = pulses_per_revolution;
    this->epr = (double) pulses_per_revolution * EDGES_PER_PULSE;
    this->pulses = 0;

    uint8_t pin_a_bm = digital_pin_to_bit_mask_PGM[pin_a];
    uint8_t pin_b_bm = digital_pin_to_bit_mask_PGM[pin_b];

    // By treating pin_x as wire A input and pin_y as wire B input, we can swap
    // the direction reported by the encoder by swapping which pin is assigned
    // to our logical pins.
    if (invert_direction) {
        this->pin_x_bm = pin_b_bm;
        this->pin_y_bm = pin_a_bm;
    } else {
        this->pin_x_bm = pin_a_bm;
        this->pin_y_bm = pin_b_bm;
    }

    // This is an extraoirdinarily hacky way to do this but I just want to get
    // this library up and running ASAP
    #ifdef __AVR_ATmega328P__
        // Setting interrupts 0 and 1 to be triggered on any logic level change, then
        // enabling them.
        // See documentation for EICRA and EIMSK registers to learn more
        EICRA = 0b00000101;
        EIMSK = 0b00000011;
    #else
        #error "Unsupported MCU. Only the ATmega328p is currently supported"
    #endif

    // Using Arduino's slow `pinMode()` here is fine since this constructor will
    // only be called once
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);

}

/**
 * Gets the current reported angle of the encoder in fractional .
*/
QuadratureEncoder::AngleType QuadratureEncoderClass::getAngle() {
    return this->pulses / this->epr;
};

/**
 * \brief Gets the average angular velocity (in ?? TODO:Figure out units) of the
 * encoder over the specified duration.
 * 
 * @param poll_duration Duration over which to find the average angular 
 * velocity, in milliseconds.
 * 
 * A blocking function which finds the average velocity by taking the initial
 * angle, 
*/
QuadratureEncoder::AngleType QuadratureEncoderClass::pollAngularVelocity(
    uint32_t poll_duration
) {
    // Getting initial duration
    QuadratureEncoder::AngleType angle_initial = this->getAngle();

    delay(poll_duration);

    QuadratureEncoder::AngleType angle_final = this->getAngle();

    return (angle_final - angle_initial) / (poll_duration / 1000.0);
}

/**
 * \brief Resets the 0 reference angle.
 * 
 * @param cur_angle The current angle of the encoder with respect to the new
 * zero position.
 * 
 * Resets the datum angle to which angles are reported. Typically used when the
 * angle of the encoder is in a known position, and angles should be reported
 * relative to that position. 
*/
void QuadratureEncoderClass::resetAngleReference(
    QuadratureEncoder::AngleType cur_angle = 0
) {
    this->pulses = (int32_t) (cur_angle * this->epr);
}
