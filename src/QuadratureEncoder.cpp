#include "QuadratureEncoder.hpp"
#include <Arduino.h>

/* ============= CONFIG HERE ============= */

#define ENCODER_SIGNAL_A_PIN 2
#define ENCODER_SIGNAL_A_INT_VECTOR INT0_vect
#define ENCODER_SIGNAL_B_PIN 3
#define ENCODER_SIGNAL_B_INT_VECTOR INT1_vect
#define ENCODER_SIGNAL_IN_PORT PIND
#define ENCODER_PULSES_PER_REVOLUTION 3600

/* ======================================= */

// Don't change these ones tho...
#define EDGES_PER_PULSE 2

// Unfortunately, because there's no good way to grab pin ports registers,
// check their interrupt vectors, or generate ISRs (without Arduino's high-
// latency `attachInterrupt()`), information on all of these must be passed to
// the library via compile-time known `#define`s to ensure that this library
// works quickly. It is the user's responsibility to ensure all related 
// `#define`s are made correctly to ensure correct funtionality.
//
// For descriptions of each of these required `#defines`, see below.
#if defined(ENCODER_SIGNAL_A_PIN)           && \
    defined(ENCODER_SIGNAL_A_INT_VECTOR)    && \
    defined(ENCODER_SIGNAL_B_PIN)           && \
    defined(ENCODER_SIGNAL_B_INT_VECTOR)    && \
    defined(ENCODER_SIGNAL_IN_PORT)

#if digitalPinToInterrupt(ENCODER_SIGNAL_A_PIN) == NOT_AN_INTERRUPT
    #error "Signal A pin must be interrupt-enabled."
#endif
#if digitalPinToInterrupt(ENCODER_SIGNAL_B_PIN) == NOT_AN_INTERRUPT
    #error "Signal B pin must be interrupt-enabled."
#endif

#define ISR_COMPILABLE

#else

// `ENCODER_SIGNAL_A_PIN` should be the Arduino digital pin number of the
// digital pin to which the "A" signal of the quadrature encoder is attached.
// Example: `#define ENCODER_SIGNAL_A_PIN 2`
#ifndef ENCODER_SIGNAL_A_PIN
    #error "`ENCODER_WIRE_A_PIN` must be defined for library use."
#endif

// `ENCODER_SIGNAL_B_PIN` should be the Arduino digital pin number of the
// digital pin to which the "B" signal of the quadrature encoder is attached.
// Example: `#define ENCODER_SIGNAL_B_PIN 3`
#ifndef ENCODER_SIGNAL_B_PIN
    #error "`ENCODER_WIRE_B_PIN` must be defined for library use."
#endif

// `ENCODER_SIGNAL_A_INT_VECTOR` should be the interrupt vector of the on-change
// interrupt of the digital pin to which the "A" signal of the quadrature
// encoder is attached. This MUST be the interrupt vector for the corresponding
// pin otherwise this library will not work.
// Example: `#define ENCODER_SIGNAL_A_INT_VECTOR INT0_vect`
#ifndef ENCODER_SIGNAL_A_INT_VECTOR
    #error "`ENCODER_SIGNAL_A_INT_VECTOR must be defined for library use"

#endif
// `ENCODER_SIGNAL_B_INT_VECTOR` should be the interrupt vector of the on-change
// interrupt of the digital pin to which the "B" signal of the quadrature
// encoder is attached. This MUST be the interrupt vector for the corresponding
// pin otherwise this library will not work.
// Example: `#define ENCODER_SIGNAL_B_INT_VECTOR INT1_vect`
#ifndef ENCODER_SIGNAL_B_INT_VECTOR
    #error "`ENCODER_SIGNAL_B_INT_VECTOR must be defined for library use"
#endif

// `ENCODER_SIGNAL_IN_PORT` should be the Portx input register for the digital
// pins to which both the "A" and "B" signals of the quadrature encoder are
// attached. These two pins MUST be on the same port to allow the fastest
// possible interrupt latency and avoid missing pulses. This MUST be the port to
// which BOTH encoder signal wires are attached, otherwise this library will not
// work.
// Example: `#define ENCODER_SIGNAL_IN_PORT PIND`
#ifndef ENCODER_SIGNAL_IN_PORT
    #error "`ENCODER_SIGNAL_IN_PORT` must be defined for library use."
#endif

#endif

// `ENCODER_PULSES_PER_REVOLUTION` should be the number of pulses generated per
// revolution by the attached quadrature encoder. Note that this library uses 
// rising and falling edges of the A and B signals of the encoder, meaning that
// angular displacement will be measured 4x more frequently than the specified
// PPR value.
#ifndef ENCODER_PULSES_PER_REVOLUTION
    #error "`ENCODER_PULSES_PER_REVOLUTION` must be defined for library use."
#endif

// `ENCODER_REVERSE` should be defined if the reported angular displacement of
// the encoder should be inverted (+/- movement directions swapped).
#ifdef ENCODER_REVERSE
#define __ENCODER_SWAP_WIRES 1
#else
#define __ENCODER_SWAP_WIRES 0
#endif

/**
 * \brief Constructor for the QuadratureEncoderClass class.
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
QuadratureEncoder::QuadratureEncoder() {
    this->ppr = ENCODER_PULSES_PER_REVOLUTION;
    this->epr = (double) (this->ppr * EDGES_PER_PULSE);
    this->pulses = 0;

    uint8_t pin_a_bm = digital_pin_to_bit_mask_PGM[ENCODER_SIGNAL_A_PIN];
    uint8_t pin_b_bm = digital_pin_to_bit_mask_PGM[ENCODER_SIGNAL_B_PIN];

    // By treating pin_x as wire A input and pin_y as wire B input, we can swap
    // the direction reported by the encoder by swapping which pin is assigned
    // to our logical pins.
    if (__ENCODER_SWAP_WIRES) {
        this->pin_x_bm = pin_b_bm;
        this->pin_y_bm = pin_a_bm;
    } else {
        this->pin_x_bm = pin_a_bm;
        this->pin_y_bm = pin_b_bm;
    }

    // This is an extraoirdinarily hacky way to do this but I just want to get
    // this library up and running tbh
    #ifdef __AVR_ATmega328P__
        // Since there are only two interrupt-enabled GPIO pins on the
        // ATmega328p, it is "safe" to assume that both are being used by this
        // library. 
        // Setting interrupts 0 and 1 to be triggered on any logic level change,
        // then enabling them.
        // See documentation for EICRA and EIMSK registers to learn more
        EICRA = 0b00000101;     // Setting int0 and int1 to on-change trigger
        EIMSK = 0b00000011;     // Enabling int0 and int1
    #else
        #error "Unsupported MCU. Only the ATmega328p is currently supported"
    #endif

    // Using Arduino's slow `pinMode()` here is fine since this constructor will
    // only be called once
    pinMode(ENCODER_SIGNAL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_SIGNAL_B_PIN, INPUT_PULLUP);

}

/**
 * Gets the current reported angle of the encoder in fractional rotations.
*/
QuadratureEncoder::AngleType QuadratureEncoder::getAngle() {
    return this->pulses / (this->epr);
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
QuadratureEncoder::AngleType QuadratureEncoder::pollAngularVelocity(
    uint32_t poll_duration
) {
    // Getting initial pos
    QuadratureEncoder::AngleType angle_initial = this->getAngle();
    // Waiting
    delay(poll_duration);
    // Getting final pos
    QuadratureEncoder::AngleType angle_final = this->getAngle();

    // Calculating average velocity
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
void QuadratureEncoder::resetAngleReference(
    QuadratureEncoder::AngleType cur_angle
) {
    this->pulses = (int32_t) (cur_angle * this->epr);
}

#ifdef ISR_COMPILABLE
/* To determine the direction of motion when a pulse starts or ends, we can
 * consult the following truth tables:
 * On change in A:
 * A B | DIR
 * 0 0 |  -
 * 0 1 |  +
 * 1 0 |  +
 * 1 1 |  -
 * 
 * On change in B:
 * A B | DIR
 * 0 0 |  +
 * 0 1 |  -
 * 1 0 |  -
 * 1 1 |  +
 */ 

// Namespace to contain some data used by the interrupts, shouldn't be
// touched by user
namespace __QuadratureEncoderISRData {
    QuadratureEncoder& encoder = QuadratureEncoder::getInstance();
    volatile int32_t* pulse_ctr = encoder.getPulseCountPtr();
    uint8_t wire_x_pin_bm = encoder.pin_x_bm;
    uint8_t wire_y_pin_bm = encoder.pin_y_bm;
};

// TODO: See what this gets compiled into, see if we need some custom ASM tuning
ISR(ENCODER_SIGNAL_A_INT_VECTOR) {
    uint8_t port_reading = ENCODER_SIGNAL_IN_PORT;
    *__QuadratureEncoderISRData::pulse_ctr += (
        port_reading & __QuadratureEncoderISRData::wire_x_pin_bm
    ) ^ (
        port_reading & __QuadratureEncoderISRData::wire_y_pin_bm
    ) ? +1 : -1;
}

ISR(ENCODER_SIGNAL_B_INT_VECTOR) {
    uint8_t port_reading = ENCODER_SIGNAL_IN_PORT;
    *__QuadratureEncoderISRData::pulse_ctr += (
        port_reading & __QuadratureEncoderISRData::wire_x_pin_bm
    ) ^ (
        port_reading & __QuadratureEncoderISRData::wire_y_pin_bm
    ) ? -1 : +1;
}

#endif
