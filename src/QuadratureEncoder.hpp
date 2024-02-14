#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

/**
 * Currently this library is extremely inflexible with regards to different
 * physical wiring configurations and data collections. It only supports signal
 * A and B being connected to the same GPIO port, and it can only collect full
 * 4x-pulse resolution data (ie collecting rising and falling edges of A and B)
*/

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

namespace QuadratureEncoder {
    typedef double AngleType;

    namespace impl {
        /**
         * Simple class intended to encapsulate measurements taken from an npn-
         * driven quadrature encoder.
        */
        class QuadratureEncoderClass {
            public:
                QuadratureEncoderClass(
                    uint8_t pin_a,
                    uint8_t pin_b,
                    uint16_t pulses_per_revolution,
                    bool invert_direction = false
                );

                AngleType getAngle();
                AngleType pollAngularVelocity(uint32_t poll_duration);

                void resetAngleReference(AngleType cur_angle = 0);

                /** Getter for pointer to angle member. */
                volatile int32_t* getPulseCountPtr()
                { return &(this->pulses); };

                uint8_t pin_x_bm;
                uint8_t pin_y_bm;

                protected:
                    uint16_t ppr;
                    double epr; // Double so we don't have to convert frequently
                    volatile int32_t pulses;
        };
    };

    // Initializing single QuadratureEncloderClass instance for use by user
    QuadratureEncoder::impl::QuadratureEncoderClass Encoder(
        ENCODER_SIGNAL_A_PIN,
        ENCODER_SIGNAL_B_PIN,
        __ENCODER_SWAP_WIRES
    );

    // Namespace to contain some data used by the interrupts, shouldn't be
    // touched by user
    namespace int_data {
        volatile int32_t* pulse_ctr = Encoder.getPulseCountPtr();
        uint8_t wire_x_pin_bm = Encoder.pin_x_bm;
        uint8_t wire_y_pin_bm = Encoder.pin_x_bm;
    };
};

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

// TODO: See what this gets compiled into, see if we need some custom ASM tuning
ISR(ENCODER_SIGNAL_A_INT_VECTOR) {
    uint8_t port_reading = ENCODER_SIGNAL_IN_PORT;
    *QuadratureEncoder::int_data::pulse_ctr += (
        port_reading & QuadratureEncoder::int_data::wire_x_pin_bm
    ) ^ (
        port_reading & QuadratureEncoder::int_data::wire_y_pin_bm
    ) ? +1 : -1;
}

ISR(ENCODER_SIGNAL_B_INT_VECTOR) {
    uint8_t port_reading = ENCODER_SIGNAL_IN_PORT;
    *QuadratureEncoder::int_data::pulse_ctr += (
        port_reading & QuadratureEncoder::int_data::wire_x_pin_bm
    ) ^ (
        port_reading & QuadratureEncoder::int_data::wire_y_pin_bm
    ) ? -1 : +1;
}
#endif

#endif