/**
 * Despite the comments in this file, I am refraining from referring to it as a
 * library as its' functionality is only guaranteed on an ATmega328p MCU.
 * Additionally, it is only statically configurable and requires statically 
 * defined configurations within the file. Since this makes it totally unfit
 * for most reuse, I am simply using it as a project file.
*/

#ifndef QUADRATUREENCODER_HPP
#define QUADRATUREENCODER_HPP

#include <Arduino.h>

/**
 * Singleton class representing a connected quadrature encoder. See the 
 * top of the .cpp definition file for configuration options.
*/
class QuadratureEncoder {
    typedef double AngleType;

    public:
        /**
         * Grabs the singleton isntance of this class.
        */
        static QuadratureEncoder& getInstance() {
            static QuadratureEncoder instance;

            return instance;
        };

        // Deleting the copy and assignment operators so no accidental instances
        QuadratureEncoder(QuadratureEncoder const&) = delete;
        void operator=(QuadratureEncoder const&)    = delete;

        AngleType getAngle();
        AngleType pollAngularVelocity(uint32_t poll_duration);

        void resetAngleReference(AngleType cur_angle = 0);

        /** Getter for pointer to angle member. */
        volatile int32_t* getPulseCountPtr()
        { return &(this->pulses); };

        uint8_t pin_x_bm;
        uint8_t pin_y_bm;

    protected:
        QuadratureEncoder();

        uint16_t ppr;
        long double epr; // Double so we don't have to convert frequently
        volatile int32_t pulses;
};

#endif