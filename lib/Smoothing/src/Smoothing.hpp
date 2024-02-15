#ifndef SMOOTHER_HPP
#define SMOOTHER_HPP

typedef double SmoothedType;
typedef double SmoothedTime;

namespace Smoothing {
    class Smoother {
        public:

        Smoother(SmoothedTime half_life, SmoothedType initial_value);

        // Think more about assumption here and in how this is going to be used,
        // will a new value really have been in that state since the last value
        // was recieved? This will hopefully work good nuff for now tho.
        SmoothedType updateSmoothedValue(SmoothedType new_val,
                                         SmoothedTime time_in_state);

        SmoothedType getSmoothedValue() { return this->smoothed_val; };   

        protected:
        SmoothedType smoothed_val;
        SmoothedTime half_life;
    };
}

#endif