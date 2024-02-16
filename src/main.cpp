#include <Arduino.h>
#include <Smoothing.hpp>

// See `define`s at top of file for configuration
#include "QuadratureEncoder.hpp"

#define LOOP_MIN_PERIOD 0   // In ms
#define COMM_BAURD_RATE 115200

QuadratureEncoder& encoder = QuadratureEncoder::getInstance();

void setup() {
  Serial.begin(COMM_BAURD_RATE);
  encoder.setupRegisters();
}

// Smoothing
unsigned long pre_sample_time = 0;
double cur_angle = 0;
double pre_angle = 0;
const double VEL_SMOOTHER_HL = 0.02; // half life of smoothing (in s)
Smoothing::Smoother vel_smoother(VEL_SMOOTHER_HL, 0);

void loop() {
  // TODO: Velocity calculation and smoothing
  // These might be small enough to introduce rounding errors... we shall see
  pre_angle = cur_angle;
  cur_angle = encoder.getAngle();

  unsigned long delta_micros = micros() - pre_sample_time;
  pre_sample_time += delta_micros;
  double delta_time = delta_micros / 1000000.0; // in seconds

  double vel = (cur_angle - pre_angle) / delta_time;  // in rot/s
  double smoothed_vel = vel_smoother.updateSmoothedValue(vel, delta_time);

  // TODO: Control Loop
  
  // TODO: Output to motor controller
  
  Serial.print(">angle:");
  Serial.println(cur_angle, 10);
  Serial.print(">rawvel:");
  Serial.println(vel, 10);
  // Serial.print(">smoothvel:");
  // Serial.println(smoothed_vel);
  // Serial.print(">frametime:");
  // Serial.println(delta_time);

  #if LOOP_MIN_PERIOD > 0
  delay(LOOP_MIN_PERIOD);
  #endif
}
