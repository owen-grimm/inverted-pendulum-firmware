#include <Arduino.h>
#include <Smoothing.hpp>

// See `define`s at top of file for configuration
#include "QuadratureEncoder.hpp"

#define LOOP_MIN_PERIOD 20   // In ms
#define COMM_BAURD_RATE 115200

QuadratureEncoder& encoder = QuadratureEncoder::getInstance();

void setup() {
  Serial.begin(COMM_BAURD_RATE);
}

// Smoothing
const double VEL_SMOOTHER_HL = 0.02; // half life of smoothing (in s)
Smoothing::Smoother vel_smoother(VEL_SMOOTHER_HL, 0);

void loop() {
  double cur_angle = encoder.getAngle();
  double vel = encoder.getAngularVelocity();
  // double smoothed_vel = vel_smoother.updateSmoothedValue(vel, delta_time);

  // TODO: Control Loop
  
  // TODO: Output to motor controller
  
  Serial.print(">angle:");
  Serial.println(cur_angle, 10);
  Serial.print(">rawvel:");
  Serial.println(vel);
  // Serial.print(">smoothvel:");
  // Serial.println(smoothed_vel);
  // Serial.print(">frametime:");
  // Serial.println(delta_time);

  #if LOOP_MIN_PERIOD > 0
  delay(LOOP_MIN_PERIOD);
  #endif
}
