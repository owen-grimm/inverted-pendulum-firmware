#include <Arduino.h>

/* To determine the direction of motion when a pulse starts or ends, we can
 * consult the following truth tables:
 * On change in A:
 * A B | DIR
 * 0 0 |  -
 * 0 1 |  +
 * 1 0 |  +
 * 1 1 |  -
 * Dir: A XOR B
 * 
 * On change in B:
 * A B | DIR
 * 0 0 |  +
 * 0 1 |  -
 * 1 0 |  -
 * 1 1 |  +
 * Dir: A XNOR B
 * 
 * Since port A and B are on the same 
 */ 


// The following pin configurations should not be changed except if porting
// code for a different microcontroller.
#define WIRE_IN_PORT    PIND
#define WIRE_A_PIN_NO   2
#define WIRE_A_PIN_BM   1 << WIRE_A_PIN_NO
#define WIRE_B_PIN_NO   3
#define WIRE_B_PIN_BM   1 << WIRE_B_PIN_NO


#define LOOP_MIN_PERIOD 1000 / 60   // In ms
#define COMM_BAURD_RATE 115200

volatile uint32_t pulses = 0;

void setup() {
  // Free to use slower arduino builtins here since setup is not time critical
  pinMode(WIRE_A_PIN_NO, INPUT_PULLUP);
  pinMode(WIRE_B_PIN_NO, INPUT_PULLUP);  

  Serial.begin(COMM_BAURD_RATE);

  // Setting interrupts 0 and 1 to be triggered on any logic level change, then
  // enabling them.
  // See documentation for EICRA and EIMSK registers to learn more
  EICRA = 0b00000101;
  EIMSK = 0b00000011;
}

void loop() {
  Serial.print(">pulses:");
  Serial.println(pulses);
  delay(LOOP_MIN_PERIOD);
}

// Interrupt service routine for wire A
ISR(INT0_vect) {
  uint8_t port_reading = WIRE_IN_PORT;
  pulses += (port_reading & WIRE_A_PIN_BM) ^ (port_reading & WIRE_B_PIN_BM) ?
              +1 : -1;
}

// Interrupt service routine for wire B
ISR(INT1_vect) {
  uint8_t port_reading = WIRE_IN_PORT;
  pulses += (port_reading & WIRE_A_PIN_BM) ^ (port_reading & WIRE_B_PIN_BM) ?
              -1 : +1;
}