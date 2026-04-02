#define STEP_PIN 26
#define DIR_PIN 35
#include <AccelStepper.h>

// TB6600 uses DRIVER mode (STEP, DIR)
AccelStepper stepper(AccelStepper::DRIVER, 24, 35);
// (STEP pin, DIR pin)

void setup() {
  stepper.setMaxSpeed(1000);     // speed
  stepper.setAcceleration(500);  // acceleration
}

void loop() {
  stepper.moveTo(1000);   // move forward
  stepper.run();

  if (stepper.distanceToGo() == 0) {
    delay(1000);
    stepper.moveTo(-1000); // move backward
  }
}