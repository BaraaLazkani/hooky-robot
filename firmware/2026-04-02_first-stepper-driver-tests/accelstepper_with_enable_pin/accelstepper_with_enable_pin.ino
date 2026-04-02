#define STEP_PIN 5
#define DIR_PIN 4
#define ENA_PIN 8   // choose any free pin

#include <AccelStepper.h>

// STEP, DIR
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  pinMode(ENA_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);  
  // LOW = ENABLE for most TB6600 (sometimes HIGH, test if needed)

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(500);
}

void loop() {
  stepper.moveTo(1000);
  stepper.run();

    //delay(1000);
    stepper.moveTo(400);
     //stepper.run();
    //delay(1000);
}
