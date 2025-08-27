#include <avr/io.h>
#include <avr/interrupt.h>

// Pins for motors
#define Y_STEP_PIN 11    // Timer1A (shared for both Y motors)
#define Y1_DIR_PIN 10
#define Y2_DIR_PIN 4
#define X_STEP_PIN 7     // Timer4A
#define X_DIR_PIN 6

// Motor parameters with 8x microstepping
#define MIN_OCR 2000     // 1000 microsteps/sec
#define MAX_OCR 236      // 8488 microsteps/sec (for 1000 mm/s)
#define ACCEL_STEPS 200  // Steps to accelerate/decelerate

// Conversion factors (8x microstepping, 30mm radius disk)
#define STEPS_PER_MM 8.488 // 1600 steps/rev ÷ (2×π×30mm) = 1600/188.5 ≈ 8.488 steps/mm

typedef struct {
  volatile long currentStep;
  long targetSteps;
  int dir;
  unsigned int ocr;
  volatile bool isMoving;
} Motor;

volatile Motor motors[2];  // Now only 2 motors: Y (combined) and X
volatile long moveCount = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper Motor Controller Ready");
  Serial.println("Enter position in format: Y(mm) X(mm)");
  
  // Initialize motor pins
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y1_DIR_PIN, OUTPUT);
  pinMode(Y2_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);

  // Initialize motor structures
  for (int i = 0; i < 2; i++) {
    motors[i].currentStep = 0;
    motors[i].targetSteps = 0;
    motors[i].dir = 0;
    motors[i].ocr = MIN_OCR;
    motors[i].isMoving = false;
  }

  // Configure Timer1 for Y motors (shared)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = MIN_OCR;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8

  // Configure Timer4 for X motor
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = MIN_OCR;
  TCCR4B |= (1 << WGM42) | (1 << CS41); // CTC mode, prescaler 8

  // Configure Timer5 for speed profile
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  OCR5A = 1999; // 1kHz interrupt (16MHz/8/2000)
  TCCR5B |= (1 << WGM52) | (1 << CS51); // CTC mode, prescaler 8
  TIMSK5 |= (1 << OCIE5A); // Enable interrupt

  sei(); // Enable global interrupts
}

void moveMotors(long ySteps, long xSteps) {
  // Stop timers and reset parameters
  TIMSK1 &= ~(1 << OCIE1A);
  TIMSK4 &= ~(1 << OCIE4A);
  
  // Set directions
  motors[0].dir = (ySteps >= 0) ? HIGH : LOW;
  digitalWrite(Y1_DIR_PIN, motors[0].dir);
  digitalWrite(Y2_DIR_PIN, !motors[0].dir); //  Y motors opposite direction

  motors[1].dir = (xSteps >= 0) ? HIGH : LOW;
  digitalWrite(X_DIR_PIN, motors[1].dir);

  // Set target steps (absolute values)
  motors[0].targetSteps = abs(ySteps);
  motors[1].targetSteps = abs(xSteps);

  // Reset counters and enable interrupts
  for (int i = 0; i < 2; i++) {
    motors[i].currentStep = 0;
    motors[i].ocr = MIN_OCR;
    motors[i].isMoving = true;
  }
  moveCount = 0;

  // Enable compare interrupts
  TIMSK1 |= (1 << OCIE1A);
  TIMSK4 |= (1 << OCIE4A);
}

// Wait until all motors stop moving
void waitUntilDone() {
  while (motors[0].isMoving || motors[1].isMoving) {
    // Do nothing, just wait
    delay(1000);
  }
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(Y_STEP_PIN, HIGH);
  digitalWrite(Y_STEP_PIN, LOW);
  motors[0].currentStep++;
  if (motors[0].currentStep >= motors[0].targetSteps) {
    TIMSK1 &= ~(1 << OCIE1A);
    motors[0].isMoving = false;
  }
}

ISR(TIMER4_COMPA_vect) {
  digitalWrite(X_STEP_PIN, HIGH);
  digitalWrite(X_STEP_PIN, LOW);
  motors[1].currentStep++;
  if (motors[1].currentStep >= motors[1].targetSteps) {
    TIMSK4 &= ~(1 << OCIE4A);
    motors[1].isMoving = false;
  }
}

ISR(TIMER5_COMPA_vect) {
  if (!motors[0].isMoving && !motors[1].isMoving) return;

  moveCount++;
  long steps = moveCount;

  for (int i = 0; i < 2; i++) {
    if (!motors[i].isMoving) continue;

    long phase = min(motors[i].targetSteps / 2, min(steps, motors[i].targetSteps - steps));
    phase = constrain(phase, 0, ACCEL_STEPS);

    if (phase < ACCEL_STEPS) {
      motors[i].ocr = MAX_OCR + (MIN_OCR - MAX_OCR) * (ACCEL_STEPS - phase) / ACCEL_STEPS;
    } else {
      motors[i].ocr = MAX_OCR;
    }
  }

  // Update timer compare values
  OCR1A = motors[0].ocr;
  OCR4A = motors[1].ocr;
}

void loop() {
  if (Serial.available() > 0) {
    // Read the position input
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Parse Y and X positions
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      float yPos = input.substring(0, spaceIndex).toFloat();
      float xPos = input.substring(spaceIndex + 1).toFloat();
      
      // Convert mm to steps (8x microstepping)
      long ySteps = round(yPos * STEPS_PER_MM);
      long xSteps = round(xPos * STEPS_PER_MM);
      
      Serial.print("Moving to position: Y=");
      Serial.print(yPos);
      Serial.print("mm, X=");
      Serial.print(xPos);
      Serial.print("mm (");
      Serial.print(ySteps);
      Serial.print(" steps, ");
      Serial.print(xSteps);
      Serial.println(" steps)");
      
      // Move to the specified position
      moveMotors(ySteps, xSteps);
      waitUntilDone();
      
      // Wait 1 second at the position
      delay(1000);
      
      // Return to zero position
      Serial.println("Returning to zero position");
      moveMotors(-ySteps, -xSteps);
      waitUntilDone();
      
      Serial.println("Ready for next command");
      Serial.println("Enter position in format: Y(mm) X(mm)");
    } else {
      Serial.println("Invalid format. Use: Y(mm) X(mm)");
    }
  }
  
  // Small delay to prevent overwhelming the serial buffer
  delay(100);
}
