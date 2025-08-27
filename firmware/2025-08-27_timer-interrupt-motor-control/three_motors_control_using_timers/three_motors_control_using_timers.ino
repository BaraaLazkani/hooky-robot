// Stepper Motor Configuration - Using direct port manipulation
// X-Axis Motor
const uint8_t x_stepPin = 5;    // Arduino Mega Digital Pin 5 (PE3)
#define X_STEP_PORT PORTE
#define X_STEP_BIT (1 << PORTE3)
#define X_STEP_DDR DDRE

const uint8_t x_dirPin = 4;     // Arduino Mega Digital Pin 4 (PG5)
#define X_DIR_PORT PORTG
#define X_DIR_BIT (1 << PORTG5)
#define X_DIR_DDR DDRG

// Y-Axis Motors (share step pin, different direction pins)
const uint8_t y_stepPin = 2;    // Arduino Mega Digital Pin 2 (PE4)
#define Y_STEP_PORT PORTE
#define Y_STEP_BIT (1 << PORTE4)
#define Y_STEP_DDR DDRE

const uint8_t y1_dirPin = 6;    // Arduino Mega Digital Pin 6 (PH3)
#define Y1_DIR_PORT PORTH
#define Y1_DIR_BIT (1 << PORTH3)
#define Y1_DIR_DDR DDRH

const uint8_t y2_dirPin = 7;    // Arduino Mega Digital Pin 7 (PH4)
#define Y2_DIR_PORT PORTH
#define Y2_DIR_BIT (1 << PORTH4)
#define Y2_DIR_DDR DDRH

// Motor and Mechanics Configuration
const float r = 0.03;           // Wheel radius in meters (3 cm)
const float microsteps = 8.0;   // Microstepping setting on the driver
const int steps_per_rev = 200;  // Steps per revolution for the motor (1.8°)

// Calculate distance per step in cm
const float DISTANCE_PER_STEP_CM = (2.0 * M_PI * r * 100.0) / (microsteps * steps_per_rev);

// Global variables for speed and position control
volatile uint32_t x_dt = 0;     // X-axis pulse period in microseconds (µs)
volatile uint32_t y_dt = 0;     // Y-axis pulse period in microseconds (µs)
float v_x = 0;                  // X-axis linear velocity in cm/s
float v_y = 0;                  // Y-axis linear velocity in cm/s
double target_x = 0;             // Target X position in cm
double target_y = 0;             // Target Y position in cm
volatile double current_x = 0;   // Current X position in cm
volatile double current_y = 0;   // Current Y position in cm

const volatile int maxStep_Y = (2.0*75.0 / DISTANCE_PER_STEP_CM);
const volatile int maxStep_Y = (2.0*400.0/ DISTANCE_PER_STEP_CM);

void setupTimer1() {
  X_STEP_DDR |= X_STEP_BIT;
  noInterrupts();          // Temporarily disable all interrupts
  TCCR1A = 0;              // Clear Timer1 Control Register A
  TCCR1B = 0;              // Clear Timer1 Control Register B
  TCNT1  = 0;              // Initialize Timer1 counter value to 0
  TCCR1B |= (1 << WGM12) | (1 << CS11); // Configure Timer1: CTC mode (WGM12=1), Prescaler = 8 (CS11=1)
  TIMSK1 &= ~(1 << OCIE1A); // Initially disable the Timer1 Compare Match A interrupt
  interrupts();             // Re-enable all interrupts
}

void setupTimer3() {
  Y_STEP_DDR |= Y_STEP_BIT;
  noInterrupts();          // Temporarily disable all interrupts
  TCCR3A = 0;              // Clear Timer3 Control Register A
  TCCR3B = 0;              // Clear Timer3 Control Register B
  TCNT3  = 0;              // Initialize Timer3 counter value to 0
  TCCR3B |= (1 << WGM32) | (1 << CS31); // Configure Timer3: CTC mode (WGM32=1), Prescaler = 8 (CS31=1)
  TIMSK3 &= ~(1 << OCIE3A); // Initially disable the Timer3 Compare Match A interrupt
  interrupts();             // Re-enable all interrupts
}
// Interrupt Service Routine for Timer1 Compare Match A (X-axis)
ISR(TIMER1_COMPA_vect) {
  X_STEP_PORT ^= X_STEP_BIT;
  current_x += +int(X_DIR_PORT & X_DIR_BIT) - int(~(X_DIR_PORT & X_DIR_BIT));
  TIMSK1 &= ~(int(current_x>=maxStep_X) << OCIE1A);
}

// Interrupt Service Routine for Timer3 Compare Match A (Y-axis)
ISR(TIMER3_COMPA_vect) {
  Y_STEP_PORT ^= Y_STEP_BIT;
  current_y += -int(Y1_DIR_PORT & Y1_DIR_BIT) + int(~(Y1_DIR_PORT & Y1_DIR_BIT));
  TIMSK3 &= ~(int(current_y>=maxStep_Y) << OCIE3A);
}
void updateTimer1Period() {
  noInterrupts();       // Disable interrupts for safe update
  TCNT1 = 0;            // Reset the timer counter
  if (x_dt > 0) {
    // Calculate timer compare value (16MHz clock with prescaler 8 = 2MHz)
    // Each timer tick is 0.5μs, so we need 2*x_dt ticks for the full period
    uint32_t ocr_value = (2 * x_dt) - 1; // Fixed calculation
    OCR1A = min(ocr_value, 65535); // Ensure it doesn't exceed 16-bit limit
  }
  interrupts();         // Re-enable all interrupts
}
void updateTimer3Period() {
  noInterrupts();       // Disable interrupts for safe update
  TCNT3 = 0;            // Reset the timer counter
  if (y_dt > 0) {
    // Calculate timer compare value (16MHz clock with prescaler 8 = 2MHz)
    // Each timer tick is 0.5μs, so we need 2*y_dt ticks for the full period
    uint32_t ocr_value = (2 * y_dt) - 1; // Fixed calculation
    OCR3A = min(ocr_value, 65535); // Ensure it doesn't exceed 16-bit limit
  }
  
  interrupts();         // Re-enable all interrupts
}

inline uint32_t calculateDtFromVelocity(float velocity_cm_s) {
  const float v_abs = fabs(velocity_cm_s);
  
  // Handle zero velocity case
  if (v_abs < 0.01) return 0;
  
  // Calculate pulse period in microseconds using the same formula as one_motor code
  float time_per_step_us = (1000000.0 * M_PI * r) / (2.0 * microsteps * steps_per_rev * v_abs);
  
  // Apply minimum period (maximum frequency) limit
  return (time_per_step_us < 30) ? 30   : (uint32_t)time_per_step_us;
}

void setup() {
  Serial.begin(115200);

  // Configure the direction pins as outputs using direct port manipulation
  X_DIR_DDR |= X_DIR_BIT;
  Y1_DIR_DDR |= Y1_DIR_BIT;
  Y2_DIR_DDR |= Y2_DIR_BIT;
  
  // Set initial directions
  X_DIR_PORT &= ~X_DIR_BIT;  // Set initial direction to FORWARD
  Y1_DIR_PORT &= ~Y1_DIR_BIT;
  Y2_DIR_PORT |= Y2_DIR_BIT; // Opposite direction to y1
  
  // Initialize timers (dt values will be set from user input)
  setupTimer1();
  setupTimer3();
  
  Serial.println("Dual Axis Stepper Motor Control with Position Limits");
  Serial.println("Enter command as: v_x v_y target_x target_y");
  Serial.println("Speeds in cm/s, positions in cm (relative to 0,0)");
  Serial.println("Positive values = BACKWARD, Negative values = FORWARD");
  Serial.println("Examples:");
  Serial.println("5.0 3.0 10.0 5.0   - X: 5 cm/s to 10 cm, Y: 3 cm/s to 5 cm");
  Serial.println("-2.5 1.0 -5.0 2.0  - X: 2.5 cm/s to -5 cm, Y: 1.0 cm/s to 2 cm");
  Serial.println("0 0 0 0             - Stop all motors");
  Serial.println("----------------------------------------");
}

void processMotorControl() {
  // Handle X-axis motor
  if (fabs(v_x) < 0.01) {
    setXMotorRunning(false);
    Serial.println("X motor stopped");
  } else {
    // Set X direction using direct port manipulation
    if (v_x >= 0) {
      X_DIR_PORT |= X_DIR_BIT;  // BACKWARD
    } else {
      X_DIR_PORT &= ~X_DIR_BIT; // FORWARD
    }

    // Calculate and update X-axis pulse period from input velocity
    x_dt = calculateDtFromVelocity(fabs(v_x));
    updateTimer1Period();
    
    // Start the X motor if it wasn't running
    if (!x_motorRunning) {
      setXMotorRunning(true);
    }
    
    Serial.print("X: v=");
    Serial.print(v_x);
    Serial.print(" cm/s, dt=");
    Serial.print(x_dt);
    Serial.print(" µs, dir=");
    Serial.print(v_x >= 0 ? "BACKWARD" : "FORWARD");
    Serial.print(", target=");
    Serial.print(target_x);
    Serial.println(" cm");
  }
  
  // Handle Y-axis motors
  if (fabs(v_y) < 0.01) {
    setYMotorRunning(false);
    Serial.println("Y motors stopped");
  } else {
    // Set Y directions using direct port manipulation
    if (v_y >= 0) {
      Y1_DIR_PORT |= Y1_DIR_BIT;  // BACKWARD
      Y2_DIR_PORT &= ~Y2_DIR_BIT; // FORWARD (opposite)
    } else {
      Y1_DIR_PORT &= ~Y1_DIR_BIT; // FORWARD
      Y2_DIR_PORT |= Y2_DIR_BIT;  // BACKWARD (opposite)
    }

    // Calculate and update Y-axis pulse period from input velocity
    y_dt = calculateDtFromVelocity(fabs(v_y));
    updateTimer3Period();
    
    // Start the Y motors if they weren't running
    if (!y_motorRunning) {
      setYMotorRunning(true);
    }
    
    Serial.print("Y: v=");
    Serial.print(v_y);
    Serial.print(" cm/s, dt=");
    Serial.print(y_dt);
    Serial.print(" µs, dir=");
    Serial.print(v_y >= 0 ? "BACKWARD" : "FORWARD");
    Serial.print(", target=");
    Serial.print(target_y);
    Serial.println(" cm");
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Read the desired linear speeds and target positions from the Serial monitor
    v_x = Serial.parseFloat();
    v_y = Serial.parseFloat();
    target_x = Serial.parseFloat();
    target_y = Serial.parseFloat();
    
    // Clear the serial buffer
    while (Serial.available() && Serial.read() != '\n');
    
    // Process motor control
    processMotorControl();
    
    Serial.println("----------------------------------------");
  }
  
  // Display current position periodically
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime > 1000) { // Update every second
    lastDisplayTime = millis();
    Serial.print("Current position - X: ");
    Serial.print(current_x);
    Serial.print(" cm, Y: ");
    Serial.print(current_y);
    Serial.println(" cm");
  }
}
