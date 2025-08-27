// Stepper Motor Configuration
const uint8_t stepPin = 5;    // Arduino Mega Digital Pin 5 (PE3)
const uint8_t dirPin = 4;     // Arduino Mega Digital Pin 4

// Motor and Mechanics Configuration
const float r = 0.03;         // Wheel radius in meters (3 cm)
const float microsteps = 8.0; // Microstepping setting on the driver
const int steps_per_rev = 200; // Steps per revolution for the motor (1.8°)

// Global variables for speed control
volatile float dt;            // Pulse half-period in microseconds (µs)
float v = 0;                  // Linear velocity in cm/s
bool motorRunning = false;    // Track if motor should be running

void setupTimer1() {
  // Configure the STEP pin as an OUTPUT using direct port manipulation
  DDRE |= (1 << DDE3); // Set PE3 (Arduino Pin 5) as OUTPUT
  
  // Configure Timer1 for CTC mode and precise interrupts
  noInterrupts();          // Temporarily disable all interrupts
  TCCR1A = 0;              // Clear Timer1 Control Register A
  TCCR1B = 0;              // Clear Timer1 Control Register B
  TCNT1  = 0;              // Initialize Timer1 counter value to 0

  // Set initial compare value
  OCR1A = (2.0 * dt) - 1;

  // Configure Timer1: CTC mode (WGM12=1), Prescaler = 8 (CS11=1)
  TCCR1B |= (1 << WGM12) | (1 << CS11);

  // Initially disable the Timer1 Compare Match A interrupt
  TIMSK1 &= ~(1 << OCIE1A);
  interrupts();             // Re-enable all interrupts
}

// Interrupt Service Routine for Timer1 Compare Match A
ISR(TIMER1_COMPA_vect) {
  // Toggle the step pin on each interrupt
  PORTE ^= (1 << PORTE3); // Toggle PE3 (Arduino Pin 5)
}

void updateTimerPeriod() {
  noInterrupts();       // Disable interrupts for safe update
  TCNT1 = 0;            // Reset the timer counter
  OCR1A = (2.0 * dt) - 1; // Update the compare register
  interrupts();         // Re-enable all interrupts
}

void setMotorRunning(bool run) {
  noInterrupts();
  if (run) {
    // Enable timer interrupt
    TIMSK1 |= (1 << OCIE1A);
    motorRunning = true;
  } else {
    // Disable timer interrupt
    TIMSK1 &= ~(1 << OCIE1A);
    // Ensure step pin is low
    digitalWrite(stepPin, LOW);
    motorRunning = false;
  }
  interrupts();
}

float calculateDtFromVelocity(float velocity_cm_s) {
  float v_abs = fabs(velocity_cm_s);
  float calculated_dt;
  
  // Handle zero velocity case
  if (v_abs < 0.01) { // Use a smaller threshold for near-zero
    return 0; // Special value to indicate stop
  }
  
  // Calculate dt using the simplified equation
  calculated_dt = (1000000.0 * M_PI * r) / (2.0 * microsteps * v_abs);
  
  // Apply minimum period (maximum frequency) limit
  if (calculated_dt < 30) calculated_dt = 30;
  
  return calculated_dt;
}

void setup() {
  Serial.begin(115200);

  // Configure the DIR pin as an output
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW); // Set initial direction to FORWARD (reversed logic)
  
  // Initialize dt with a safe, default value
  dt = 2000; // 2000µs = slow pulse
  setupTimer1(); // Initialize Timer1 (but interrupts are disabled initially)
  
  Serial.println("Stepper Motor Control - Linear Speed Input");
  Serial.println("Enter speed in cm/s (positive=BACKWARD, negative=FORWARD):");
  Serial.println("Enter 0 to stop the motor (no pulses)");
  Serial.println("Format: [speed]");
  Serial.println("Example: 5.0 for 5 cm/s BACKWARD");
  Serial.println("Example: -3.5 for 3.5 cm/s FORWARD");
  Serial.println("Example: 0 to stop");
  Serial.println("----------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the desired linear speed from the Serial monitor
    v = Serial.parseFloat();
    
    // Clear the serial buffer
    while (Serial.available() && Serial.read() != '\n');
    
    // Handle zero velocity case
    if (fabs(v) < 0.01) {
      setMotorRunning(false);
      Serial.println("Motor stopped (no pulses)");
      Serial.println("----------------------------------------");
      return;
    }
    
    // Set direction based on the sign of the velocity (REVERSED LOGIC)
    // POSITIVE v -> HIGH (BACKWARD)
    // NEGATIVE v -> LOW (FORWARD)
    digitalWrite(dirPin, v >= 0 ? HIGH : LOW);

    // Calculate the new pulse period 'dt'
    float new_dt = calculateDtFromVelocity(fabs(v)); // Use absolute value for speed calculation
    
    // Print the input and calculated dt
    Serial.print("Input v: ");
    Serial.print(v);
    Serial.print(" cm/s | Calculated dt: ");
    Serial.print(new_dt);
    Serial.println(" µs");
    
    // Update the global dt variable
    dt = new_dt;
    
    // Update the Timer1 period with the new calculated value
    updateTimerPeriod();
    
    // Start the motor if it wasn't running
    if (!motorRunning) {
      setMotorRunning(true);
    }

    // Additional debug information
    Serial.print("Direction: ");
    Serial.println(v >= 0 ? "BACKWARD" : "FORWARD");
    Serial.print("DIR Pin State: ");
    Serial.println(v >= 0 ? "HIGH" : "LOW");
    Serial.print("Timer OCR1A value: ");
    Serial.println((2.0 * dt) - 1);
    Serial.println("----------------------------------------");
  }
}
