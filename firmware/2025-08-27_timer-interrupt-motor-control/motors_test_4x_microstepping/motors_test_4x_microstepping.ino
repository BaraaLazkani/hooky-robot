#define Y1_STEP_PIN PA0 // Pin [22] Arduino Mega.
#define Y2_STEP_PIN PA1 // Pin [23] Arduino Mega.
#define X_STEP_PIN PA2  // Pin [24] Arduino Mega.

#define Y1_DIR_PIN PC0  // Pin [37] Arduino Mega.
#define Y2_DIR_PIN PC1  // Pin [36] Arduino Mega.
#define X_DIR_PIN PC2   // Pin [35] Arduino Mega.

#define r 0.03  // Roller radius [m]->[30mm]->[3cm].

#define microsteps_X 8.0 //Microstepping Factor X_AXIS.
#define microsteps_Y 8.0 //Microstepping Factor Y_AXIS.

#define STEPS_PER_REV_X (200.0 * microsteps_X)
#define STEPS_PER_REV_Y (200.0 * microsteps_Y)

#define rad2step_freq_x(w) (w * (STEPS_PER_REV_X / (2.0 * M_PI)))
#define rad2step_freq_y(w) (w * (STEPS_PER_REV_Y / (2.0 * M_PI)))

#define oneStep2m_x float((2.0 * M_PI * r)/((200.0*microsteps_X))) // one step in meters.
#define oneStep2m_y float((2.0 * M_PI * r)/((200.0*microsteps_Y))) // one step in meters.

#define linear2step_freq_x(v) (rad2step_freq_x(v / r))
#define linear2step_freq_y(v) (rad2step_freq_y(v / r))

#define max_x_pos_steps int(50.0/100.0 / oneStep2m_x) 
#define max_y_pos_steps int(70.0/100.0 / oneStep2m_y)

#define MAX_X_TOGGLES (2 * max_x_pos_steps)
#define MAX_Y_TOGGLES (2 * max_y_pos_steps)

volatile long x_step_count = 0;
volatile long y_step_count = 0;
volatile bool x_moving = false;
volatile bool y_moving = false;
volatile int x_toggle_counter = 0;
volatile int y_toggle_counter = 0;

// Global velocity variables
double dt_x = 1000, dt_y = 1000; // Initialize with safe values
double wx = 0, vx = 0, wy = 0, vy = 0; // Angular/Linear velocities [step/s]/[m/s].

void start_X_motor(bool state) {
  TIMSK1 = (state) ? (TIMSK1 | (1 << OCIE1A)) : (TIMSK1 & ~(1 << OCIE1A));
  x_moving = state;
}

void start_Y_motor(bool state) {
  TIMSK3 = (state) ? (TIMSK3 | (1 << OCIE3A)) : (TIMSK3 & ~(1 << OCIE3A));
  y_moving = state;
}

void setupTimer_X() {
  noInterrupts();         // Disable interrupts
  start_X_motor(false);
  x_step_count = 0;       // Reset step counter
  TCCR1A = 0;             // Clear control register A
  TCCR1B = 0;             // Clear control register B
  TCNT1  = 0;             // Initialize counter value to 0
  // Set compare match register to desired value (dt [us])
  OCR1A = (dt_x > 0) ? (2.0 * dt_x) - 1 : 0;
  TCCR1B |= (1 << WGM12); // Set CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);  // Set CS11 bit for 8 prescaler (gives us 2 MHz clock)
  interrupts();           // Enable interrupts
}

void setupTimer_Y() {
  noInterrupts();         // Disable interrupts
  start_Y_motor(false);
  y_step_count = 0;       // Reset step counter
  TCCR3A = 0;             // Clear control register A
  TCCR3B = 0;             // Clear control register B
  TCNT3  = 0;             // Initialize counter value to 0
  // Set compare match register to desired value (dt [us])
  OCR3A = (dt_y > 0) ? (2.0 * dt_y) - 1 : 0;
  TCCR3B |= (1 << WGM32); // Set CTC mode (Clear Timer on Compare Match)
  TCCR3B |= (1 << CS31);  // Set CS31 bit for 8 prescaler (gives us 2 MHz clock)
  interrupts();           // Enable interrupts
}

void changeTimer_X_desiredTime(double dt) {
  noInterrupts();
  start_X_motor(false);
  TCNT1 = 0; 
  OCR1A = (2.0 * dt) - 1; // CORRECTED: 2.0 instead of 4.0
  start_X_motor(true);
  interrupts();
}

void changeTimer_Y_desiredTime(double dt) {
  noInterrupts();
  start_Y_motor(false);
  TCNT3 = 0; 
  OCR3A = (2.0 * dt) - 1; // CORRECTED: 2.0 instead of 4.0
  start_Y_motor(true);
  interrupts();
}

volatile unsigned long x_toggle_count = 0;
volatile unsigned long y_toggle_count = 0;

ISR(TIMER1_COMPA_vect) {
  // Toggle step pin
  PORTA ^= (1 << X_STEP_PIN);
  
  // Update toggle count based on direction with minimal condition checks
  if (vx != 0) {
    // Use a single conditional check for both directions
    int direction = (vx > 0) ? 1 : -1;
    long new_count = x_toggle_count + direction;
    
    // Check if new count is within bounds
    if ((direction > 0 && new_count <= MAX_X_TOGGLES) || 
        (direction < 0 && new_count >= 0)) {
      x_toggle_count = new_count;
    } else {
      // Limit reached, stop motor
      start_X_motor(false);
      vx = 0;
    }
  }
}

ISR(TIMER3_COMPA_vect) {
  // Toggle both Y step pins simultaneously
  PORTA ^= ((1 << Y1_STEP_PIN) | (1 << Y2_STEP_PIN));
  
  // Update toggle count based on direction with minimal condition checks
  if (vy != 0) {
    // Use a single conditional check for both directions
    int direction = (vy > 0) ? 1 : -1;
    long new_count = y_toggle_count + direction;
    
    // Check if new count is within bounds
    if ((direction > 0 && new_count <= MAX_Y_TOGGLES) || 
        (direction < 0 && new_count >= 0)) {
      y_toggle_count = new_count;
    } else {
      // Limit reached, stop motor
      start_Y_motor(false);
      vy = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  DDRA |= (1 << Y1_STEP_PIN) | (1 << Y2_STEP_PIN) | (1 << X_STEP_PIN);
  DDRC |= (1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN) | (1 << X_DIR_PIN);

  PORTA = 0;
  PORTC = 0;

  setupTimer_X();
  setupTimer_Y();
  
  Serial.println("Ready - Send velocities as 'Y_velocity X_velocity' in cm/s");
}

void loop() {
  if (Serial.available()) {
    // Read both velocities
    vy = Serial.parseFloat() / 100.0; // Convert cm/s to m/s for Y
    vx = Serial.parseFloat() / 100.0; // Convert cm/s to m/s for X
    
    // Clear serial buffer
    while (Serial.available() && Serial.read() != '\n');
    
    // Calculate timing for both axes
    double step_freq_y = linear2step_freq_y(fabs(vy));
    double step_freq_x = linear2step_freq_x(fabs(vx));
    
    // Prevent division by zero
    dt_y = (step_freq_y > 0.1) ? (1000000.0 / (2.0 * step_freq_y)) : 0;
    dt_x = (step_freq_x > 0.1) ? (1000000.0 / (2.0 * step_freq_x)) : 0;
    if(dt_y>0) changeTimer_Y_desiredTime(dt_y); else {start_Y_motor(false); vy = 0;}
    if(dt_x>0) changeTimer_X_desiredTime(dt_x); else {start_X_motor(false); vx = 0;}
    
// Set Y direction and check limits
    if (vy > 0) {
      PORTC |= (1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN);
    } else if (vy < 0) {
      PORTC &= ~((1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN));
    }
    
    // Set X direction
    if (vx > 0) {
      PORTC |= (1 << X_DIR_PIN);
    } else if (vx < 0) {
      PORTC &= ~(1 << X_DIR_PIN);
    }
    // Display calculations
    Serial.print("wy: ");
    Serial.print(step_freq_y);
    Serial.print(" \twx: ");
    Serial.print(step_freq_x);
    Serial.print(" \tvy: ");
    Serial.print(vy * 100);
    Serial.print("\t vx: ");
    Serial.print(vx * 100);
    Serial.print("\tY_dt: ");
    Serial.print(dt_y);
    Serial.print("\tX_dt: ");
    Serial.println(dt_x);
  }
}
