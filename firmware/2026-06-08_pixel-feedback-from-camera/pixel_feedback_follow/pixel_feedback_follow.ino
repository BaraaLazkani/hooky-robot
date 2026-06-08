// X AXIS MOTOR
const int STEP_PIN_X = 24;
const int DIR_PIN_X = 35;

// Y AXIS MOTORS
const int STEP_PIN_Y1 = 23;
const int DIR_PIN_Y1 = 18;
const int STEP_PIN_Y2 = 22;
const int DIR_PIN_Y2 = 19;

// MECHANICAL PARAMETERS
#define r 0.0305
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)

// CURRENT POSITION
const float START_X_CM = 40.8;
const float START_Y_CM = 9.2;
float x_pos = 0;
float y_pos = 0;

float pixel_x = 0;
float pixel_y = 0;

// STEP COUNTERS
long x_steps = 0;
long y_steps = 0;

// TARGET STEPS (globals, only updated on new serial command)
long target_x_steps = 0;
long target_y_steps = 0;

// TARGET POSITION (cm)
float target_x = START_X_CM;
float target_y = START_Y_CM;

// MAX SPEEDS (cm/s)
const float X_MAX_SPEED = 100.0;
const float Y_MAX_SPEED = 60.0;

// PROPORTIONAL GAIN
const float X_GAIN = 100.0;
const float Y_GAIN = 3.0;

// DEADZONE (cm)
const float X_DEADZONE = 1;
const float Y_DEADZONE = 1;

// Y AXIS LIMITS
const float Y_MAX_LIMIT = 30.0;   // Robot never moves beyond this
const float Y_HOME_TRIGGER = 130.0; // If goal Y exceeds this, return home

// NON-BLOCKING TIMERS
unsigned long last_x_step_time = 0;
unsigned long last_y_step_time = 0;
unsigned long x_step_interval_us = 0;
unsigned long y_step_interval_us = 0;

// =========================================
// Convert cm -> motor steps
// =========================================
long cm_to_steps(float cm)
{
  float meters = cm / 100.0;
  float circumference = 2.0 * PI * r;
  float revs = meters / circumference;
  return (long)(revs * STEPS_PER_REV);
}

// =========================================
// Convert steps -> cm
// =========================================
float steps_to_cm(long steps)
{
  float revs = (float)steps / STEPS_PER_REV;
  float meters = revs * (2.0 * PI * r);
  return meters * 100.0;
}

// =========================================
// Convert speed (cm/s) -> step interval (μs)
// =========================================
unsigned long speed_to_interval(float speed_cm_s)
{
  float cm_per_step = (2.0 * PI * r * 100.0) / STEPS_PER_REV;
  float steps_per_second = speed_cm_s / cm_per_step;
  if (steps_per_second <= 0.0)
    return 1000000UL;
  return (unsigned long)(1000000.0 / steps_per_second);
}

// =========================================
// STEP X MOTOR
// =========================================
void step_x(bool dir)
{
  digitalWrite(DIR_PIN_X, dir);
  digitalWrite(STEP_PIN_X, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN_X, LOW);
  if (dir)
    x_steps--; // flipped to match physical direction
  else
    x_steps++; // flipped to match physical direction
}

// =========================================
// STEP BOTH Y MOTORS
// =========================================
void step_y(bool dir)
{
  digitalWrite(DIR_PIN_Y1, dir);
  digitalWrite(DIR_PIN_Y2, dir);
  digitalWrite(STEP_PIN_Y1, HIGH);
  digitalWrite(STEP_PIN_Y2, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN_Y1, LOW);
  digitalWrite(STEP_PIN_Y2, LOW);
  if (dir)
    y_steps++;
  else
    y_steps--;
}

// =========================================
// READ SERIAL
// =========================================
void readSerial()
{
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);

  if (c1 < 0 || c2 < 0 || c3 < 0)
    return;

  float center_x = line.substring(0, c1).toFloat();
  float center_y = line.substring(c1 + 1, c2).toFloat();

  pixel_x = line.substring(c2 + 1, c3).toFloat();
  pixel_y = line.substring(c3 + 1).toFloat();

  // store center values if needed
  target_x = center_x;
  target_y = center_y;

  // current position comes directly from image pixels
  x_pos = pixel_x;
  y_pos = pixel_y;
}

// =========================================
// SETUP
// =========================================
void setup()
{
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y1, OUTPUT);
  pinMode(DIR_PIN_Y1, OUTPUT);
  pinMode(STEP_PIN_Y2, OUTPUT);
  pinMode(DIR_PIN_Y2, OUTPUT);
  digitalWrite(STEP_PIN_X, LOW);
  digitalWrite(STEP_PIN_Y1, LOW);
  digitalWrite(STEP_PIN_Y2, LOW);

  // Initialise step counters to start position
  x_steps = cm_to_steps(START_X_CM);
  y_steps = cm_to_steps(START_Y_CM);

  // Initialise targets to start position (robot won't move on boot)
  target_x_steps = cm_to_steps(START_X_CM);
  target_y_steps = cm_to_steps(START_Y_CM);

  Serial.println("System Ready");
}

// =========================================
// MAIN LOOP
// =========================================
void loop()
{
  readSerial();
  unsigned long now = micros();

  // =====================================
  // PROPORTIONAL SPEED FOR X
  // =====================================
  float x_error_cm = steps_to_cm(target_x_steps) - x_pos;
  float x_speed = abs(x_error_cm) * X_GAIN;
  x_speed = constrain(x_speed, 0.0, X_MAX_SPEED);

  // =====================================
  // MOVE X AXIS
  // =====================================
  if (abs(x_error_cm) > X_DEADZONE)
  {
    x_step_interval_us = speed_to_interval(x_speed);
    if (now - last_x_step_time >= x_step_interval_us)
    {
      last_x_step_time = now;
      if (x_error_cm > 0)
        step_x(false);
      else
        step_x(true);
    }
  }
  target_x_steps = cm_to_steps(target_x);
  target_y_steps = cm_to_steps(target_y);

  // =====================================
  // PROPORTIONAL SPEED FOR Y
  // =====================================
  float y_error_cm = steps_to_cm(target_y_steps) - y_pos;
  float y_speed = abs(y_error_cm) * Y_GAIN;
  y_speed = constrain(y_speed, 0.0, Y_MAX_SPEED);

  // =====================================
  // MOVE Y AXIS
  // =====================================
  if (abs(y_error_cm) > Y_DEADZONE)
  {
    y_step_interval_us = speed_to_interval(y_speed);
    if (now - last_y_step_time >= y_step_interval_us)
    {
      last_y_step_time = now;
      if (y_error_cm > 0)
        step_y(true); // flip if Y goes wrong direction
      else
        step_y(false);
    }
  }

  // =====================================
  // PRINT POSITION
  // =====================================
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100)
  {
    lastPrint = millis();
    Serial.print("CenterX=");
Serial.print(target_x);

Serial.print(" CenterY=");
Serial.print(target_y);

Serial.print(" PixelX=");
Serial.print(pixel_x);

Serial.print(" PixelY=");
Serial.print(pixel_y);

Serial.print(" x_pos=");
Serial.print(x_pos);

Serial.print(" y_pos=");
Serial.println(y_pos);
  }
}
