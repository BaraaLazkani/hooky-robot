//
// X AXIS MOTOR
//
const int STEP_PIN_X = 24;
const int DIR_PIN_X  = 35;

//
// Y AXIS MOTORS
//
const int STEP_PIN_Y1 = 23;
const int DIR_PIN_Y1  = 18;

const int STEP_PIN_Y2 = 22;
const int DIR_PIN_Y2  = 19;


//
// MECHANICAL PARAMETERS
//
#define r 0.0305
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)


//
// CURRENT POSITION - tracked INTERNALLY via step counting.
// This is deliberate: step counting is deterministic and noise-free,
// unlike a camera reading of the robot's own position, which jitters
// frame to frame and causes erratic direction changes if used directly
// in the control loop.
//
const float START_X_CM = 40.7;
const float START_Y_CM = 8.8;

float x_pos = START_X_CM;
float y_pos = START_Y_CM;


//
// STEP COUNTERS (ground truth of robot position)
//
long x_steps = 0;
long y_steps = 0;


//
// TARGET STEPS - this is the BALL position, converted to steps.
// Updated once per new serial line, not recomputed every loop.
//
long target_x_steps = 0;
long target_y_steps = 0;


//
// TARGET POSITION (cm) - the ball's position, from the camera
//
float target_x = START_X_CM;
float target_y = START_Y_CM;


//
// ARRIVED FLAGS
//
bool x_arrived = true;
bool y_arrived = true;


//
// PROPORTIONAL SPEED CONTROL
//
const float X_MAX_SPEED = 40.0;   // cm/s
const float Y_MAX_SPEED = 25.0;   // cm/s
const float X_GAIN = 6.0;         // (cm/s) per cm of error
const float Y_GAIN = 6.0;

const float X_DEADZONE_CM = 0.3;  // stop trying once within this distance
const float Y_DEADZONE_CM = 0.3;


//
// NON-BLOCKING TIMERS
//
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
// Convert speed (cm/s) -> step interval (us)
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
        x_steps--;
    else
        x_steps++;
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
// Expected line: robotX,robotY,ballX,ballY
// We only use ballX,ballY (fields 3 and 4) as the target.
// robotX,robotY are intentionally ignored for motion control -
// our own step count is the trusted position source.
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
        return; // malformed line - ignore, keep last good target

    // fields: robotX (unused), robotY (unused), ballX, ballY
    float ball_x = line.substring(c2 + 1, c3).toFloat();
    float ball_y = line.substring(c3 + 1).toFloat();

    target_x = ball_x;
    target_y = ball_y;

    // Computed ONCE per received command, not every loop
    target_x_steps = cm_to_steps(target_x);
    target_y_steps = cm_to_steps(target_y);

    // New target -> allow motion again
    x_arrived = false;
    y_arrived = false;

    Serial.print("New Target: ");
    Serial.print(target_x);
    Serial.print(", ");
    Serial.println(target_y);
}


// =========================================
// SETUP
// =========================================
void setup()
{
    Serial.begin(115200);

    pinMode(STEP_PIN_X,  OUTPUT);
    pinMode(DIR_PIN_X,   OUTPUT);
    pinMode(STEP_PIN_Y1, OUTPUT);
    pinMode(DIR_PIN_Y1,  OUTPUT);
    pinMode(STEP_PIN_Y2, OUTPUT);
    pinMode(DIR_PIN_Y2,  OUTPUT);

    digitalWrite(STEP_PIN_X,  LOW);
    digitalWrite(STEP_PIN_Y1, LOW);
    digitalWrite(STEP_PIN_Y2, LOW);

    // Initialise step counters to start position
    x_steps = cm_to_steps(START_X_CM);
    y_steps = cm_to_steps(START_Y_CM);

    // Initialise targets to start position (robot won't move on boot)
    target_x_steps = cm_to_steps(START_X_CM);
    target_y_steps = cm_to_steps(START_Y_CM);

    // Robot starts already at target so mark as arrived
    x_arrived = true;
    y_arrived = true;

    Serial.println("System Ready");
}


// =========================================
// MAIN LOOP
// =========================================
void loop()
{
    readSerial();

    unsigned long now = micros();

    // Current position, from our own trusted step count
    x_pos = steps_to_cm(x_steps);
    y_pos = steps_to_cm(y_steps);


    // =====================================
    // MOVE X AXIS - proportional speed
    // =====================================
    if (!x_arrived)
    {
        float x_error_cm = steps_to_cm(target_x_steps) - x_pos;

        if (abs(x_error_cm) > X_DEADZONE_CM)
        {
            float x_speed = constrain(abs(x_error_cm) * X_GAIN, 0.0, X_MAX_SPEED);
            x_step_interval_us = speed_to_interval(x_speed);

            if (now - last_x_step_time >= x_step_interval_us)
            {
                last_x_step_time = now;

                if (x_error_cm > 0)
                    step_x(false);   // flipped - matches your validated wiring
                else
                    step_x(true);
            }
        }
        else
        {
            x_arrived = true;
            Serial.println("X arrived");
        }
    }


    // =====================================
    // MOVE Y AXIS - proportional speed
    // =====================================
    if (!y_arrived)
    {
        float y_error_cm = steps_to_cm(target_y_steps) - y_pos;

        if (abs(y_error_cm) > Y_DEADZONE_CM)
        {
            float y_speed = constrain(abs(y_error_cm) * Y_GAIN, 0.0, Y_MAX_SPEED);
            y_step_interval_us = speed_to_interval(y_speed);

            if (now - last_y_step_time >= y_step_interval_us)
            {
                last_y_step_time = now;

                if (y_error_cm > 0)
                    step_y(true);    // check if Y needs flipping on your machine
                else
                    step_y(false);
            }
        }
        else
        {
            y_arrived = true;
            Serial.println("Y arrived");
        }
    }


    // =====================================
    // PRINT POSITION
    // =====================================
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        Serial.print("X = ");
        Serial.print(x_pos);
        Serial.print(" cm   Target X = ");
        Serial.print(target_x);
        Serial.print("   Y = ");
        Serial.print(y_pos);
        Serial.print(" cm   Target Y = ");
        Serial.println(target_y);
    }
}
