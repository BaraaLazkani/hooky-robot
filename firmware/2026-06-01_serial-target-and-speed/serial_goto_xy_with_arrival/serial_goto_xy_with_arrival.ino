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
// CURRENT POSITION
//
const float START_X_CM = 40.7;
const float START_Y_CM = 8.8;

float x_pos = START_X_CM;
float y_pos = START_Y_CM;


//
// STEP COUNTERS
//
long x_steps = 0;
long y_steps = 0;


//
// TARGET STEPS (globals, only updated on new serial command)
//
long target_x_steps = 0;
long target_y_steps = 0;


//
// TARGET POSITION (cm)
//
float target_x = START_X_CM;
float target_y = START_Y_CM;


//
// ARRIVED FLAGS
//
bool x_arrived = false;
bool y_arrived = false;


//
// SPEEDS (cm/s)
//
float x_speed_cm_s = 5.0;
float y_speed_cm_s = 5.0;


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
// Convert speed (cm/s) -> step interval (µs)
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

    if(dir)
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

    if(dir)
        y_steps++;
    else
        y_steps--;
}



// =========================================
// READ SERIAL
// =========================================
void readSerial()
{
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        int comma = line.indexOf(',');

        if (comma > 0)
        {
            target_x = line.substring(0, comma).toFloat();
            target_y = line.substring(comma + 1).toFloat();

            // Computed ONCE per command, not every loop
            target_x_steps = cm_to_steps(target_x);
            target_y_steps = cm_to_steps(target_y);

            // Reset arrived flags for new command
            x_arrived = false;
            y_arrived = false;

            Serial.print("New Target: ");
            Serial.print(target_x);
            Serial.print(", ");
            Serial.println(target_y);
        }
    }
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

    // Compute step intervals from desired speeds
    x_step_interval_us = speed_to_interval(x_speed_cm_s);
    y_step_interval_us = speed_to_interval(y_speed_cm_s);

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


    // =====================================
    // MOVE X AXIS
    // =====================================
    if(!x_arrived && now - last_x_step_time >= x_step_interval_us)
    {
        last_x_step_time = now;

        long x_error = target_x_steps - x_steps;

        if(abs(x_error) >5)
        {
            if(x_error > 0)
                step_x(false);   // flipped
            else
                step_x(true);    // flipped
        }
        else
        {
            x_arrived = true;
            Serial.println("X arrived################################################################################");
        }
    }


    // =====================================
    // MOVE Y AXIS
    // =====================================
    if(!y_arrived && now - last_y_step_time >= y_step_interval_us)
    {
        last_y_step_time = now;

        long y_error = target_y_steps - y_steps;

        if(abs(y_error) > 5)
        {
            if(y_error > 0)
                step_y(true);    // check if Y needs flipping on your machine
            else
                step_y(false);
        }
        else
        {
            y_arrived = true;
            Serial.println("Y arrived#######################################################################################################################");
        }
    }


    // =====================================
    // UPDATE POSITION
    // =====================================
    x_pos = steps_to_cm(x_steps);
    y_pos = steps_to_cm(y_steps);


    // =====================================
    // PRINT POSITION
    // =====================================
    static unsigned long lastPrint = 0;

    if(millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        Serial.print("X = ");
        Serial.print(x_pos);
        Serial.print(" cm   Y = ");
        Serial.print(y_pos);
        Serial.println(" cm");
    }
}
