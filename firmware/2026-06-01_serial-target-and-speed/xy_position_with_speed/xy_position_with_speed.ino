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
// TARGET POSITION (cm)
//
float target_x = 10.0;
float target_y = 5.0;


//
// SPEEDS (cm/s)
//
float x_speed_cm_s = 5.0;
float y_speed_cm_s = 2.0;


//
// CURRENT POSITION
//
float x_pos = 0.0;
float y_pos = 0.0;


//
// STEP COUNTERS
//
long x_steps = 0;
long y_steps = 0;


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
    float cm_per_step =
        (2.0 * PI * r * 100.0) / STEPS_PER_REV;

    float steps_per_second =
        speed_cm_s / cm_per_step;

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
    delayMicroseconds(2);      // pulse width

    digitalWrite(STEP_PIN_X, LOW);

    if(dir)
        x_steps++;
    else
        x_steps--;
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

    delayMicroseconds(2);      // pulse width

    digitalWrite(STEP_PIN_Y1, LOW);
    digitalWrite(STEP_PIN_Y2, LOW);

    if(dir)
        y_steps++;
    else
        y_steps--;
}



// =========================================
// SETUP
// =========================================
void setup()
{
    Serial.begin(115200);

    pinMode(STEP_PIN_X, OUTPUT);
    pinMode(DIR_PIN_X, OUTPUT);

    pinMode(STEP_PIN_Y1, OUTPUT);
    pinMode(DIR_PIN_Y1, OUTPUT);

    pinMode(STEP_PIN_Y2, OUTPUT);
    pinMode(DIR_PIN_Y2, OUTPUT);

    digitalWrite(STEP_PIN_X, LOW);

    digitalWrite(STEP_PIN_Y1, LOW);
    digitalWrite(STEP_PIN_Y2, LOW);

    //
    // Compute intervals from desired speeds
    //
    x_step_interval_us = speed_to_interval(x_speed_cm_s);
    y_step_interval_us = speed_to_interval(y_speed_cm_s);

    Serial.println("System Ready");
}



// =========================================
// MAIN LOOP
// =========================================
void loop()
{
    unsigned long now = micros();

    long target_x_steps = cm_to_steps(target_x);
    long target_y_steps = cm_to_steps(target_y);



    // =====================================
    // MOVE X AXIS
    // =====================================
    if(now - last_x_step_time >= x_step_interval_us)
    {
        last_x_step_time = now;

        if(x_steps < target_x_steps)
        {
            step_x(true);
        }
        else if(x_steps > target_x_steps)
        {
            step_x(false);
        }
    }



    // =====================================
    // MOVE Y AXIS
    // =====================================
    if(now - last_y_step_time >= y_step_interval_us)
    {
        last_y_step_time = now;

        if(y_steps < target_y_steps)
        {
            step_y(true);
        }
        else if(y_steps > target_y_steps)
        {
            step_y(false);
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
