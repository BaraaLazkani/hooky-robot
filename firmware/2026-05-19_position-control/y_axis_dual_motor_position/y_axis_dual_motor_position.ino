const int STEP_PIN_2 = 23;
const int DIR_PIN_2  = 18;

const int STEP_PIN_3 = 22;
const int DIR_PIN_3  = 19;

#define r 0.0305
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)

float x = 0.0;              // current position (cm)
float target_x = 5.0;     // desired position (cm)

long current_steps = 0;


// Convert cm -> steps
long cm_to_steps(float cm)
{
    float meters = cm / 100.0;

    float circumference = 2.0 * PI * r;

    float revs = meters / circumference;

    return (long)(revs * STEPS_PER_REV);
}


// Convert steps -> cm
float steps_to_cm(long steps)
{
    float revs = (float)steps / STEPS_PER_REV;

    float meters = revs * (2.0 * PI * r);

    return meters * 100.0;
}


// Step BOTH motors together
void step_motors(bool dir)
{
    // Set direction
    digitalWrite(DIR_PIN_2, dir);
    digitalWrite(DIR_PIN_3, dir);

    // Step pulse
    digitalWrite(STEP_PIN_2, HIGH);
    digitalWrite(STEP_PIN_3, HIGH);

    delayMicroseconds(500);

    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);

    delayMicroseconds(500);

    // Update software position
    if(dir)
        current_steps++;
    else
        current_steps--;
}


void setup()
{
    Serial.begin(115200);

    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);

    pinMode(STEP_PIN_3, OUTPUT);
    pinMode(DIR_PIN_3, OUTPUT);

    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);

    digitalWrite(DIR_PIN_2, LOW);
    digitalWrite(DIR_PIN_3, LOW);
}


void loop()
{
    long target_steps = cm_to_steps(target_x);

    // Move forward
    if(current_steps < target_steps)
    {
        step_motors(true);
    }

    // Move backward
    else if(current_steps > target_steps)
    {
        step_motors(false);
    }

    // Compute current position
    x = steps_to_cm(current_steps);

    Serial.print("Position = ");
    Serial.print(x);
    Serial.println(" cm");
}
