const int STEP_PIN_2 = 23;
const int DIR_PIN_2  = 18;

const int STEP_PIN_3 = 22;
const int DIR_PIN_3  = 19;

#define r 0.03
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)

float v = 0;


// Convert velocity -> step frequency
double linear2step_freq(float v_m_s)
{
    double w = v_m_s / r;

    return w * (STEPS_PER_REV / (2.0 * PI));
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

    Serial.println("Send velocity:");
    Serial.println("10  -> forward");
    Serial.println("-10 -> reverse");
    Serial.println("0   -> stop");
}


void loop()
{
    // Read velocity from serial
    if (Serial.available())
    {
        v = Serial.parseFloat();

        while (Serial.available())
            Serial.read();

        Serial.print("Velocity = ");
        Serial.println(v);

        // SET DIRECTION
        if (v >= 0)
        {
            digitalWrite(DIR_PIN_2, HIGH);
            digitalWrite(DIR_PIN_3, HIGH);
        }
        else
        {
            digitalWrite(DIR_PIN_2, LOW);
            digitalWrite(DIR_PIN_3, LOW);
        }

        // IMPORTANT:
        // give driver time to change direction
        delayMicroseconds(100);
    }

    // STOP
    if (abs(v) < 0.01)
        return;

    // Convert cm/s -> m/s
    float v_m_s = abs(v) / 100.0;

    // Compute frequency
    double step_freq = linear2step_freq(v_m_s);

    if (step_freq < 1)
        return;

    // pulse timing
    double dt = 1000000.0 / (2.0 * step_freq);

    if (dt < 200)
        dt = 200;

    // STEP BOTH MOTORS
    digitalWrite(STEP_PIN_2, HIGH);
    digitalWrite(STEP_PIN_3, HIGH);

    delayMicroseconds(dt);

    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);

    delayMicroseconds(dt);
}
