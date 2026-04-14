const int STEP_PIN = 24;
const int DIR_PIN  = 35;

#define r 0.03
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)

float v = 0; // velocity in cm/s

// Convert linear velocity → step frequency
double linear2step_freq(float v_m_s) {
  double w = v_m_s / r;
  return w * (STEPS_PER_REV / (2.0 * PI));
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("Send velocity in cm/s (e.g. 10, -5, 0)");
}

void loop() {

  // Read velocity from Serial
  if (Serial.available()) {
    v = Serial.parseFloat(); // cm/s
    while (Serial.available()) Serial.read();

    Serial.print("Velocity: ");
    Serial.print(v);
    Serial.println(" cm/s");

    // Set direction
    if (v > 0) {
      digitalWrite(DIR_PIN, HIGH);
    } else if (v < 0) {
      digitalWrite(DIR_PIN, LOW);
    }
  }

  // Stop if velocity is ~0
  if (abs(v) < 0.01) return;

  float v_m_s = abs(v) / 100.0;
  double step_freq = linear2step_freq(v_m_s);

  if (step_freq < 1) return;

  double dt = 1000000.0 / (2.0 * step_freq);

  // Safety limit (prevents too fast pulses)
  if (dt < 100) dt = 100;

  // Generate step pulse
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(dt);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(dt);
}
