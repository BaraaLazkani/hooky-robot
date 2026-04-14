const int STEP_PIN = 26;
const int DIR_PIN  = 35;

#define r 0.03
#define microsteps 8.0
#define STEPS_PER_REV (200.0 * microsteps)

float vx = 0;  // velocity (m/s)

double linear2step_freq(float v) {
  double w = v / r; // angular velocity
  return w * (STEPS_PER_REV / (2.0 * PI));
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, HIGH);

  Serial.println("Ready - Send velocity in cm/s");
}

void loop() {

  // Read velocity from Serial
  if (Serial.available()) {
    vx = Serial.parseFloat() / 100.0; // cm/s → m/s
    while (Serial.available()) Serial.read();

    Serial.print("vx = ");
    Serial.print(vx * 100);
    Serial.println(" cm/s");

    // Set direction
    if (vx > 0) {
      digitalWrite(DIR_PIN, HIGH);
    } else if (vx < 0) {
      digitalWrite(DIR_PIN, LOW);
    }
  }

  // Move motor if velocity != 0
  if (vx != 0) {
    double step_freq = linear2step_freq(abs(vx));

    if (step_freq > 0.1) {
      double dt = 1000000.0 / (2.0 * step_freq); // µs

      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(dt);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(dt);
    }
  }
}
