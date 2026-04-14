#define STEP_BIT 4   // pin 12 = PORTB bit 4
#define DIR_BIT  5   // pin 13 = PORTB bit 5

#define r 0.03
#define microsteps_X 8.0
#define STEPS_PER_REV_X (200.0 * microsteps_X)
#define rad2step_freq_x(w) (w * (STEPS_PER_REV_X / (2.0 * M_PI)))
#define oneStep2m_x float((2.0 * M_PI * r) / (200.0 * microsteps_X))
#define linear2step_freq_x(v) (rad2step_freq_x(v / r))

volatile long x_toggle_count = 0;
volatile long isr_count = 0;
double dt_x = 1000;
volatile double vx = 0;

void setupTimer_X() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 0xFFFF;          // safe default
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // 64 prescaler
  // OCIE1A not enabled yet — motor off at boot
  interrupts();
}

void setTimer_X(double dt) {
  noInterrupts();
  TIMSK1 &= ~(1 << OCIE1A);
  TCNT1 = 0;
  long ocr = (long)(dt / 4.0) - 1;
  if (ocr < 1)     ocr = 1;
  if (ocr > 65534) ocr = 65534;
  OCR1A = ocr;
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  Serial.print("[DEBUG] dt="); Serial.print(dt);
  Serial.print(" OCR1A="); Serial.println(ocr);
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << STEP_BIT);  // toggle pin 12 (PORTB bit 4)
  isr_count++;
  if (vx != 0) {
    x_toggle_count += (vx > 0) ? 1 : -1;
  }
}

void setup() {
  Serial.begin(115200);

  // Set pins 12 and 13 as output via PORTB
  DDRB |= (1 << STEP_BIT) | (1 << DIR_BIT);

  // Step starts HIGH (inactive for common anode), DIR low
  PORTB |=  (1 << STEP_BIT);
  PORTB &= ~(1 << DIR_BIT);

  setupTimer_X();

  Serial.println("Ready - Send velocity in cm/s");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("[HEARTBEAT] isr_count="); Serial.print(isr_count);
    Serial.print(" x_toggle="); Serial.print(x_toggle_count);
    Serial.print(" vx="); Serial.print(vx * 100);
    Serial.print(" cm/s  TIMSK1="); Serial.println(TIMSK1, BIN);
    isr_count = 0;
  }

  if (Serial.available()) {
    vx = Serial.parseFloat() / 100.0;
    while (Serial.available() && Serial.read() != '\n');

    Serial.print("CMD: vx="); Serial.print(vx * 100); Serial.println(" cm/s");

    double step_freq_x = linear2step_freq_x(fabs(vx));
    dt_x = (step_freq_x > 0.1) ? (1000000.0 / (2.0 * step_freq_x)) : 0;

    Serial.print("step_freq="); Serial.print(step_freq_x);
    Serial.print(" dt="); Serial.println(dt_x);

    if (vx > 0) {
      PORTB |=  (1 << DIR_BIT);
      Serial.println("DIR=HIGH (forward)");
    } else if (vx < 0) {
      PORTB &= ~(1 << DIR_BIT);
      Serial.println("DIR=LOW (backward)");
    }

    if (dt_x > 0) {
      setTimer_X(dt_x);
      Serial.println("Motor STARTED");
    } else {
      TIMSK1 &= ~(1 << OCIE1A);
      vx = 0;
      Serial.println("Motor STOPPED");
    }
  }
}
