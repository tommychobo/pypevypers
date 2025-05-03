#define A0_PIN 54   // PWM Output A
#define A1_PIN 55   // PWM Output B

#define POT_DUTY_A A2
#define POT_DUTY_B A3
#define POT_FREQ   A4

#define TIMER1_PRESCALER 1024
#define F_CPU 16000000UL

#define MIN_HZ_SETTING 1
#define MAX_HZ_SETTING 10

// Global timing state
volatile uint16_t pwmTop = 3125; // Gets changed immediately
volatile uint16_t counter = 0;
volatile uint16_t dutyTicksA = 0;
volatile uint16_t dutyTicksB = 0;
volatile bool inFirstHalf = true;

void setup() {
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);

  pinMode(POT_DUTY_A, INPUT);
  pinMode(POT_DUTY_B, INPUT);
  pinMode(POT_FREQ, INPUT);

  // Timer1 setup
  cli(); // disable interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024

  OCR1A = pwmTop;
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
  sei(); // enable interrupts
}

ISR(TIMER1_COMPA_vect) {
  counter++;
  if (counter >= pwmTop) {
    counter = 0;
    inFirstHalf = !inFirstHalf; // flip to next phase
  }

  if (inFirstHalf) {
    digitalWrite(A0_PIN, (counter < dutyTicksA) ? HIGH : LOW);
    digitalWrite(A1_PIN, LOW);
  } else {
    digitalWrite(A1_PIN, (counter < dutyTicksB) ? HIGH : LOW);
    digitalWrite(A0_PIN, LOW);
  }
}

void loop() {
  // --- Frequency Control ---
  int freqRaw = analogRead(POT_FREQ); // 0–1023
  // Map to ~1–10 Hz (half-period)
  float freqHz = map(freqRaw, 0, 1023, MIN_HZ_SETTING, MAX_HZ_SETTING);
  float halfPeriodSec = 0.5 / freqHz;
  uint32_t ticks = (uint32_t)(halfPeriodSec * (F_CPU / TIMER1_PRESCALER));

  if (ticks > 65535) ticks = 65535;
  pwmTop = ticks;
  OCR1A = pwmTop;

  // --- Duty Cycle Controls ---
  int rawA = analogRead(POT_DUTY_A);
  int rawB = analogRead(POT_DUTY_B);

  // Cap to 50% max so they can't overlap
  dutyTicksA = map(rawA, 0, 1023, 0, pwmTop / 2);
  dutyTicksB = map(rawB, 0, 1023, 0, pwmTop / 2);

  delay(50); // slow down reads
}
