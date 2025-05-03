#include <SPI.h>

#define A0_PIN 54   // Solenoid Output Pressure In
#define A1_PIN 55   // Solenoid Output Pressure Out
#define A14_PIN 60  // Pressure daa enclosure side
#define CS_PIN 53

#define TIMER1_PRESCALER 1024
#define F_CPU 16000000UL

// // Global timing state
// volatile uint16_t pwmTop = 3125; // Gets changed immediately
// volatile uint16_t counter = 0;
// volatile uint16_t dutyTicksA = 0;
// volatile uint16_t dutyTicksB = 0;
// volatile bool inFirstHalf = true;

void setup() {

  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  digitalWrite(A0_PIN, LOW);
  digitalWrite(A1_PIN, LOW);

  // Begin SPI as Master
  SPI.begin();

  // // Timer1 setup
  // cli(); // disable interrupts
  // TCCR1A = 0;
  // TCCR1B = 0;

  // TCCR1B |= (1 << WGM12); // CTC mode
  // TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024

  // OCR1A = pwmTop;
  // TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
  // sei(); // enable interrupts
}

// ISR(TIMER1_COMPA_vect) {
//   counter++;
//   if (counter >= pwmTop) {
//     counter = 0;
//     inFirstHalf = !inFirstHalf; // flip to next phase
//   }

//   if (inFirstHalf) {
//     digitalWrite(A0_PIN, (counter < dutyTicksA) ? HIGH : LOW);
//     digitalWrite(A1_PIN, LOW);
//   } else {
//     digitalWrite(A1_PIN, (counter < dutyTicksB) ? HIGH : LOW);
//     digitalWrite(A0_PIN, LOW);
//   }
// }

// Function to grab data from nano, again only set up for pressure data
uint16_t readPressure() {
  uint8_t 1half, 2half;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
    SPI.transfer(0);      // tell slave “pressure = 0”
    1half = SPI.transfer(0); // grab high 8 bits
    2half = SPI.transfer(0); // grab low  8 bits
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return (uint16_t(hi) << 8 | lo) & 0x03FF;
}

void loop() {
  // Receive from Nano
  // 0 to receive pressure data, 1 for IMU 
  // int data_type = 0;
  // if (data_type == 1){
  //     digitalWrite(56, HIGH);
  //     delay(1);
  // }
  // else {
  //    digitalWrite(56, LOW);
  //    int pressure_data = SPI.transfer16(0x00);
  // }
               
  uint16_t pressure_data = readPressure();
  float voltage  = pressure_data * (5.0f / 1023.0f);
  float pressure = (voltage - 0.5f) * (150.0f / 4.0f);


  // // --- Frequency Control ---
  // float freqHz 1;
  // float halfPeriodSec = 0.5 / freqHz;
  // uint32_t ticks = (uint32_t)(halfPeriodSec * (F_CPU / TIMER1_PRESCALER));

  // if (ticks > 65535) ticks = 65535;
  // pwmTop = ticks;
  // OCR1A = pwmTop;
 
  // Solenoid Control Loops
  float target_pressure = 30;
  if (data_want == 0) {
    if (pressure < (target_pressure - 0.25)) {
      digitalWrite(54, HIGH);
      digitalWrite(55, LOW);
    }
    else if ((target_pressure - 0.25) <= pressure <= (target_pressure + 0.25)) {
      digitalWrite(54, LOW);
      digitalWrite(55, LOW);
    }
    else {
      digitalWrite(54, LOW);
      digitalWrite(55, HIGH);
    }
  }

  // // --- Duty Cycle Controls ---
  // int SOL_DUTY_A = 50;
  // int SOL_DUTY_B = 50;

  // // Cap to 50% max so they can't overlap
  // dutyTicksA = map(SOL_DUTY_A, 0, 100, 0, pwmTop / 2);
  // dutyTicksB = map(SOL_DUTY_B, 0, 100, 0, pwmTop / 2);

  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("PSI");
  delay(100); // slow down reads
}
