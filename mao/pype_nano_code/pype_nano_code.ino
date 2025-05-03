#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <SPI.h>

#define BNO055_ADDRESS 0x28

#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_SYS_TRIGGER_ADDR   0x3F
#define BNO055_UNIT_SEL_ADDR      0x3B

#define BNO055_ACC_DATA_X_LSB     0x08
#define BNO055_GYR_DATA_X_LSB     0x14

#define OPERATION_MODE_IMU        0x08
#define OPERATION_MODE_CONFIG     0x00

volatile bool sampleReady = false;

// Varaibles for pressure reading and which byte is being sent
volatile int pressure_reading = 0;
volatile uint8_t  which_byte = 0;

void setupTimer1For100Hz() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 2499; // 16MHz / 64 / 100Hz = 2500 - 1
  TCCR1B |= (1 << WGM12); // CTC
  TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
  TIMSK1 |= (1 << OCIE1A); // enable compare match A interrupt
  sei();
}

ISR(TIMER1_COMPA_vect) {
  sampleReady = true;
}

void write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 1);
  return Wire.read();
}

void readLen(uint8_t reg, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

void bno055_init() {
  // Make sure we're communicating
  uint8_t chipID = read8(BNO055_CHIP_ID_ADDR);
  if (chipID != 0xA0) {
    Serial.println("Error: BNO055 not detected.");
    while (1);
  }

  // Set to config mode
  write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  delay(20);

  // Optional: Reset the sensor
  // write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  // delay(650); // wait for reset

  // Set to IMU mode
  write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMU);
  delay(20);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  bno055_init();
  setupTimer1For100Hz();
  Serial.println("BNO055 Raw IMU Init Done.");
  
  // COnfigure for SPI
  pinMode(SS, INPUT_PULLUP);
  pinMode(MISO, OUTPUT);
  SPCR = _BV(SPE) | _BV(SPIE);
  sei();

}
// This ISR has dummy code in place for IMU data, IMU data is not currently set up to be sent
ISR(SPI_STC_vect) {
  if (which_byte == 0) {
    // Listen for 0 or 1 from MEGA for pressure or IMU data
    uint8_t cmd = SPDR;

    // grab sensor data (Currently only configured for pressure sensor)
    if (cmd == 0) {
      pressure_reading = analogRead(19);
    // This else if statement would send IMU data
    } else if (cmd == 1) {
      pressure_reading = analogRead(19);
    } else {
      pressure_reading = 0;
    }

    // Send the most significant byte
    SPDR = uint8_t(pressure_reading >> 8);
    which_byte = 1;
  }
  else {
    // send the least significant byte
    SPDR = uint8_t(pressure_reading & 0xFF);
    which_byte = 0;
  }
}

void loop() {
  if (sampleReady) {
    sampleReady = false;

    uint8_t buffer[12];

    // Read 6 bytes of accel + 6 bytes of gyro
    readLen(BNO055_ACC_DATA_X_LSB, buffer, 6);
    readLen(BNO055_GYR_DATA_X_LSB, buffer + 6, 6);

    int16_t ax = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t ay = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t az = (int16_t)(buffer[5] << 8 | buffer[4]);

    int16_t gx = (int16_t)(buffer[7] << 8 | buffer[6]);
    int16_t gy = (int16_t)(buffer[9] << 8 | buffer[8]);
    int16_t gz = (int16_t)(buffer[11] << 8 | buffer[10]);

    // // Print raw data (you can convert to m/s^2 or dps if needed)
    // Serial.print("Accel: ");
    // Serial.print(ax); Serial.print(", ");
    // Serial.print(ay); Serial.print(", ");
    // Serial.print(az);

    // Serial.print(" | Gyro: ");
    // Serial.print(gx); Serial.print(", ");
    // Serial.print(gy); Serial.print(", ");
    // Serial.println(gz);
  }
}
