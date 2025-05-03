#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/interrupt.h>

#define BNO055_ADDRESS 0x28

#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_SYS_TRIGGER_ADDR   0x3F
#define BNO055_UNIT_SEL_ADDR      0x3B

#define BNO055_ACC_DATA_X_LSB     0x08
#define BNO055_GYR_DATA_X_LSB     0x14

#define OPERATION_MODE_IMU        0x08
#define OPERATION_MODE_CONFIG     0x00

#define PRESS_D_PIN               A0
#define CS_PIN                    10

volatile bool sampleReady = false;

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
  Wire.requestFrom((uint8_t)BNO055_ADDRESS, (uint8_t)1);
  return Wire.read();
}

void readLen(uint8_t reg, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BNO055_ADDRESS, (uint8_t)len);
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
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  bno055_init();
  setupTimer1For100Hz();
  pinMode(PRESS_D_PIN, INPUT);
}

void loop() {
  if (sampleReady) {
    sampleReady = false;

    uint8_t buffer[14];
    uint16_t press_data;

    // Read 6 bytes of accel + 6 bytes of gyro
    readLen(BNO055_ACC_DATA_X_LSB, buffer, 6);
    readLen(BNO055_GYR_DATA_X_LSB, buffer + 6, 6);

    press_data = (uint16_t)analogRead(PRESS_D_PIN);

    buffer[12] = (uint8_t)press_data&0xff;
    buffer[13] = (uint8_t)(press_data&0xff00 >> 8);

    /* SPI transmit the buffer on Arduino Nano
     *  MISO: D12
     *  MOSI: D11
     *  /CS: D10
     *  SCK: D13
     */
    digitalWrite(CS_PIN, LOW);
    for(uint8_t i =0; i < 14; i++){
      SPI.transfer(buffer[i]);
    }
    digitalWrite(CS_PIN, HIGH);
  }
}
