#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

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

#define BUFFER_SIZE               16

int8_t press_index = -1;
int8_t imu_index = -1;
volatile bool reset_ordered = false;
volatile bool refresh_data = false;
uint8_t buffer[BUFFER_SIZE];

void setupSpiPeripheral() {
  cli();
  DDRB &= ~((1 << PB3) | (1 << PB5) | (1 << PB2)); // MOSI, SCK, SS as input
  DDRB |= (1 << PB4); // MISO as output
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  sei();
}

void setupTimer1(int freq){
  if(freq <= 0){
    return;
  }
  TCCR1A = 0;
  TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);
  OCR1A = 16000000UL/(64*freq) - 1;
  TIMSK1 = (1<<OCIE1A);
}

void reset_board(){
  wdt_enable(WDTO_15MS);
  while(1);
}

void init_buffer(){
  for(uint8_t i = 0; i < BUFFER_SIZE; i++){
    buffer[i] = 0;
  }
}

ISR(SPI_STC_vect){
  if(digitalRead(CS_PIN) == LOW){
    uint8_t data = SPDR;
    if((data&0xf0) == 0x40){ // pressure sensor
      SPDR = buffer[(data&0x0f)+12];
    }
    else if((data&0xf0) == 0x50){ // IMU
      SPDR = buffer[data&0x0f];
    }
    else if(data == 0x60){ // reset
      reset_ordered = true;
    }
  }
}

ISR(TIMER1_COMPA_vect){
  refresh_data = true;
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
  Wire.begin();
  setupSpiPeripheral();
  pinMode(CS_PIN, INPUT_PULLUP);
  bno055_init();
  init_buffer();
  pinMode(PRESS_D_PIN, INPUT);
  cli();
  setupTimer1(100);
  sei();
}

void loop() {
  if(reset_ordered){
    reset_board();
  }

  if(refresh_data){
    refresh_data = false;
    cli();
    // Read 6 bytes of accel + 6 bytes of gyro
    readLen(BNO055_ACC_DATA_X_LSB, buffer, 6);
    readLen(BNO055_GYR_DATA_X_LSB, buffer + 6, 6);
    uint16_t press_data = (uint16_t)analogRead(PRESS_D_PIN);
    buffer[12] = (uint8_t)press_data&0xff;
    buffer[13] = (uint8_t)((press_data&0xff00) >> 8);
    sei();
    /* SPI transmit the buffer on Arduino Nano
      *  MISO: D12
      *  MOSI: D11
      *  /CS: D10
      *  SCK: D13
      */
  }
}
