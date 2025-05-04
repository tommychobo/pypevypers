#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>


#define TARGET_PSI          15
#define SAMPLE_RATE_IMU     100
#define SAMPLE_RATE_PRESS   30

#define SOL1      A0 // assumed atmospheric solenoid
#define SOL2      A1 // assumed pressurized solenoid
#define PRESS_T   A14
#define PSI_MAX   150

#define SEND_PRESSD   0x40
#define SEND_IMU      0x50
#define SEND_RESET    0x60


#define BUFFER_SIZE   14
#define CS_PIN        53

const uint32_t toleranceMicro = 1000000;

volatile uint8_t buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile bool press_d_request = false;
volatile bool imu_request = false;
volatile bool spi_first_byte = false;

volatile int16_t accelRawXYZ[3];
volatile int16_t gyroRawXYZ[3];
volatile uint16_t pressRawD;
volatile uint16_t pressRawT;
volatile int32_t microPsi_D;
volatile int32_t microPsi_T;

/*
 * 50 - yellow - MISO
 * 51 - green - MOSI
 * 52 - blue - CLK
 * A0, A1 - solenoids
 * A14 - press_t_pin
 * 53 - white - CS
 */


void setupPulse(int num, int freq){
  uint32_t top;
  const uint32_t F_CPU_HZ = 16000000UL;
  const uint16_t prescaler = 64;
  if(freq <= 0){
    return;
  }
  top = (F_CPU_HZ/(prescaler*freq)) - 1;
  switch(num){
    case 1:
    //ISR: TIMER1_COMPA_vect
      TCCR1A = 0;
      TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);
      OCR1A = top;
      TIMSK1 = (1<<OCIE1A);
      break;
    case 3:
    //ISR: TIMER3_COMPA_vect
      TCCR3A = 0;
      TCCR3B = (1<<WGM32)|(1<<CS31)|(1<<CS30);
      OCR3A = top;
      TIMSK3 = (1<<OCIE3A);
      break;
    case 4:
    //ISR: TIMER4_COMPA_vect
      TCCR4A = 0;
      TCCR4B = (1<<WGM42)|(1<<CS41)|(1<<CS40);
      OCR4A = top;
      TIMSK4 = (1<<OCIE4A);
      break;
    case 5:
    //ISR: TIMER5_COMPA_vect
      TCCR5A = 0;
      TCCR5B = (1<<WGM52)|(1<<CS51)|(1<<CS50);
      OCR5A = top;
      TIMSK5 = (1<<OCIE5A);
      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(SOL1, OUTPUT);
  pinMode(SOL2, OUTPUT);
  pinMode(PRESS_T, INPUT);
  digitalWrite(SOL1, LOW);
  digitalWrite(SOL2, LOW);

  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  cli(); //disable interrupts
  
  setupPulse(1, SAMPLE_RATE_IMU); // sample IMU 100 Hz
  setupPulse(3, SAMPLE_RATE_PRESS); // sample pressure sensors 30 Hz
  sei(); //enable interrupts
}

void reset_board(){
  digitalWrite(SOL1, LOW);
  digitalWrite(SOL2, LOW);
  wdt_enable(WDTO_15MS); //watchdog timer for 15 ms
  while(1); //busy wait for the reset
}

ISR(SPI_STC_vect){
  if(digitalRead(CS_PIN) == LOW && (press_d_request || imu_request)){
    uint8_t data = SPDR;
    if(bufferIndex != 0){
      buffer[bufferIndex-1] = data;
    }
  }
}

ISR(TIMER1_COMPA_vect){
  if(!press_d_request && !imu_request){
    spi_first_byte = true;
    press_d_request = true;
    imu_request = false;
  }
}

ISR(TIMER3_COMPA_vect){
  if(!press_d_request && !imu_request){
    spi_first_byte = true;
    imu_request = true;
    press_d_request = false;
  }
}

void grab_press_data(){
  pressRawD = (((uint16_t)buffer[1])<<8) + (uint16_t)buffer[0];
  pressRawT = (uint16_t)analogRead(PRESS_T);
  microPsi_D = (int32_t)((((int32_t)pressRawD)*5000000/1023 - 500000)*PSI_MAX/4);
  microPsi_T = (int32_t)((((int32_t)pressRawT)*5000000/1023 - 500000)*PSI_MAX/4);
}

void transmitPressure(){
  Serial.print("DP:");
  Serial.println(microPsi_D);
  Serial.print("TP:");
  Serial.println(microPsi_T);
}

void grab_IMU_data(){
  //Accel data is in cm/s^2, gyro data is in 16ths of degrees per second
  for(uint8_t i = 0; i < 3; i++){
    accelRawXYZ[i] = (((uint16_t)buffer[(2*i)+1])<<8) + (uint16_t)buffer[2*i];
  }
  for(uint8_t i = 0; i < 3; i++){
    gyroRawXYZ[i] = (((uint16_t)buffer[(2*i)+7])<<8) + (uint16_t)buffer[(2*i)+6];
  }
  
}

void transmitIMU(){
  Serial.print("AX:");
  Serial.println(accelRawXYZ[0]);
  Serial.print("AY:");
  Serial.println(accelRawXYZ[1]);
  Serial.print("AZ:");
  Serial.println(accelRawXYZ[2]);
  Serial.print("GX:");
  Serial.println(gyroRawXYZ[0]);
  Serial.print("GY:");
  Serial.println(gyroRawXYZ[1]);
  Serial.print("GZ:");
  Serial.println(gyroRawXYZ[2]);
}

void manageSPI(){
  if(spi_first_byte){  
    bufferIndex = 0;
    spi_first_byte = false;
  }
  if(press_d_request){
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((uint8_t)(SEND_PRESSD + (bufferIndex&0x0f)));
    digitalWrite(CS_PIN, HIGH);
    bufferIndex++;
    if(bufferIndex >= 3){
      grab_press_data();
      transmitPressure();
      press_d_request = false;
    }
  }
  if(imu_request){
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((uint8_t)(SEND_IMU + (bufferIndex&0x0f)));
    digitalWrite(CS_PIN, HIGH);
    bufferIndex++;
    if(bufferIndex >= 13){
      grab_IMU_data();
      transmitIMU();
      imu_request = false;
    }
  }
}

void serialControls(){
  if(Serial.available()){
    char command = Serial.read();
    if(command == 'R'){
      digitalWrite(CS_PIN, LOW);
      SPI.transfer(SEND_RESET);
      delay(10);
      reset_board();
    }
  }
}

void updateSolenoids(uint8_t psi_target){
  uint32_t target_micro = psi_target*1000000;
  bool closeAtmospheric = false;
  bool closePressurized = false;
  if(target_micro < microPsi_D){
    closePressurized = true;
  }
  if(target_micro+toleranceMicro > microPsi_D){
    closeAtmospheric = true;
  }
  digitalWrite(SOL1, closeAtmospheric ? HIGH : LOW);
  digitalWrite(SOL2, closePressurized ? HIGH : LOW);
}

void loop() {
  manageSPI();
  
  serialControls();

  updateSolenoids(TARGET_PSI);
}
