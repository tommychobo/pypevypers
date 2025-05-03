#include <Arduino.h>
#define SOL1      A0
#define SOL2      A1
#define PRESS_T   A14
#define PSI_MAX   150

/*
#define POT_DUTY_A A2
#define POT_DUTY_B A3
#define POT_FREQ   A4

#define TIMER1_PRESCALER 1024
#define F_CPU 16000000UL

#define MIN_HZ_SETTING 1
#define MAX_HZ_SETTING 10
*/

#define BUFFER_SIZE   14
#define CS_PIN        53

volatile uint8_t buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile bool spiDataReady = false;

volatile uint16_t accelRawXYZ[3];
volatile uint16_t gyroRawXYZ[3];
volatile uint16_t pressRawD;
volatile uint16_t pressRawT;

/*
 * 50 - yellow - MISO
 * 51 - green - MOSI
 * 52 - blue - CLK
 * A0, A1 - solenoids
 * A2 - press_t_pin
 * 53 - white - CS
 */


void setup() {
  Serial.begin(115200);
  pinMode(SOL1, OUTPUT);
  pinMode(SOL2, OUTPUT);
  pinMode(PRESS_T, INPUT);
  digitalWrite(SOL1, LOW);
  digitalWrite(SOL2, LOW);

  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(CS_PIN, INPUT);

  cli(); //disable interrupts
  /*SPI peripheral mode*/
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

  uint8_t oldData = SPSR;
  oldData = SPDR;
  sei(); //enable interrupts
}

ISR(SPI_STC_vect){
  /*if(digitalRead(CS_PIN) == HIGH){
    bufferIndex = 0;
  }*/
  if(digitalRead(CS_PIN) == LOW && !spiDataReady){
    uint8_t data = SPDR;
    buffer[bufferIndex] = data;
    bufferIndex++;

    if(bufferIndex >= BUFFER_SIZE){
      bufferIndex = 0;
      spiDataReady = true;
    }
  }
}


void loop() {
  if(spiDataReady){
    spiDataReady = false;
    for(uint8_t i = 0; i < 3; i++){
      accelRawXYZ[i] = (((uint16_t)buffer[(2*i)+1])<<8) + (uint16_t)buffer[2*i];
    }
    for(uint8_t i = 0; i < 3; i++){
      gyroRawXYZ[i] = (((uint16_t)buffer[(2*i)+7])<<8) + (uint16_t)buffer[(2*i)+6];
    }
    pressRawD = (((uint16_t)buffer[13])<<8) + (uint16_t)buffer[12];
    pressRawT = (uint16_t)analogRead(PRESS_T);
    int32_t microPsi_D = (int32_t)((((int32_t)pressRawD)*5000000/1023 - 500000)*PSI_MAX/4);
    int32_t microPsi_T = (int32_t)((((int32_t)pressRawT)*5000000/1023 - 500000)*PSI_MAX/4);
    
    Serial.print("DP:");
    Serial.println(microPsi_D);
    Serial.print("TP:");
    Serial.println(microPsi_T);
  }

  
  
}
