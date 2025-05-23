#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>



#define SOL1      A0 // assumed atmospheric solenoid
#define SOL2      A1 // assumed pressurized solenoid
#define PRESS_T   A14
#define PRESS_2   A13
#define PSI_MAX   150

#define SEND_PRESSD   0x40
#define SEND_IMU      0x50
#define SEND_RESET    0x60
//REMEMBER: we are using 0x10, 0x20, 0x30 for sending frequency updates to the nano


#define BUFFER_SIZE   14
#define SERIAL_BUFFER_SIZE 32
#define CS_PIN        53

const uint32_t toleranceMicro = 1000000;


// values controllable by the console
int target_psi = 15;
int sample_rate_imu = 100;
int sample_rate_press = 30;
int sample_rate_solenoid = 10;


volatile uint8_t buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile bool press_d_request = false;
volatile bool imu_request = false;
volatile bool spi_first_byte = false;
volatile bool sol_update_ready = false;

volatile int16_t accelRawXYZ[3];
volatile int16_t gyroRawXYZ[3];
volatile uint16_t pressRawD;
volatile uint16_t pressRaw2;
volatile uint16_t pressRawT;
volatile int32_t microPsi_D;
volatile int32_t microPsi_T;
volatile int32_t microPsi_2;

bool command_incoming = false;
bool command_ready = false;
bool test_running = false;
bool solenoid_control = false;
int command_index = 0;
char command[SERIAL_BUFFER_SIZE];


/*
 * 50 - yellow - MISO
 * 51 - green - MOSI
 * 52 - blue - CLK
 * A0, A1 - solenoids
 * A14 - press_t_pin
 * 53 - white - CS
 */

// wrap all MEGA timer setups in this function
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
  
  setupPulse(1, sample_rate_imu); // sample IMU 100 Hz
  setupPulse(3, sample_rate_press); // sample pressure sensors 30 Hz
  sei(); //enable interrupts
}

void reset_board(){
  digitalWrite(SOL1, LOW);
  digitalWrite(SOL2, LOW);
  wdt_enable(WDTO_15MS); //watchdog timer for 15 ms
  while(1); //busy wait for the reset
}

//The mega never uses the SPI interrupt, but I left this here in case we need it
/*ISR(SPI_STC_vect){
  if(press_d_request || imu_request){
    uint8_t data = SPDR;
    if(bufferIndex != 0){
      buffer[bufferIndex-1] = data;
    }
  }
}*/

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

ISR(TIMER5_COMPA_vect){
  if(!sol_update_ready){
    sol_update_ready = true;
  }
}

void grab_press_data(){
  pressRawT = (uint16_t)analogRead(PRESS_T);
  pressRaw2 = (uint16_t)analogRead(PRESS_2);
  microPsi_D = (int32_t)((((int32_t)pressRawD)*5000000/1023 - 500000)*PSI_MAX/4);
  microPsi_T = (int32_t)((((int32_t)pressRawT)*5000000/1023 - 500000)*PSI_MAX/4);
  microPsi_2 = (int32_t)((((int32_t)pressRaw2)*5000000/1023 - 500000)*PSI_MAX/4);
}

void transmitPressure(){
  Serial.print("~DP:");
  Serial.println(microPsi_D);
  Serial.print("~TP:");
  Serial.println(microPsi_T);
  Serial.print("~2P:");
  Serial.println(microPsi_2);
}

void grab_IMU_data(){
  //Accel data is in cm/s^2, gyro data is in 16ths of degrees per second
  for(uint8_t i = 0; i < 3; i++){
    accelRawXYZ[i] = (((uint16_t)buffer[(2*i)])<<8) + (uint16_t)buffer[(2*i)+1];
  }
  for(uint8_t i = 0; i < 3; i++){
    gyroRawXYZ[i] = (((uint16_t)buffer[(2*i)+6])<<8) + (uint16_t)buffer[(2*i)+7];
  }
  
}

void transmitIMU(){
  Serial.print("~AX:");
  Serial.println(accelRawXYZ[0]);
  Serial.print("~AY:");
  Serial.println(accelRawXYZ[1]);
  Serial.print("~AZ:");
  Serial.println(accelRawXYZ[2]);
  Serial.print("~GX:");
  Serial.println(gyroRawXYZ[0]);
  Serial.print("~GY:");
  Serial.println(gyroRawXYZ[1]);
  Serial.print("~GZ:");
  Serial.println(gyroRawXYZ[2]);
}

// communicate with the NANO over SPI
void manageSPI(){
  uint8_t data;
  if(spi_first_byte){  
    bufferIndex = 0;
    spi_first_byte = false;
  }
  if(press_d_request){
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((uint8_t)(SEND_PRESSD + 0));
    uint16_t data = SPDR;//flush
    SPI.transfer((uint8_t)(SEND_PRESSD + 1));
    data |= (uint16_t)(SPDR&0xff)<<8;
    SPI.transfer((uint8_t)(SEND_PRESSD + 1));
    data = (uint16_t)SPDR&0xff;
    digitalWrite(CS_PIN, HIGH);
    pressRawD = data;
    grab_press_data();
    transmitPressure();
    press_d_request = false;
  }
  if(imu_request){
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((uint8_t)(SEND_IMU + (bufferIndex&0x0f)));
    data = (uint8_t)SPDR;
    digitalWrite(CS_PIN, HIGH);
    if(bufferIndex != 0){
      buffer[bufferIndex-1] = data;
    }
    if(bufferIndex >= 13){
      grab_IMU_data();
      transmitIMU();
      imu_request = false;
    }
    bufferIndex++;
  }
}

void update_nano_freq(int freq){
  digitalWrite(CS_PIN, LOW);
  SPI.transfer((uint8_t)(0x10 + (freq&0x00f)));
  uint8_t data = (uint8_t)SPDR;
  SPI.transfer((uint8_t)(0x20 + ((freq&0x0f0)>>4)));
  data = (uint8_t)SPDR;
  SPI.transfer((uint8_t)(0x30 + ((freq&0xf00)>>8)));
  data |= (uint8_t)SPDR;
  
  if(data != freq&0xff){
    Serial.println("NANO:frequency update failed");
    delay(100);
  }else{
    Serial.print("NANO:frequency updated to ");
    Serial.println(freq);
  }
  digitalWrite(CS_PIN, HIGH);
}

// big important control function for the mega
void serialControls(){
  if(Serial.available()){
    size_t len = Serial.readBytesUntil('\n', command, SERIAL_BUFFER_SIZE-1);
    if(command[0] != '~'){
      return;
    }
    //char com = Serial.read();
    int value = atoi((char*)(command+2));
    //Serial.print("MEGA:Received:");
    //Serial.println(command);
    switch(command[1]){
      case 'P': //change the frequency of the pressure sensor
        if(value > 0 && value < 1000){
          setupPulse(3, value);
          update_nano_freq(value);
          Serial.print("MEGA:Updated pressure sample rate to ");
          Serial.println(value);
        }
        else{
          Serial.println("MEGA:Invalid pressure sample rate");
          delay(100);
        }
        break;
      case 'E': //start/stop the test
        if(test_running){
          test_running = false;
          Serial.println("MEGA:Stopping test...");
        }
        else{
          test_running = true;
          Serial.println("MEGA:Starting test...");
        }
        break;
      case 'N': //change the target PSI
        if(value > 0 && value < PSI_MAX){
          target_psi = value;
          Serial.print("MEGA:Target PSI set to ");
          Serial.println(target_psi);
        }
        else{
          Serial.println("MEGA:Invalid target PSI");
          delay(500);
        }
        break;
      case 'I': //change the frequency of the IMU
        if(value > 0 && value < 1000){
          setupPulse(1, value);
          update_nano_freq(value);
          Serial.print("MEGA:IMU sample rate set to ");
          Serial.println(value);
        }
        else{
          Serial.println("MEGA:Invalid IMU sample rate");
          delay(500);
        }
        break;
      case 'R': //reset the board
        Serial.println("MEGA:Resetting board...");
        Serial.println("NANO:Resetting board...");
        digitalWrite(CS_PIN, LOW);
        SPI.transfer(SEND_RESET);
        delay(10);
        reset_board();
        break;
      case 'F': // just used in the console. NOP
        break;
      case 'S': //solenoid control
        if(solenoid_control){
          solenoid_control = false;
          digitalWrite(SOL1, LOW);
          digitalWrite(SOL2, LOW);
          Serial.println("MEGA:Stopping solenoid control...");
        }
        else{
          if(value <= 0 || value > 100){
            Serial.println("MEGA:Invalid solenoid duty cycle: specify frequency in Hz");
            delay(100);
          }else{
            solenoid_control = true;
            Serial.println("MEGA:Starting solenoid control...");
            setupPulse(5, value);
          }
        }
        break;
      default:
        Serial.println("MEGA:Invalid command");
        break;
      
    }
  }
  
}

void updateSolenoids(){
  uint32_t target_micro = target_psi*1000000;
  bool closeAtmospheric = false;
  bool closePressurized = false;
  if(target_micro < microPsi_T){
    closePressurized = true;
  }
  if(target_micro+toleranceMicro > microPsi_T){
    closeAtmospheric = true;
  }
  digitalWrite(SOL1, closeAtmospheric ? HIGH : LOW);
  digitalWrite(SOL2, closePressurized ? HIGH : LOW);
  sol_update_ready = false;
}

void loop() {
  if(test_running){
    manageSPI();
    // solenoid_control updated by serialControls(), 
    //sol_update_ready is set by the timer interrupt
    if(solenoid_control && sol_update_ready){
      updateSolenoids();
    }
  }
  serialControls();
}
