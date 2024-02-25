#include <HardwareSerial.h>
#include "uartSerial.h"
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Adafruit_ADXL345_U.h>



//UART2
#define RXD2 16 // PIN 25, GPIO 16
#define TXD2 17 // PIN 27, GPIO 17

//I2C
#define SDA 21
#define SCL 22

//EXTERNAL INTERRUPT
#define EXTINT 15


//BOUD-RATES
#define BR_GPS 38400
#define BR_SERIAL 38400


struct accelerometer_t {
  float x;
  float y;
  float z;
};

bool flg = false;

volatile size_t received_bytes = 0;
Adafruit_ADXL345_Unified acc = Adafruit_ADXL345_Unified(12345);


void IRAM_ATTR INTER();
void onReceiveFunction(void);
void writeConfiguration(const unsigned char config[], long len);
void setupGPS(void);
void setupAccelerometer(void);

void setup() {

  delay(1000);
  Serial.begin(BR_SERIAL); // Initialize the serial monitor
  Serial2.begin(BR_GPS);
  Serial2.setRxFIFOFull(2);
  Serial2.onReceive(onReceiveFunction, false);

  delay(1000); // Delay to let GPS module initialize
  
  if(Serial2.available()){
    Serial.println("--GPS serial available--");
    setupGPS();
  }

  attachInterrupt(EXTINT, INTER, RISING);

  setupAccelerometer();

}

void loop() {

  if(Serial2.available()){
    Serial.print(Serial2.read(), HEX);
    Serial.print(" ");
  }

  if(flg){
    sensors_event_t event; 

    acc.getEvent(&event);
    accelerometer_t a;

    a.x = event.acceleration.x;
    a.y = event.acceleration.y;
    a.z = event.acceleration.z;

    Serial.print(a.x); Serial.print(" ");
    Serial.print(a.y); Serial.print(" ");
    Serial.print(a.z); Serial.println();

    flg = false;
  }

}


void IRAM_ATTR INTER(){
  flg = true;
}


void onReceiveFunction(void) {
  // This is a callback function that will be activated on UART RX events
  size_t available = Serial2.available();
  received_bytes = received_bytes + available;
  Serial.printf("onReceive Callback:: There are %d bytes available: ", available);
  while (available --) {
    Serial.print((char)Serial2.read(), HEX);
  }
  Serial.println();
}


void writeConfiguration(const unsigned char config[], long len){

  for(uint16_t i=0; i <= len; i++){
    Serial2.write( pgm_read_byte(config + i) );
    delay(5);
  }
}

void setupGPS(void){

  writeConfiguration(UBX_VALSET_INOUTPROT, sizeof(UBX_VALSET_INOUTPROT));
  writeConfiguration(UBX_VALSET_TP_FREQ10_DUTY5, sizeof(UBX_VALSET_TP_FREQ100_DUTY5));

}

void setupAccelerometer(void){

  if(!acc.begin()){
    Serial.println("Accelerometer failed to begin...");
    setupAccelerometer();
  }
  Serial.print("Accelerometer begin");

  acc.setDataRate(ADXL345_DATARATE_400_HZ);
  acc.setRange(ADXL345_RANGE_8_G);

}



