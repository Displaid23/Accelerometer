#include <HardwareSerial.h>
#include "uartSerial.h"
#include <SparkFun_Ublox_Arduino_Library.h>


SFE_UBLOX_GPS myGPS;
UBX_ACK ack;
volatile size_t received_bytes = 0;

//U0UXD is used to program ESP32
//U2UXD is used to communicate with GPS M9N module

//UART2
#define RXD2 16 // PIN 25, GPIO 16
#define TXD2 17 // PIN 27, GPIO 17

#define BR_GPS 38400
#define BR_SERIAL 38400


void setup() {

  delay(1000);
  Serial.begin(BR_SERIAL); // Initialize the serial monitor
  Serial2.begin(BR_GPS);
  Serial2.setRxFIFOFull(2);
  Serial2.onReceive(onReceiveFunction, false);

  delay(1000); // Delay to let GPS module initialize
  
  if(Serial2.available())
    Serial.println("--GPS serial available--");

  delay(5);

  writeConfiguration(UBX_VALSET_INOUTPROT, sizeof(UBX_VALSET_INOUTPROT));

  delay(5);
  Serial.println();
  writeConfiguration(UBX_VALSET_TP_FREQ100_DUTY5, sizeof(UBX_VALSET_TP_FREQ100_DUTY5));

}

void loop() {

  if(Serial2.available()){
    Serial.print(Serial2.read(), HEX);
    Serial.print(" ");
  }
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
    //Serial.print(Serial2.read(), HEX); //LOOPBACK SITUATION
    //Serial.print(" ");
    delay(5);
  }
}



