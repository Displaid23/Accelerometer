
#include "RAK12033-IIM42652.h"
#define who_I_am 0x75
IIM42652 IMU;   // Our IMU is IIM42352

void setup()
{
  Serial.begin(921600);
  Wire.begin();
}

void loop()
{
  IIM42652_axis_t accel_data;
  float temp;

  float acc_x, acc_y, acc_z;

  IMU.ex_idle();
  IMU.accelerometer_enable();
  IMU.temperature_enable();

  delay(5);

  IMU.get_accel_data(&accel_data);
  IMU.get_temperature(&temp);

  /*
   * ±16 g  : 2048  LSB/g
   * ±8 g   : 4096  LSB/g
   * ±4 g   : 8192  LSB/g
   * ±2 g   : 16384 LSB/g
   */
  acc_x = (float)accel_data.x / 2048; // Read accelarometer data example
  acc_y = (float)accel_data.y / 2048; // Read accelarometer data example
  acc_z = (float)accel_data.z / 2048; // Read accelarometer data example

  Serial.print("Accel X: ");
  Serial.print(acc_x);
  Serial.print("[g]  Y: ");
  Serial.print(acc_y);
  Serial.print("[g]  Z: ");
  Serial.print(acc_z);
  Serial.println("[g]");

  /*
   * ±2000 º/s    : 16.4   LSB/(º/s)
   * ±1000 º/s    : 32.8   LSB/(º/s)
   * ±500  º/s    : 65.5   LSB/(º/s)
   * ±250  º/s    : 131    LSB/(º/s)
   * ±125  º/s    : 262    LSB/(º/s)
   * ±62.5  º/s   : 524.3  LSB/(º/s)
   * ±31.25  º/s  : 1048.6 LSB/(º/s)
   * ±15.625 º/s  : 2097.2 LSB/(º/s)
   */

  //Serial.print("Temper : ");
  //Serial.print(temp);
  //Serial.println("[ºC]");

  IMU.accelerometer_disable();
  IMU.temperature_disable();
  IMU.idle();
  delay(5);
  uint8_t tmp;
  IMU.readRegister(who_I_am, &tmp, 1); // Read Register Example
  //Serial.println(tmp);
}
