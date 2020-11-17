#include <Arduino.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 4 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  while (Serial.available())
    mpu.dmpInitialize();

  mpu.CalibrateAccel(6);

  mpu.setDMPEnabled(true);

  mpu.setDLPFMode(2);
  mpu.setDHPFMode(1);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{

  Serial.println(abs(mpu.getAccelerationX()) + abs(mpu.getAccelerationY()) + abs(mpu.getAccelerationZ()) + 1000);
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}