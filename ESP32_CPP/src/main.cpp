#include <Arduino.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 4 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int acceleration = 0;
long int currentTime = 0;
long int previousReading = 0;
long int checkPeriod = 0;
int minAcc = 0;
int maxAcc = 0;
int AccThresh = 1000;

void readAcceleration()
{
  acceleration = abs(mpu.getAccelerationX()) + abs(mpu.getAccelerationY()) + abs(mpu.getAccelerationZ()) + 1500;
}

void setMinAndMax()
{
  maxAcc = acceleration > maxAcc ? acceleration : maxAcc;
  minAcc = acceleration < minAcc ? acceleration : minAcc;
}

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
  readAcceleration();
  minAcc = acceleration;
  maxAcc = acceleration;
}

void loop()
{
  currentTime = millis();
  readAcceleration();
  setMinAndMax();
  if (currentTime - previousReading >= checkPeriod)
  {
    previousReading = currentTime;
    if (maxAcc - minAcc >= AccThresh)
    {
      digitalWrite(LED_PIN, 1);
      maxAcc = acceleration;
      minAcc = acceleration;
    }
    else
    {
      digitalWrite(LED_PIN, 0);
    }
  }
  digitalWrite(LED_PIN, 0);
}