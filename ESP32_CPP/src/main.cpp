#include <Arduino.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 4 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  while (Serial.available())

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.setDLPFMode(2);
        mpu.setDHPFMode(1);



  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  // // if programming failed, don't try to do anything
  // if (!dmpReady)
  //   return;
  // // read a packet from FIFO
  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  // { // Get the Latest packet

  //   //Serial.println(abs(mpu.getAccelerationX()) + abs(mpu.getAccelerationY()) + abs(mpu.getAccelerationZ()));
  //   // Serial.println(abs(mpu.getAccelerationX()) + abs(mpu.getAccelerationY()) + abs(mpu.getAccelerationZ()) +1000);
  //   // blink LED to indicate activity
  //   blinkState = !blinkState;
  //   digitalWrite(LED_PIN, blinkState);
  //       Serial.print(mpu.getMotionDetectionDuration());
  //       Serial.print(mpu.getXPosMotionDetected());
  //       Serial.println(mpu.getMotionDetectionThreshold());


  // }

  Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();

  if (act.isActivity)
  {
    digitalWrite(4, HIGH);
  } else
  {
    digitalWrite(4, LOW);
  }

  if (act.isInactivity)
  {
    digitalWrite(7, HIGH);
  } else
  {
    digitalWrite(7, LOW);
  }

  Serial.print(act.isActivity);
  Serial.print(act.isInactivity);

  Serial.print(" ");
  Serial.print(act.isPosActivityOnX);
  Serial.print(act.isNegActivityOnX);
  Serial.print(" ");

  Serial.print(act.isPosActivityOnY);
  Serial.print(act.isNegActivityOnY);
  Serial.print(" ");

  Serial.print(act.isPosActivityOnZ);
  Serial.print(act.isNegActivityOnZ);
  Serial.print("\n");
  delay(50);
}