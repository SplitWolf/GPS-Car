#include <Arduino.h>
#include "HMC5883L_Driver.h"

static const uint8_t FL = 6, FL2 = 7, FR = 2, FR2 = 3, RL = 8, RL2 = 9, RR = 4, RR2 = 5;
static const uint32_t GPSBaud = 9600;
static HMC5883L driver = HMC5883L(19,20);

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  pinMode(FL, OUTPUT);
  pinMode(FL2, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(FR2, OUTPUT);
  pinMode(RL, OUTPUT);
  pinMode(RL2, OUTPUT);
  pinMode(RR, OUTPUT);
  pinMode(RR2, OUTPUT);
  driver.begin();
}


void loop() {
  driver.waitForDataReady();
  HMC5883L_Data data = driver.readData();
  Serial.printf("X: %d Y: %d Z: %d \n",data.x,data.y,data.z);

  float heading = atan2f(data.y,data.x) * 180/PI +180;
  Serial.printf("Heading: %f deg \n", heading);
  delay(2000);  
}