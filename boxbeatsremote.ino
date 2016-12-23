#include "application.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Sensor.h"

#define NEXT_BUTTON D2
#define PREVIOUS_BUTTON D4

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
sensors_event_t accel, mag, gyro, temp;

UDP udpMulticast;
int port = 47555;
IPAddress remoteIP(239,1,1,232);

int nextButtonState = 0;
int lastNextButtonState = 0;
int previousButtonState = 0;
int lastPreviousButtonState = 0;

void configureButtonPins(void) {
    pinMode(NEXT_BUTTON, INPUT);
    pinMode(PREVIOUS_BUTTON, INPUT);
}

void configureSensor(void)
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void)
{
  Serial.begin(11520);
  udpMulticast.begin(0);

  Serial.print("LocalIP: "); Serial.println(WiFi.localIP());
  Serial.print("Subment Mask: "); Serial.println(WiFi.subnetMask());
  Serial.print("GatewayIP: "); Serial.println(WiFi.gatewayIP());
  Serial.print("SSID: "); Serial.println(WiFi.SSID());

  if(!lsm.begin()) {
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  configureButtonPins();

  configureSensor();
}

void loop(void)
{
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  checkForModeButtonPresses();
  delay(5);
  sendAccelerationData();
}

void checkForModeButtonPresses() {
  nextButtonState = digitalRead(NEXT_BUTTON);
  previousButtonState = digitalRead(PREVIOUS_BUTTON);

  if(nextButtonState == HIGH && lastNextButtonState == HIGH) {
      delay(50);
  } else if(nextButtonState == HIGH) {
      Particle.publish("NEXT_MODE");
      lastNextButtonState = HIGH;
  } else {
      lastNextButtonState = LOW;
  }

  if(previousButtonState == HIGH && lastPreviousButtonState == HIGH) {
      delay(50);
  } else if(previousButtonState == HIGH) {
      Particle.publish("PREVIOUS_MODE");
      lastPreviousButtonState = HIGH;
  } else {
      lastPreviousButtonState = LOW;
  }
}

void sendAccelerationData() {
  int acceleration ((accel.acceleration.z * 100) + 120);
  uint8_t accelBytes[2];
  accelBytes[0] = acceleration >> 8;
  accelBytes[1] = acceleration & 0xFF;

  if(udpMulticast.sendPacket(accelBytes, 2, remoteIP, port) < 0) {
    udpMulticast.begin(0);
  }
}
