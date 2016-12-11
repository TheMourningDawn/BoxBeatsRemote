#include "application.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Sensor.h"

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

#define NEXT_BUTTON D2
#define PREVIOUS_BUTTON D4

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

TCPServer server = TCPServer(54555);
TCPClient client;

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
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void)
{
//   Serial.begin(9600);
  server.begin();

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
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

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

  if (client.connected()) {
    if(client.available()) {
        int acceleration = (int) ((accel.acceleration.z * 100) + 120);
        server.print(acceleration);
    }
    client.flush();
  } else {
    client = server.available();
  }

}