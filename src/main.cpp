#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <cmath>
#include "PID/PID.h"
#include "Filters/IIR_Filter.hpp"
#include <Filters/KalmanFilter.hpp>
#include <Servo.h>
#include <IMU.hpp>

//PINS
#define Motor0 GPIO_NUM_16
#define Motor1 GPIO_NUM_17
#define Motor2 GPIO_NUM_18
#define Motor3 GPIO_NUM_19

Servo ESC0;
Servo ESC1;
Servo ESC2;
Servo ESC3;

#define LED GPIO_NUM_4

#define BattV GPIO_NUM_35

#define SCL GPIO_NUM_22
#define SDA GPIO_NUM_21

#define minSignal 1000
#define maxSignal 2000

const char* ssid = "Drone";
const char* password = "Dr0nePass"; 
AsyncWebServer server(80);

void softAP();
void localWifi();
void i2cScanAdresses();

IMU imu;

void setup(){
  Serial.begin(115200);

  while(!Serial);

  delay(100); // Give more time for serial to initialize

  Wire.begin(SDA,SCL,400000);

  //Initialize I2C and scan for devices
  i2cScanAdresses();


  imu.begin();
  imu.CalibrateGyro();
  
  // Configure motor pins
  pinMode(Motor0,OUTPUT);
  pinMode(Motor1,OUTPUT);
  pinMode(Motor2,OUTPUT);
  pinMode(Motor3,OUTPUT);

  pinMode(LED,OUTPUT);

  pinMode(BattV,INPUT);

  // Setup PWM for motors
  ESC0.attach(Motor0);
  ESC1.attach(Motor1);
  ESC2.attach(Motor2);
  ESC3.attach(Motor3);
  

  //Chose wifi connection type
  //Serial.println("Starting WiFi connection...");
  //localWifi();
  softAP();

  Serial.println("Setup Finished");
}

// Timing variables for main loop
unsigned long PT {};
unsigned long CT, DT;
unsigned long passedTime{};
//uint16_t interval = 4000; // Frequency of code in Micros per cycle.

PID Roll(0, 0.0, 0.0);
PID Pitch(0.0, 0.0, 0.0);
PID Yaw(0.0, 0.0, 0.0);
PID Height(0.0, 0.0, 0.0);

KalmanFilter kf;

void loop(){
  CT  = micros();
  DT = CT - PT;
  PT = CT;

  imu.Update();


  kf.Predict(imu.,dt);

  //PID
  float rollCorrection = Roll.compute(0, kalmanAngleRoll, DT);
  float pitchCorrection = Pitch.compute(0, kalmanAnglePitch, DT);
  float yawCorrection = Yaw.compute(0, kalmanAngleYaw, DT);

  
    double m0 = currentThrottle - pitchCorrection - rollCorrection - yawCorrection;
    double m1 = currentThrottle + pitchCorrection - rollCorrection + yawCorrection;
    double m2 = currentThrottle + pitchCorrection + rollCorrection - yawCorrection;
    double m3 = currentThrottle - pitchCorrection + rollCorrection + yawCorrection;

    m0 = constrain(m0, minSignal, maxSignal);
    m1 = constrain(m1, minSignal, maxSignal);
    m2 = constrain(m2, minSignal, maxSignal);
    m3 = constrain(m3, minSignal, maxSignal);
    
    ESC0.writeMicroseconds(m0);
    ESC1.writeMicroseconds(m1);
    ESC2.writeMicroseconds(m2);
    ESC3.writeMicroseconds(m3);
}

void softAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("");
  Serial.println("WiFi Setup Complete.");
  Serial.println("Access Point Created");
  Serial.print("Website URL: http://");
  Serial.println(WiFi.softAPIP());
}

void localWifi(){
  WiFi.begin("TAFJORD_38A9_2GHz", "wLtZ5xHaQLPq");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("Connected To WiFi.");
  Serial.println("IP address: ");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
}

void i2cScanAdresses(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}