#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <string>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <cmath>
#include "PID.h"

//PINS
#define Motor0 GPIO_NUM_16
#define Motor1 GPIO_NUM_17
#define Motor2 GPIO_NUM_18
#define Motor3 GPIO_NUM_19

#define LED GPIO_NUM_4

#define SCL GPIO_NUM_22
#define SDA GPIO_NUM_21

const char* ssid = "Drone";
const char* password = "Dr0nePass"; 
WiFiServer server(80);
String header;

void softAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("");
  Serial.println("WiFi Setup Complete.");
  Serial.println("Access Point Created");
  Serial.print("Website URL: http://");
  Serial.println(WiFi.softAPIP());
  server.begin();
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
  server.begin();
}
void website();
void handlePIDUpdate(String pidType, String header, double &kp, double &ki, double &kd);
void sendWebPage(WiFiClient& client);
void i2cScanAdresses();
void oneSecond();

// LED control variables
uint8_t ledBrightness = 0;
String ledToggle = "off";

// Global sensor values for website display
double currentRoll = 0.0;
double currentPitch = 0.0;
double currentHeading = 0.0;

// PID parameters for website tuning
double rollKp = 0.0, rollKi = 0.0, rollKd = 0.0;
double pitchKp = 0.0, pitchKi = 0.0, pitchKd = 0.0;
double yawKp = 0.0, yawKi = 0.0, yawKd = 0.0;

// Adafruit_ICM20948 icm;

void setup(){
  Serial.begin(115200);
  delay(10);

  // Wire.begin(SDA,SCL);

  // Initialize I2C and scan for devices
  //i2cScanAdresses();
  
  // Configure motor pins
  pinMode(Motor0,OUTPUT);
  pinMode(Motor1,OUTPUT);
  pinMode(Motor2,OUTPUT);
  pinMode(Motor3,OUTPUT);

  pinMode(LED,OUTPUT);

  //1ms at 512, 2ms at 1024
  // Setup PWM for motors
  ledcSetup(0,50,10);
  ledcAttachPin(Motor0,0);

  ledcSetup(1,50,10);
  ledcAttachPin(Motor1,1);

  ledcSetup(2,50,10);
  ledcAttachPin(Motor2,2);

  ledcSetup(3,50,10);
  ledcAttachPin(Motor3,3);

  //Chose wifi connection type
  localWifi();
  //softAP();

  // icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  // icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  // icm.setMagDataRate(ak09916_data_rate_t::AK09916_MAG_DATARATE_100_HZ);
  // icm.begin_I2C();
}

unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

unsigned long PT {};
unsigned long CT, DT;


PID Roll(rollKp, rollKi, rollKd, DT);
PID Pitch(pitchKp, pitchKi, pitchKd, DT);
PID Yaw(yawKp, yawKi, yawKd, DT);
PID Height(0.0, 0.0, 0.0, DT);


void loop(){
  CT  = millis();
  DT = CT - PT;

  // sensors_event_t acc;
  // sensors_event_t gyro;
  // sensors_event_t mag;
  // sensors_event_t temp;
  // icm.getEvent(&acc, &gyro, &temp, &mag);

  // double alpha = 0.98;

  // double pitch = std::atan(-acc.acceleration.x/(sqrt(pow(acc.acceleration.y,2)+pow(acc.acceleration.z,2))));
  // double roll =  std::atan(acc.acceleration.y/(sqrt(pow(acc.acceleration.z,2)+pow(acc.acceleration.z,2))));
  // double heading = std::atan2(mag.magnetic.y, mag.magnetic.x);

  // pitch = alpha * (pitch + gyro.gyro.x * (DT / 1000.0)) + (1 - alpha) * pitch;
  // roll = alpha * (roll + gyro.gyro.y * (DT / 1000.0)) + (1 - alpha) * roll;
  // heading = alpha * (heading + gyro.gyro.z * (DT / 1000.0)) + (1 - alpha) * heading;

float roll{}, pitch{}, heading;{} // Declare variables to hold the computed values

  // Update global variables for website display
  currentRoll = roll * 180.0 / PI;    // Convert to degrees
  currentPitch = pitch * 180.0 / PI;   // Convert to degrees
  currentHeading = heading * 180.0 / PI; // Convert to degrees

  double rollCorrection = Roll.compute(0, roll);
  double pitchCorreection = Pitch.compute(0, pitch);
  double headingCorrection = Yaw.compute(0, heading);


  website();

  PT = CT;
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

void oneSecond(){
  static double prevTime = 0;
  static double elapsedTime = 0;
  static int interval = 1000; // Changed to 1000ms for 1 second interval

  uint64_t currentTime = millis(); // Fixed u64_t to uint64_t
  double DT = currentTime - prevTime;
  elapsedTime += DT;

  if (elapsedTime >= interval){
    // // Print sensor data every second
    // Serial.print("Heading: ");
    // Serial.print(heading);
    // Serial.print("° | Gravity: ");
    // Serial.print(grav);
    // Serial.println(" m/s²");
    
    elapsedTime = 0; // Reset elapsed time
  }

  prevTime = currentTime;
}

void website() {
    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {
        currentTime = millis();
        previousTime = currentTime;
        Serial.println("New Client.");
        String currentLine = "";
        while (client.connected() && currentTime - previousTime <= timeoutTime) {
            currentTime = millis();
            if (client.available()) {
                char c = client.read();
                Serial.write(c);
                header += c;
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                        // Handle LED toggle
                        if (header.indexOf("GET /led/on") >= 0) {
                            ledToggle = "on";
                            analogWrite(LED, ledBrightness); // Use PWM for brightness
                        } else if (header.indexOf("GET /led/off") >= 0) {
                            ledToggle = "off";
                            analogWrite(LED, 0);
                        }
                        // Handle LED brightness slider
                        int sliderIndex = header.indexOf("GET /led/brightness?");
                        if (sliderIndex >= 0) {
                            int valStart = header.indexOf("value=", sliderIndex);
                            if (valStart >= 0) {
                                int valEnd = header.indexOf(' ', valStart);
                                String valStr = header.substring(valStart + 6, valEnd);
                                int brightness = valStr.toInt();
                                ledBrightness = brightness;
                                if (ledToggle == "on") {
                                    analogWrite(LED, ledBrightness);
                                }
                            }
                        }
                        
                        // Handle PID tuning - Roll
                        if (header.indexOf("GET /pid/roll?") >= 0) {
                            handlePIDUpdate("roll", header, rollKp, rollKi, rollKd);
                            Roll.setP(rollKp);
                            Roll.setI(rollKi);
                            Roll.setD(rollKd);
                        }
                        
                        // Handle PID tuning - Pitch
                        if (header.indexOf("GET /pid/pitch?") >= 0) {
                            handlePIDUpdate("pitch", header, pitchKp, pitchKi, pitchKd);
                            Pitch.setP(pitchKp);
                            Pitch.setI(pitchKi);
                            Pitch.setD(pitchKd);
                        }
                        
                        // Handle PID tuning - Yaw
                        if (header.indexOf("GET /pid/yaw?") >= 0) {
                            handlePIDUpdate("yaw", header, yawKp, yawKi, yawKd);
                            Yaw.setP(yawKp);
                            Yaw.setI(yawKi);
                            Yaw.setD(yawKd);
                        }
                        
                        // Send HTTP response
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println("Connection: close");
                        client.println();
                        
                        // Send HTML page
                        sendWebPage(client);
                        break;
                    } else {
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
            }
        }
        header = "";
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
}

void handlePIDUpdate(String pidType, String header, double &kp, double &ki, double &kd) {
    int kpStart = header.indexOf("kp=");
    int kiStart = header.indexOf("ki=");
    int kdStart = header.indexOf("kd=");
    
    if (kpStart >= 0) {
        int kpEnd = header.indexOf('&', kpStart);
        if (kpEnd < 0) kpEnd = header.indexOf(' ', kpStart);
        String kpStr = header.substring(kpStart + 3, kpEnd);
        kp = kpStr.toFloat();
    }
    
    if (kiStart >= 0) {
        int kiEnd = header.indexOf('&', kiStart);
        if (kiEnd < 0) kiEnd = header.indexOf(' ', kiStart);
        String kiStr = header.substring(kiStart + 3, kiEnd);
        ki = kiStr.toFloat();
    }
    
    if (kdStart >= 0) {
        int kdEnd = header.indexOf('&', kdStart);
        if (kdEnd < 0) kdEnd = header.indexOf(' ', kdStart);
        String kdStr = header.substring(kdStart + 3, kdEnd);
        kd = kdStr.toFloat();
    }
}

void sendWebPage(WiFiClient& client) {
    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.println("<link rel=\"icon\" href=\"data:,\">");
    client.println("<style>");
    client.println("* { margin: 0; padding: 0; box-sizing: border-box; }");
    client.println("html, body { font-family: Helvetica; background-color: #f0f0f0; height: 100vh; overflow: hidden; }");
    client.println(".main-container { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: auto 1fr; gap: 15px; padding: 15px; height: 100vh; }");
    client.println(".header { grid-column: 1 / -1; text-align: center; background-color: white; padding: 15px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }");
    client.println(".top-left { background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); border-left: 4px solid #4CAF50; }");
    client.println(".top-right { background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); border-left: 4px solid #2196F3; }");
    client.println(".bottom-half { grid-column: 1 / -1; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); border-left: 4px solid #FF9800; overflow-y: auto; }");
    client.println(".pid-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; }");
    client.println(".pid-box { background-color: #fff5f5; padding: 15px; border-radius: 8px; border-left: 4px solid #ff6b6b; }");
    client.println(".button { background-color: #05a100ff; border: none; color: white; padding: 10px 20px; text-decoration: none; font-size: 16px; margin: 5px; cursor: pointer; border-radius: 5px; }");
    client.println(".button2 { background-color: #c6d3f0ff; }");
    client.println("input[type=range] { width: 100%; margin: 8px 0; }");
    client.println("input[type=number] { width: 70px; padding: 4px; margin: 2px; border: 1px solid #ddd; border-radius: 4px; }");
    client.println(".pid-form { text-align: left; }");
    client.println("h1 { color: #2869bdff; margin: 0; font-size: 24px; }");
    client.println("h2 { color: #333; margin-bottom: 15px; font-size: 20px; }");
    client.println("h3 { color: #555; margin-bottom: 10px; font-size: 18px; }");
    client.println(".sensor-value { font-size: 18px; font-weight: bold; color: #2196F3; margin: 8px 0; }");
    client.println(".led-status { font-size: 16px; margin: 8px 0; }");
    client.println(".brightness-container { margin: 15px 0; }");
    client.println("@media (max-width: 768px) { .main-container { grid-template-columns: 1fr; grid-template-rows: auto auto auto 1fr; } .bottom-half { grid-column: 1; } }");
    client.println("</style>");
    client.println("<script>");
    client.println("setInterval(function() { window.location.reload(); }, 3000);"); // Auto-refresh every 3 seconds
    client.println("</script>");
    client.println("</head>");
    client.println("<body>");
    
    client.println("<div class=\"main-container\">");
    
    // Header
    client.println("<div class=\"header\">");
    client.println("<h1>🚁 Drone Control Panel</h1>");
    client.println("</div>");
    
    // Top Left - LED Control
    client.println("<div class=\"top-left\">");
    client.println("<h2>💡 LED Control</h2>");
    client.println("<div class=\"led-status\"><strong>Status:</strong> " + ledToggle + "</div>");
    if (ledToggle == "off") {
        client.println("<a href=\"/led/on\"><button class=\"button\">TURN ON</button></a>");
    } else {
        client.println("<a href=\"/led/off\"><button class=\"button button2\">TURN OFF</button></a>");
    }
    client.println("<div class=\"brightness-container\">");
    client.println("<div><strong>Brightness:</strong> " + String(ledBrightness) + "/255</div>");
    client.println("<form action=\"/led/brightness\" method=\"get\">");
    client.println("<input type=\"range\" min=\"0\" max=\"255\" value=\"" + String(ledBrightness) + "\" name=\"value\" onchange=\"this.form.submit()\">");
    client.println("</form>");
    client.println("</div>");
    client.println("</div>");
    
    // Top Right - IMU Sensor Data
    client.println("<div class=\"top-right\">");
    client.println("<h2>📊 Orientation</h2>");
    client.println("<div class=\"sensor-value\">Roll: " + String(currentRoll, 1) + "°</div>");
    client.println("<div class=\"sensor-value\">Pitch: " + String(currentPitch, 1) + "°</div>");
    client.println("<div class=\"sensor-value\">Heading: " + String(currentHeading, 1) + "°</div>");
    client.println("</div>");
    
    // Bottom Half - PID Tuning
    client.println("<div class=\"bottom-half\">");
    client.println("<h2>⚙️ PID Controller Tuning</h2>");
    client.println("<div class=\"pid-grid\">");
    
    // Roll PID
    client.println("<div class=\"pid-box\">");
    client.println("<h3>Roll PID</h3>");
    client.println("<div class=\"pid-form\">");
    client.println("<form action=\"/pid/roll\" method=\"get\">");
    client.println("Kp: <input type=\"number\" step=\"0.01\" name=\"kp\" value=\"" + String(rollKp, 3) + "\"><br><br>");
    client.println("Ki: <input type=\"number\" step=\"0.001\" name=\"ki\" value=\"" + String(rollKi, 3) + "\"><br><br>");
    client.println("Kd: <input type=\"number\" step=\"0.001\" name=\"kd\" value=\"" + String(rollKd, 3) + "\"><br><br>");
    client.println("<button type=\"submit\" class=\"button\">Update</button>");
    client.println("</form>");
    client.println("</div>");
    client.println("</div>");
    
    // Pitch PID
    client.println("<div class=\"pid-box\">");
    client.println("<h3>Pitch PID</h3>");
    client.println("<div class=\"pid-form\">");
    client.println("<form action=\"/pid/pitch\" method=\"get\">");
    client.println("Kp: <input type=\"number\" step=\"0.01\" name=\"kp\" value=\"" + String(pitchKp, 3) + "\"><br><br>");
    client.println("Ki: <input type=\"number\" step=\"0.001\" name=\"ki\" value=\"" + String(pitchKi, 3) + "\"><br><br>");
    client.println("Kd: <input type=\"number\" step=\"0.001\" name=\"kd\" value=\"" + String(pitchKd, 3) + "\"><br><br>");
    client.println("<button type=\"submit\" class=\"button\">Update</button>");
    client.println("</form>");
    client.println("</div>");
    client.println("</div>");
    
    // Yaw PID
    client.println("<div class=\"pid-box\">");
    client.println("<h3>Yaw PID</h3>");
    client.println("<div class=\"pid-form\">");
    client.println("<form action=\"/pid/yaw\" method=\"get\">");
    client.println("Kp: <input type=\"number\" step=\"0.01\" name=\"kp\" value=\"" + String(yawKp, 3) + "\"><br><br>");
    client.println("Ki: <input type=\"number\" step=\"0.001\" name=\"ki\" value=\"" + String(yawKi, 3) + "\"><br><br>");
    client.println("Kd: <input type=\"number\" step=\"0.001\" name=\"kd\" value=\"" + String(yawKd, 3) + "\"><br><br>");
    client.println("<button type=\"submit\" class=\"button\">Update</button>");
    client.println("</form>");
    client.println("</div>");
    client.println("</div>");
    
    client.println("</div>"); // Close pid-grid
    client.println("</div>"); // Close bottom-half
    client.println("</div>"); // Close main-container
    client.println("</body></html>");
    client.println();
}