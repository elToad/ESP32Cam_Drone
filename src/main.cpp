// #include <Wire.h>
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include "PID/PID.h"
// #include "Filters/KalmanCompFilter.h"
// #include "IMU.h"
// #include "Filters/IIR_Filter.hpp"

// //PINS
// #define Motor0 GPIO_NUM_16
// #define Motor1 GPIO_NUM_17
// #define Motor2 GPIO_NUM_18
// #define Motor3 GPIO_NUM_19

// #define LED GPIO_NUM_4

// #define BattV GPIO_NUM_35

// #define SCL GPIO_NUM_22
// #define SDA GPIO_NUM_21

// #define freq 50 //50Hz
// #define res 12
// #define minSignal (0.05*std::pow(2,res)) //1ms
// #define maxSignal (minSignal*2) //2ms

// const char* ssid = "Drone";
// const char* password = "Dr0nePass"; 
// AsyncWebServer server(80);

// // Global variables for web interface
// bool isArmed = false;
// bool emergencyStop = false;
// float throttlePercent = 0.0;
// int ledBrightness = 0;
// bool ledState = false;


// void softAP();
// void localWifi();
// void i2cScanAdresses();
// void setupWebServer();
// String getWebPage();

// // Timing variables for main loop
// unsigned long PT {};
// unsigned long CT, DT;
// unsigned long passedTime{};
// const uint16_t interval = 4000; // Frequency of code in Micros per cycle.

// IMU imu;

// KalmanCompFilter KalmanRoll(2*2); // Uncertainty is expected error of starting angle, in this case i expect it to be about 2 degrees.
// KalmanCompFilter KalmanPitch(2*2);
// KalmanCompFilter KalmanYaw(2*2); // Yaw can be anything between -180 to 180 so i inserted a random value


// PID PIDRoll(0.0,0.0,0.0);
// PID PIDPitch(0.0,0.0,0.0);
// PID PIDYaw(0.0,0.0,0.0);
// PID PIDHeight(0.0,0.0,0.0);

// void setup(){
//   Serial.begin(115200);

//     while(!Serial);

//   delay(100); // Give more time for serial to initialize

//   Wire.begin(SDA,SCL,400000);
//   //Initialize I2C and scan for devices
//   i2cScanAdresses();
  
//   imu.begin();
//   imu.CalibrateGyro();
  
//   // Configure motor pins
//   pinMode(Motor0,OUTPUT);
//   pinMode(Motor1,OUTPUT);
//   pinMode(Motor2,OUTPUT);
//   pinMode(Motor3,OUTPUT);

//   pinMode(LED,OUTPUT);

//   pinMode(BattV,INPUT);

//   // Setup PWM for motors
//   ledcSetup(0, freq, res);
//   ledcSetup(1, freq, res);
//   ledcSetup(2, freq, res);
//   ledcSetup(3, freq, res);
//   ledcSetup(4, 5000, 8);
  
//   ledcAttachPin(Motor0, 0);
//   ledcAttachPin(Motor1, 1);
//   ledcAttachPin(Motor2, 2);
//   ledcAttachPin(Motor3, 3);
//   ledcAttachPin(LED, 4);
  

//   //Chose wifi connection type
//   //localWifi();
//   softAP();
  
//   // Setup web server
//   setupWebServer();

//   Serial.println("Setup Finished");
// }

// double Roll{};
// double Pitch{};
// double Yaw{};

// IIR_Filter magnoFiltered(0.96);

// void loop(){
//   CT  = micros();
//   DT = CT - PT;
//   PT = CT;
//   passedTime += DT;
//   if (passedTime < interval) return;
//   passedTime = 0;
//   double DTSeconds = DT / 1000000;

//   //Code Runs Here After Interval
//   //Due to the rotation of my imu, in my case roll is x and pitch is y.
//   imu.Update();

//   Roll = KalmanRoll.Compute(imu.GX, imu.GetRoll(), DTSeconds);
//   Pitch = KalmanPitch.Compute(imu.GY, imu.GetPitch(), DTSeconds);
//   // Yaw = KalmanYaw.Compute(imu.GetYaw(),imu.GZ,DTSeconds);

//   Yaw = imu.GetYaw();
//   Yaw = magnoFiltered.filter(Yaw);
//   // Serial.println(Yaw);

//   Serial.print("Roll");
//   Serial.println(Roll);
//   Serial.print("Pitch");
//   Serial.println(Pitch);

//   Serial.print("Roll");
//   Serial.println(imu.GetRoll());
//   Serial.print("Pitch");
//   Serial.println(imu.GetPitch());


//   //PID
//   float rollCorrection = PIDRoll.compute(0,Roll,DTSeconds);
//   float pitchCorrection = PIDPitch.compute(0, Pitch,DTSeconds);
//   float yawCorrection = PIDYaw.compute(0, Yaw,DTSeconds);
  
//   // Calculate throttle based on percentage (0-100%) from web interface
//   float currentThrottle =  minSignal + (throttlePercent / 100.0) * (maxSignal - minSignal);
//   //Code to run if armed is active on webpage
//   if (isArmed && !emergencyStop) {
//     double m0 = currentThrottle - pitchCorrection - rollCorrection + yawCorrection;
//     double m1 = currentThrottle + pitchCorrection - rollCorrection - yawCorrection;
//     double m2 = currentThrottle + pitchCorrection + rollCorrection + yawCorrection;
//     double m3 = currentThrottle - pitchCorrection + rollCorrection - yawCorrection;
    
//     ledcWrite(0, (uint32_t)m0);
//     ledcWrite(1, (uint32_t)m1);
//     ledcWrite(2, (uint32_t)m2);
//     ledcWrite(3, (uint32_t)m3);
//   } else {
//     // Safety: Set all motors to minimum when disarmed or emergency stop
//     ledcWrite(0, (uint32_t)minSignal);
//     ledcWrite(1, (uint32_t)minSignal);
//     ledcWrite(2, (uint32_t)minSignal);
//     ledcWrite(3, (uint32_t)minSignal);
//   }

//   // Update LED based on web interface
//   ledcWrite(4, ledState ? ledBrightness : 0);
// }

// void softAP(){
//   WiFi.mode(WIFI_AP);
//   WiFi.softAP(ssid, password);

//   Serial.println("");
//   Serial.println("WiFi Setup Complete.");
//   Serial.println("Access Point Created");
//   Serial.print("Website URL: http://");
//   Serial.println(WiFi.softAPIP());
// }

// void localWifi(){
//   WiFi.begin("TAFJORD_38A9_2GHz", "wLtZ5xHaQLPq");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   // Print local IP address and start web server
//   Serial.println("");
//   Serial.println("Connected To WiFi.");
//   Serial.println("IP address: ");
//   Serial.print("http://");
//   Serial.println(WiFi.localIP());
// }

// void i2cScanAdresses(){
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for(address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.print(address,HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error==4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.println(address,HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
// }
// //Reccomend not touching anything below this point, no clue what the hell is going on there.
// void setupWebServer() {

//   // Serve the main webpage
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send(200, "text/html", getWebPage().c_str());
//   });

//   // API endpoint for sensor data
//   server.on("/api/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
//     // Read battery voltage (assuming 12V system with voltage divider)
//     double batteryRaw = analogRead(BattV);
//     float batteryPercent = map(batteryRaw, 2265,3724, 0, 100);
//     // Serial.println(batteryRaw,5);
    
//     String json = "{";
//     json += "\"roll\":" + String(Roll * RAD_TO_DEG, 2) + ",";
//     json += "\"pitch\":" + String(Pitch * RAD_TO_DEG, 2) + ",";
//     json += "\"heading\":" + String(Yaw * RAD_TO_DEG, 2) + ",";
//     json += "\"armed\":" + String(isArmed ? "true" : "false") + ",";
//     json += "\"emergency\":" + String(emergencyStop ? "true" : "false") + ",";
//     json += "\"throttle\":" + String(throttlePercent, 1) + ",";
//     json += "\"battery\":" + String(batteryPercent, 1);
//     json += "}";
//     request->send(200, "application/json", json);
//   });

//   // Throttle control
//   server.on("/throttle", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("value")) {
//       throttlePercent = request->getParam("value")->value().toFloat();
//       throttlePercent = constrain(throttlePercent, 0.0, 100.0);
//       Serial.println("Throttle set to: " + String(throttlePercent) + "%");
//     }
//     request->send(200, "text/plain", "OK");
//   });

//   // LED brightness control
//   server.on("/brightness", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("value")) {
//       ledBrightness = request->getParam("value")->value().toInt();
//       ledBrightness = constrain(ledBrightness, 0, 255);
//     }
//     request->send(200, "text/plain", "OK");
//   });

//   // LED state control
//   server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("state")) {
//       String state = request->getParam("state")->value();
//       if (state == "on") {
//         ledState = true;
//       } else if (state == "off") {
//         ledState = false;
//       } else if (state == "blink") {
//         // Simple blink implementation
//         ledState = !ledState;
//       }
//     }
//     request->send(200, "text/plain", "OK");
//   });

//   // PID control
//   server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("type") && request->hasParam("kp") && 
//         request->hasParam("ki") && request->hasParam("kd")) {
      
//       String type = request->getParam("type")->value();
//       float kp = request->getParam("kp")->value().toFloat();
//       float ki = request->getParam("ki")->value().toFloat();
//       float kd = request->getParam("kd")->value().toFloat();
      
//       if (type == "roll") {
//         PIDRoll.setP(kp);
//         PIDRoll.setI(ki);
//         PIDRoll.setD(kd);
//         PIDRoll.reset();
//       } else if (type == "pitch") {
//         PIDPitch.setP(kp);
//         PIDPitch.setI(ki);
//         PIDPitch.setD(kd);
//         PIDPitch.reset();
//       } else if (type == "yaw") {
//         PIDYaw.setP(kp);
//         PIDYaw.setI(ki);
//         PIDYaw.setD(kd);
//         PIDYaw.reset();
//       } else if (type == "height") {
//         PIDHeight.setP(kp);
//         PIDHeight.setI(ki);
//         PIDHeight.setD(kd);
//         PIDHeight.reset();
//       }
      
//       Serial.println("PID " + type + " updated: P=" + String(kp) + " I=" + String(ki) + " D=" + String(kd));
//     }
//     request->send(200, "text/plain", "OK");
//   });

//   // Flight mode and arming controls
//   server.on("/flight/arm", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("status")) {
//       String status = request->getParam("status")->value();
//       if (status == "armed" && !emergencyStop) {
//         isArmed = true;
//         Serial.println("DRONE ARMED");
//       } else {
//         isArmed = false;
//         Serial.println("DRONE DISARMED");
//       }
//     }
//     request->send(200, "text/plain", "OK");
//   });

//   // Emergency stop
//   server.on("/emergency/stop", HTTP_GET, [](AsyncWebServerRequest *request){
//     emergencyStop = true;
//     isArmed = false;
//     throttlePercent = 0.0;
//     Serial.println("EMERGENCY STOP ACTIVATED!");
//     request->send(200, "text/plain", "EMERGENCY STOP ACTIVATED");
//   });

//   // Clear emergency stop
//   server.on("/emergency/clear", HTTP_GET, [](AsyncWebServerRequest *request){
//     emergencyStop = false;
//     Serial.println("Emergency stop cleared - drone can be armed again");
//     request->send(200, "text/plain", "Emergency cleared");
//   });

//   server.begin();
//   Serial.println("Web server started");
// }

// String getWebPage() {
//   return R"=====(<!DOCTYPE HTML>
// <html>
// <head>
//   <title>DRONE CONTROL SYSTEM</title>
//   <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
//   <meta charset="utf-8">
//   <style>
//     @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap');
    
//     * { box-sizing: border-box; }
    
//     body {
//       font-family: 'Orbitron', monospace;
//       text-align: center; 
//       margin: 0;
//       padding: 20px;
//       min-height: 100vh;
//       color: #E0F2FE;
//       overflow-x: hidden;
//       position: relative;
//     }
    
//     body::before {
//       content: '';
//       position: fixed;
//       top: 0;
//       left: 0;
//       width: 100%;
//       height: 100%;
//       background: 
//         radial-gradient(circle at 85% 20%, #FFD700 0%, #FFA500 15%, transparent 25%),
//         radial-gradient(ellipse 300px 60px at 20% 25%, rgba(255,255,255,0.8) 0%, transparent 60%),
//         radial-gradient(ellipse 200px 40px at 60% 15%, rgba(255,255,255,0.6) 0%, transparent 60%),
//         linear-gradient(180deg, #87CEEB 0%, #4682B4 40%, #2E86AB 60%, #1E3A8A 80%, #0F172A 100%);
//       z-index: -2;
//     }
    
//     .systemHeader {
//       text-align: center;
//       margin-bottom: 30px;
//       padding: 20px;
//       background: rgba(70, 131, 180, 0.74);
//       border: 2px solid #4682B4;
//       border-radius: 10px;
//       box-shadow: 0 0 20px rgba(70,130,180,0.4);
//     }
    
//     .systemHeader h1 {
//       font-size: 2.5em;
//       font-weight: 900;
//       margin: 0;
//       text-shadow: 0 0 20px #4682B4;
//       letter-spacing: 3px;
//       color: #E0F2FE;
//     }
    
//     .topHalf {
//       display: flex;
//       justify-content: space-between;
//       align-items: stretch;
//       margin-bottom: 30px;
//       min-height: 50vh;
//       gap: 20px;
//     }
    
//     .ledContrainer, .imuContainer, .statusPanelInline {
//       width: 32%;
//       min-height: 100%;
//       display: flex;
//       flex-direction: column;
//       justify-content: space-between;
//       padding: 25px;
//       border-radius: 15px;
//       position: relative;
//       overflow: hidden;
//       transition: all 0.3s ease;
//       z-index: 1;
//     }
    
//     .ledContrainer { 
//       background: linear-gradient(145deg, #1E40AF, #3B82F6);
//       border: 2px solid #60A5FA;
//       box-shadow: 0 8px 32px rgba(96,165,250,0.3);
//     }
    
//     .ledContrainer:hover {
//       transform: translateY(-2px);
//       z-index: 2;
//     }
    
//     .imuContainer {
//       background: linear-gradient(145deg, #059669, #10B981);
//       border: 2px solid #34D399;
//       box-shadow: 0 8px 32px rgba(52,211,153,0.3);
//     }
    
//     .imuContainer:hover {
//       transform: translateY(-2px);
//       z-index: 2;
//     }
    
//     /* PID Container Styling */
//     .pidContainer {
//       background: linear-gradient(145deg, #EA580C, #F97316);
//       flex: 1;
//       margin: 10px;
//       border: 2px solid #FB923C;
//       padding: 20px;
//       border-radius: 12px;
//       box-shadow: 
//         0 8px 25px rgba(251,146,60,0.3),
//         inset 0 0 15px rgba(254,215,170,0.1);
//       color: #FEF2F2;
//       position: relative;
//       overflow: hidden;
//       transition: all 0.3s ease;
//       backdrop-filter: blur(5px);
//     }
    
//     .pidContainer:hover {
//       box-shadow: 
//         0 12px 35px rgba(251,146,60,0.5),
//         inset 0 0 25px rgba(254,215,170,0.2);
//       transform: translateY(-3px);
//     }
    
//     .pidContainer::before {
//       content: '';
//       position: absolute;
//       top: 0;
//       left: -100%;
//       width: 100%;
//       height: 100%;
//       background: linear-gradient(90deg, transparent, rgba(254,215,170,0.2), transparent);
//       transition: left 0.6s;
//     }
    
//     .pidContainer:hover::before {
//       left: 100%;
//     }
    
//     .pidContainer h3 {
//       text-align: center;
//       font-weight: 700;
//       font-size: 1.2em;
//       margin-bottom: 15px;
//       text-shadow: 0 0 10px #EA580C;
//       letter-spacing: 1px;
//       color: #FEF2F2;
//     }
    
//     .pidGrid {
//       display: flex;
//       flex-wrap: wrap;
//       justify-content: space-between;
//       gap: 15px;
//       margin-top: 20px;
//     }
    
//     .pidInput {
//       width: 80px;
//       padding: 8px;
//       margin: 5px 0;
//       border: 2px solid #71c2f8;
//       border-radius: 6px;
//       font-size: 14px;
//       font-family: 'Orbitron', monospace;
//       background: rgba(15,23,42,0.6);
//       color: #FEF2F2;
//       text-align: center;
//       backdrop-filter: blur(5px);
//       transition: all 0.3s ease;
//     }
    
//     .pidInput:focus {
//       border-color: #FCA5A5;
//       box-shadow: 0 0 15px rgba(248,113,113,0.5);
//       outline: none;
//     }
    
//     .pidLabel {
//       display: inline-block;
//       width: 25px;
//       font-weight: bold;
//       text-shadow: 0 0 5px #DC2626;
//       color: #FEF2F2;
//     }
    
//     .statusPanel {
//       position: fixed;
//       top: 20px;
//       left: 50%;
//       transform: translateX(-50%);
//       width: 300px;
//       background: rgba(15,23,42,0.9);
//       border: 2px solid #4682B4;
//       border-radius: 12px;
//       padding: 15px;
//       backdrop-filter: blur(10px);
//       z-index: 1000;
//     }

//     .statusPanelInline {
//       background: linear-gradient(145deg, #1F2937, #374151);
//       border: 2px solid #1F2937;
//       box-shadow: 
//         0 8px 32px rgba(31,41,55,0.5),
//         inset 0 0 20px rgba(75,85,99,0.1);
//     }
    
//     .statusPanelInline::before {
//       content: '';
//       position: absolute;
//       top: 0;
//       left: -100%;
//       width: 100%;
//       height: 100%;
//       background: linear-gradient(90deg, transparent, rgba(75,85,99,0.3), transparent);
//       transition: left 0.5s;
//       z-index: 1;
//     }
    
//     .statusPanelInline:hover {
//       box-shadow: 
//         0 12px 40px rgba(31,41,55,0.7),
//         inset 0 0 30px rgba(75,85,99,0.2);
//       transform: translateY(-2px);
//       z-index: 2;
//     }
    
//     .statusPanelInline:hover::before {
//       left: 100%;
//     }
    
//     .statusPanelInline h2 {
//       color: #E0F2FE;
//       text-shadow: 0 0 15px #60A5FA;
//     }

//     .statusPanel h3 {
//       margin: 0 0 15px 0;
//       color: #E0F2FE;
//       text-shadow: 0 0 10px #4682B4;
//     }    /* Battery Monitor */
//     .batteryContainer {
//       margin-bottom: 15px;
//     }
    
//     .batteryFrame {
//       width: 100px;
//       height: 40px;
//       border: 3px solid #4682B4;
//       border-radius: 4px;
//       position: relative;
//       margin: 10px auto;
//       background: rgba(0,0,0,0.3);
//     }
    
//     .batteryFrame::after {
//       content: '';
//       position: absolute;
//       right: -8px;
//       top: 50%;
//       transform: translateY(-50%);
//       width: 5px;
//       height: 16px;
//       background: #4682B4;
//       border-radius: 0 2px 2px 0;
//     }
    
//     .batteryLevel {
//       height: 100%;
//       background: linear-gradient(90deg, #EF4444 0%, #F59E0B 50%, #10B981 100%);
//       border-radius: 2px;
//       transition: width 0.5s ease;
//       position: relative;
//       overflow: hidden;
//     }
    
//     .batteryLevel.low { background: linear-gradient(90deg, #EF4444, #DC2626); }
//     .batteryLevel.medium { background: linear-gradient(90deg, #F59E0B, #D97706); }
//     .batteryLevel.high { background: linear-gradient(90deg, #10B981, #059669); }
    
//     .batteryLevel::after {
//       content: '';
//       position: absolute;
//       top: 0;
//       left: -100%;
//       width: 100%;
//       height: 100%;
//       background: linear-gradient(90deg, transparent, rgba(255,255,255,0.4), transparent);
//       animation: batteryShine 2s infinite;
//     }
    
//     @keyframes batteryShine {
//       0% { left: -100%; }
//       100% { left: 100%; }
//     }
    
//     .batteryPercent {
//       position: absolute;
//       top: 50%;
//       left: 50%;
//       transform: translate(-50%, -50%);
//       color: white;
//       font-weight: bold;
//       font-size: 0.9em;
//       text-shadow: 1px 1px 2px rgba(0,0,0,0.8);
//       z-index: 10;
//     }
    
//     .statusItem {
//       display: flex;
//       justify-content: space-between;
//       align-items: center;
//       margin: 8px 0;
//       padding: 6px;
//       background: rgba(70,130,180,0.1);
//       border-radius: 6px;
//       font-size: 0.9em;
//     }
    
//     .slider-container { margin: 20px 0; }
    
//     .slider { 
//       width: 100%; 
//       height: 8px;
//       background: linear-gradient(90deg, #1E3A8A, #3B82F6);
//       outline: none;
//       border-radius: 5px;
//       -webkit-appearance: none;
//       appearance: none;
//     }
    
//     .slider::-webkit-slider-thumb {
//       -webkit-appearance: none;
//       width: 25px;
//       height: 25px;
//       background: linear-gradient(45deg, #60A5FA, #93C5FD);
//       border-radius: 50%;
//       cursor: pointer;
//       box-shadow: 0 0 15px rgba(96,165,250,0.6);
//     }
    
//     .button { 
//       padding: 12px 24px; 
//       margin: 8px; 
//       font-size: 14px;
//       font-family: 'Orbitron', monospace;
//       font-weight: 700;
//       border: 2px solid;
//       border-radius: 8px; 
//       cursor: pointer;
//       transition: all 0.3s ease;
//       text-transform: uppercase;
//       letter-spacing: 1px;
//       min-width: 120px;
//     }
    
//     .button:hover {
//       transform: translateY(-2px);
//       box-shadow: 0 5px 15px rgba(0,0,0,0.3);
//     }
    
//     .on { 
//       background: linear-gradient(45deg, #1D4ED8, #3B82F6);
//       color: white;
//       border-color: #2563EB;
//       box-shadow: 0 3px 10px rgba(29,78,216,0.4);
//     }
    
//     .off { 
//       background: linear-gradient(45deg, #DC2626, #EF4444);
//       color: white;
//       border-color: #DC2626;
//       box-shadow: 0 3px 10px rgba(220,38,38,0.4);
//     }
    
//     .emergency-btn {
//       width: 100%;
//       padding: 15px;
//       background: linear-gradient(45deg, #DC2626, #EF4444);
//       border: none;
//       border-radius: 6px;
//       color: white;
//       font-family: 'Orbitron', monospace;
//       font-weight: 700;
//       cursor: pointer;
//       font-size: 16px;
//       margin-top: 10px;
//       animation: emergencyPulse 2s infinite ease-in-out;
//     }
    
//     @keyframes emergencyPulse {
//       0%, 100% { box-shadow: 0 0 5px rgba(220,38,38,0.5); }
//       50% { box-shadow: 0 0 20px rgba(220,38,38,0.8); }
//     }
    
//     .sensorValue {
//       display: block;
//       font-size: 1.1em;
//       font-weight: 700;
//       margin: 12px 0;
//       padding: 8px;
//       background: rgba(15,23,42,0.4);
//       border-radius: 5px;
//       border-left: 4px solid #34D399;
//       color: #ECFDF5;
//     }
    
//     .armed { color: #10B981; }
//     .disarmed { color: #EF4444; }
//     .emergency { color: #F59E0B; animation: blink 1s infinite; }
    
//     @keyframes blink {
//       0%, 50% { opacity: 1; }
//       51%, 100% { opacity: 0.3; }
//     }
    
//     h2 {
//       margin-top: 0;
//       margin-bottom: 20px;
//       font-weight: 700;
//       text-transform: uppercase;
//       letter-spacing: 2px;
//     }
    
//     .ledContrainer h2 { color: #E0F2FE; text-shadow: 0 0 15px #60A5FA; }
//     .imuContainer h2 { color: #ECFDF5; text-shadow: 0 0 15px #34D399; }
//     .throttleContainer h2 { color: #FEF2F2; text-shadow: 0 0 15px #F87171; }
    
//     #throttleValue, #brightnessValue { 
//       font-weight: bold; 
//       font-size: 1.2em;
//       text-shadow: 0 0 10px currentColor;
//     }
    
//     /* Responsive Design */
//     @media screen and (max-width: 1400px) {
//       .systemHeader h1 { font-size: 2.2em; }
//       .pidContainer { margin: 8px; padding: 18px; }
//       .pidInput { width: 70px; font-size: 12px; }
//       .topHalf { min-height: 45vh; }
//     }
    
//     @media screen and (max-width: 1200px) {
//       body { padding: 15px; }
//       .systemHeader h1 { font-size: 2em; }
//       .topHalf { gap: 15px; min-height: 40vh; }
//       .ledContrainer, .imuContainer, .statusPanelInline { padding: 20px; }
//       .pidContainer { margin: 6px; padding: 15px; }
//       .pidInput { width: 65px; }
//       h2 { font-size: 1.1em; }
//     }
    
//     @media screen and (max-width: 1000px) {
//       .topHalf { 
//         flex-direction: column; 
//         gap: 20px; 
//         min-height: auto;
//       }
//       .ledContrainer, .imuContainer, .statusPanelInline { 
//         width: 100%; 
//         min-height: auto;
//       }
//       .pidGrid { 
//         grid-template-columns: repeat(2, 1fr);
//         display: grid;
//       }
//       .pidContainer { 
//         margin: 10px 0; 
//         flex: none;
//       }
//     }
    
//     @media screen and (max-width: 768px) {
//       body { padding: 10px; }
//       .systemHeader { padding: 15px; }
//       .systemHeader h1 { font-size: 1.8em; letter-spacing: 2px; }
//       .pidGrid { 
//         grid-template-columns: 1fr;
//         gap: 10px;
//       }
//       .pidContainer { 
//         margin: 5px 0; 
//         padding: 15px;
//       }
//       .pidInput { width: 60px; font-size: 12px; }
//       .button { 
//         padding: 10px 20px; 
//         font-size: 12px;
//         min-width: 100px;
//       }
//       h2 { font-size: 1em; }
//       .sensorValue { font-size: 1em; }
//     }
    
//     @media screen and (max-width: 480px) {
//       body { padding: 8px; }
//       .systemHeader h1 { font-size: 1.5em; }
//       .ledContrainer, .imuContainer, .statusPanelInline { padding: 15px; }
//       .pidContainer { padding: 12px; }
//       .pidInput { width: 55px; font-size: 11px; }
//       .button { 
//         padding: 8px 16px; 
//         font-size: 11px;
//         margin: 4px;
//       }
//       .emergency-btn { font-size: 14px; padding: 12px; }
//     }
//   </style>
// </head>
// <body>
//   <div class="systemHeader">
//     <h1>FLIGHT CONTROL SYSTEM</h1>
//     <div>Status: <span id="droneStatus" class="disarmed">DISARMED</span></div>
//   </div>
  
//   <div class="topHalf">
//     <div class="ledContrainer">
//       <h2>FRONT LIGHTS</h2>
//       <div style="flex-grow: 1; display: flex; flex-direction: column; justify-content: space-between;">
//         <div class="slider-container">
//           <label>INTENSITY: <span id="brightnessValue">0</span>/255</label>
//           <input type="range" min="0" max="255" value="0" class="slider" id="brightnessSlider">
//         </div>
//         <div style="display: flex; flex-direction: column; gap: 10px; margin-top: 20px;">
//           <button class="button on" onclick="controlLED('on')">LIGHTS ON</button>
//           <button class="button off" onclick="controlLED('off')">LIGHTS OFF</button>
//         </div>
//       </div>
//     </div>

//     <div class="statusPanelInline">
//       <h2>AIRCRAFT STATUS</h2>
      
//       <div style="flex-grow: 1; display: flex; flex-direction: column; justify-content: space-between;">
//         <div>
//           <div class="batteryContainer">
//             <div class="batteryFrame">
//               <div class="batteryLevel high" id="batteryLevel" style="width: 85%;">
//                 <div class="batteryPercent" id="batteryPercent">85%</div>
//               </div>
//             </div>
//           </div>
          
//           <div class="statusItem">
//             <label>Flight Mode:</label>
//             <select id="flightMode" style="width: 100%; padding: 8px; border-radius: 5px; border: 1px solid #4682B4; background: rgba(15,23,42,0.8); color: #E0F2FE;">
//               <option value="manual">Manual</option>
//               <option value="stabilize" selected>Stabilize</option>
//               <option value="altitude">Altitude Hold</option>
//               <option value="gps">GPS Hold</option>
//               <option value="rtl">Return to Launch</option>
//             </select>
//           </div>
          
//           <div class="statusItem">
//             <label>Armed:</label>
//             <select id="armStatus" onchange="updateArmStatus()" style="width: 100%; padding: 8px; border-radius: 5px; border: 1px solid #4682B4; background: rgba(15,23,42,0.8); color: #E0F2FE;">
//               <option value="disarmed" selected>Disarmed</option>
//               <option value="armed">Armed</option>
//             </select>
//           </div>
//         </div>
        
//         <div>
//           <div class="slider-container" style="margin: 15px 0;">
//             <label style="font-weight: 700; color: #E0F2FE; font-size: 1.0em;">THROTTLE: <span id="throttleValue">0</span>%</label>
//             <input type="range" min="0" max="100" value="0" class="slider" id="throttleSlider">
//           </div>
          
//           <div style="margin-top: 25px; padding-top: 15px; border-top: 1px solid rgba(96,165,250,0.3);">
//             <button class="emergency-btn" onclick="emergencyStop()" style="padding: 15px 20px; font-size: 1.1em; font-weight: bold; border-radius: 8px; width: 100%;">EMERGENCY DISARM</button>
//           </div>
//         </div>
//       </div>
//     </div>

//     <div class="imuContainer">
//       <h2>FLIGHT TELEMETRY</h2>
//       <div style="flex-grow: 1; display: flex; flex-direction: column; justify-content: center; gap: 15px;">
//         <div class="sensorValue">ROLL: <span id="rollValue">0.0°</span></div>
//         <div class="sensorValue">PITCH: <span id="pitchValue">0.0°</span></div>
//         <div class="sensorValue">HEADING: <span id="headingValue">0.0°</span></div>
//       </div>
//     </div>
//   </div>

//   <div class="bottomHalf">
//     <h2 style="color: #FEF2F2; margin-bottom: 20px; font-family: 'Orbitron', monospace; font-weight: 900; font-size: 1.8em; text-shadow: 0 0 20px #DC2626; letter-spacing: 3px; text-align: center;">FLIGHT STABILITY CONTROL</h2>
//     <div class="pidGrid">
//       <div class="pidContainer">
//         <h3>ROLL AXIS CONTROL</h3>
//         <div>
//           <span class="pidLabel">P:</span>
//           <input type="number" class="pidInput" id="rollP" step="0.01" value="0.00" min="0" max="100">
//         </div>
//         <div>
//           <span class="pidLabel">I:</span>
//           <input type="number" class="pidInput" id="rollI" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <div>
//           <span class="pidLabel">D:</span>
//           <input type="number" class="pidInput" id="rollD" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <button class="button" onclick="updatePID('roll')" style="background: linear-gradient(45deg, #059669, #10B981); border-color: #059669; box-shadow: 0 5px 15px rgba(5,150,105,0.4); margin-top: 10px;">SYNC ROLL</button>
//       </div>

//       <div class="pidContainer">
//         <h3>PITCH AXIS CONTROL</h3>
//         <div>
//           <span class="pidLabel">P:</span>
//           <input type="number" class="pidInput" id="pitchP" step="0.01" value="0.00" min="0" max="100">
//         </div>
//         <div>
//           <span class="pidLabel">I:</span>
//           <input type="number" class="pidInput" id="pitchI" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <div>
//           <span class="pidLabel">D:</span>
//           <input type="number" class="pidInput" id="pitchD" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <button class="button" onclick="updatePID('pitch')" style="background: linear-gradient(45deg, #059669, #10B981); border-color: #059669; box-shadow: 0 5px 15px rgba(5,150,105,0.4); margin-top: 10px;">SYNC PITCH</button>
//       </div>

//       <div class="pidContainer">
//         <h3>YAW AXIS CONTROL</h3>
//         <div>
//           <span class="pidLabel">P:</span>
//           <input type="number" class="pidInput" id="yawP" step="0.01" value="0.00" min="0" max="100">
//         </div>
//         <div>
//           <span class="pidLabel">I:</span>
//           <input type="number" class="pidInput" id="yawI" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <div>
//           <span class="pidLabel">D:</span>
//           <input type="number" class="pidInput" id="yawD" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <button class="button" onclick="updatePID('yaw')" style="background: linear-gradient(45deg, #059669, #10B981); border-color: #059669; box-shadow: 0 5px 15px rgba(5,150,105,0.4); margin-top: 10px;">SYNC YAW</button>
//       </div>

//       <div class="pidContainer">
//         <h3>ALTITUDE CONTROL</h3>
//         <div>
//           <span class="pidLabel">P:</span>
//           <input type="number" class="pidInput" id="heightP" step="0.01" value="0.00" min="0" max="100">
//         </div>
//         <div>
//           <span class="pidLabel">I:</span>
//           <input type="number" class="pidInput" id="heightI" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <div>
//           <span class="pidLabel">D:</span>
//           <input type="number" class="pidInput" id="heightD" step="0.001" value="0.000" min="0" max="10">
//         </div>
//         <button class="button" onclick="updatePID('height')" style="background: linear-gradient(45deg, #059669, #10B981); border-color: #059669; box-shadow: 0 5px 15px rgba(5,150,105,0.4); margin-top: 10px;">SYNC ALTITUDE</button>
//       </div>
//     </div>
//   </div>

//   <script>
//     const brightnessSlider = document.getElementById('brightnessSlider');
//     const brightnessValue = document.getElementById('brightnessValue');
//     const throttleSlider = document.getElementById('throttleSlider');
//     const throttleValue = document.getElementById('throttleValue');
    
//     brightnessSlider.oninput = function() {
//       brightnessValue.innerHTML = this.value;
//       fetch(`/brightness?value=${this.value}`);
//     }
    
//     throttleSlider.oninput = function() {
//       throttleValue.innerHTML = this.value;
//       fetch(`/throttle?value=${this.value}`);
//     }
    
//     function controlLED(command) {
//       fetch(`/led?state=${command}`);
//     }
    
//     function setThrottle(value) {
//       throttleSlider.value = value;
//       throttleValue.innerHTML = value;
//       fetch(`/throttle?value=${value}`);
//     }
    
//     function updateSensorData() {
//       fetch('/api/sensors')
//         .then(response => response.json())
//         .then(data => {
//           document.getElementById('rollValue').innerHTML = data.roll.toFixed(1) + '°';
//           document.getElementById('pitchValue').innerHTML = data.pitch.toFixed(1) + '°';
//           document.getElementById('headingValue').innerHTML = data.heading.toFixed(1) + '°';
          
//           const statusElement = document.getElementById('droneStatus');
//           const armSelect = document.getElementById('armStatus');
          
//           if (data.emergency) {
//             statusElement.innerHTML = 'EMERGENCY STOP';
//             statusElement.className = 'emergency';
//             armSelect.value = 'disarmed';
//             armSelect.disabled = true;
//           } else if (data.armed) {
//             statusElement.innerHTML = 'ARMED';
//             statusElement.className = 'armed';
//             armSelect.disabled = false;
//           } else {
//             statusElement.innerHTML = 'DISARMED';
//             statusElement.className = 'disarmed';
//             armSelect.disabled = false;
//           }
          
//           throttleValue.innerHTML = data.throttle.toFixed(0);
//           throttleSlider.value = data.throttle;
          
//           // Update battery level
//           updateBatteryLevel(data.battery);
//         })
//         .catch(error => console.log('Error:', error));
//     }
    
//     function updateBatteryLevel(percentage) {
//       const batteryLevel = document.getElementById('batteryLevel');
//       const batteryPercent = document.getElementById('batteryPercent');
      
//       batteryLevel.style.width = percentage + '%';
//       batteryPercent.innerHTML = percentage.toFixed(0) + '%';
      
//       // Update battery color based on level
//       if (percentage <= 20) {
//         batteryLevel.className = 'batteryLevel low';
//       } else if (percentage <= 50) {
//         batteryLevel.className = 'batteryLevel medium';
//       } else {
//         batteryLevel.className = 'batteryLevel high';
//       }
//     }
    
//     function updateArmStatus() {
//       const armed = document.getElementById('armStatus').value;
//       if (armed === 'disarmed') {
//         setThrottle(0);
//       }
//       fetch(`/flight/arm?status=${armed}`)
//         .then(() => console.log('Arm status updated to:', armed))
//         .catch(error => console.log('Arm status update error:', error));
//     }
    
//     function emergencyStop() {
//       fetch('/flight/arm?status=disarmed')
//         .then(() => {
//           console.log('EMERGENCY DISARM EXECUTED!');
//           document.getElementById('armStatus').value = 'disarmed';
//           setThrottle(0);
//         })
//         .catch(error => console.log('Emergency disarm error:', error));
//     }
    
//     // PID update function
//     function updatePID(pidType) {
//       const kp = document.getElementById(pidType + 'P').value;
//       const ki = document.getElementById(pidType + 'I').value;
//       const kd = document.getElementById(pidType + 'D').value;
      
//       fetch(`/pid?type=${pidType}&kp=${kp}&ki=${ki}&kd=${kd}`)
//         .then(() => {
//           console.log(`${pidType} PID updated: P=${kp}, I=${ki}, D=${kd}`);
//           // Visual feedback
//           const button = event.target;
//           const originalText = button.innerHTML;
//           const originalColor = button.style.background;
//           button.innerHTML = 'Updated!';
//           button.style.background = 'linear-gradient(45deg, #059669, #10B981)';
//           setTimeout(() => {
//             button.innerHTML = originalText;
//             button.style.background = originalColor;
//           }, 1500);
//         })
//         .catch(error => console.log('Error updating PID:', error));
//     }
    
//     setInterval(updateSensorData, 100);
//     window.onload = updateSensorData;
//   </script>
// </body>
// </html>)=====";
// }
