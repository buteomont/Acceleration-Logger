/* Program to read the GY-521 MPU6050 6-axis Accelerometer Gyroscope Sensor
   and report the readings to the display, serial port, and/or MQTT broker. 
   ----------- Note ------------
   This project has both code and data (a web page) that needs to be uploaded
   to the processor. The code is uploaded normally, but to upload the web page
   open a terminal and enter "pio run --target uploadfs".
   */


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "AccelerationLogger.h"


MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


/* MPU6050 default I2C address is 0x68*/
//MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming 
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if there is no magnetometer present.

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  bool debug=true;
  bool displayenabled=true;    //enable the display
  bool invertdisplay=false;   //rotate display 180 degrees
  int sensitivity=1000;     //This is braking sensitivity. Lower is more sensitive.
  char ssid[SSID_SIZE] = "logger"; //connect to this SSID on your phone to configure
  char wifiPass[PASSWORD_SIZE] = "12345678";
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String lastMessage=""; //contains the last message sent to display. Sometimes need to reshow it
boolean rssiShowing=false; //used to redraw the RSSI indicator after clearing display


/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
// Quaternion q;           // [w, x, y, z]         Quaternion container
// VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
// VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            Gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

char dispbuf[32];

/*
*  Initialize the settings from eeprom. No need to determine if they are valid
*  in this project.
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  }



/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }



void show(String msg)
  {
  display.clearDisplay(); // clear the buffer regardless

  if (settings.displayenabled)
    {
    lastMessage=msg; //in case we need to redraw it

    display.setCursor(0, 0);  // Top-left corner

    if (msg.length()>20)
      {
      display.setTextSize(1);      // tiny text
      }
    else if (msg.length()>7 || rssiShowing) //make room for rssi indicator
      {
      display.setTextSize(2);      // small text
      }
    else
      {
      display.setTextSize(3);      // Normal 1:1 pixel scale
      }
    display.println(msg);
    }
  display.display(); // move the buffer contents to the OLED
  }

// void show(uint16_t val, String suffix)
//   {
//   String msg=String(val)+suffix;
//   show(msg);
//   }

void show(String msg, bool override)
  {
  bool oldVal=settings.displayenabled; //Save the disabled flag
  settings.displayenabled=true;       //Override it
  show(msg);
  settings.displayenabled=oldVal;     //Put it back
  }



void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
  {
  switch (type)
    {
    case WStype_CONNECTED:
      {
      IPAddress clientIP = webSocket.remoteIP(num);
      show("Conn from\n"+clientIP.toString());

      // Send sensitivity value
      DynamicJsonDocument doc(256);
      doc["type"] = "initialize";
      doc["key"] = "sensitivity";
      doc["value"] = map(settings.sensitivity, 0, 5000, 100, 0); // Map to slider range
      String message;
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);
      
      //send display enabled status
      doc["key"] = "displayEnabled";
      doc["value"] = settings.displayenabled;// 1=true, 0=false or use boolean values
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);
      
      //send display inverted status
      doc["key"] = "displayInverted";
      doc["value"] = settings.invertdisplay;// 1=true, 0=false or use boolean values
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);
     
      //send WiFi SSID
      doc["key"] = "WiFiSSID";
      doc["value"] = settings.ssid;
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);
     
      //send WiFi password
      doc["key"] = "WiFiPass";
      doc["value"] = settings.wifiPass;
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);

      break;
      }
    case WStype_TEXT:
      {
      //Serial.printf("[%u] Received: %s\n", num, payload);
      String message = String((char *)payload);

      DynamicJsonDocument doc(256);
      deserializeJson(doc, message);

      String type = doc["type"];
      if (type == "control")
        {
        String key = doc["key"];
        int value = doc["value"];
        if (key == "sensitivity")
          {
          settings.sensitivity = map(value,0,100,5000,0);
          saveSettings();
          }
        else if (key == "displayEnabled")
          {
          // value should be 1 (checked) or 0 (unchecked)
          settings.displayenabled = (value != 0);
          saveSettings();
          }
        else if (key == "displayInverted")
          {
          // value should be 1 (checked) or 0 (unchecked)
          settings.invertdisplay = (value != 0);
          saveSettings();
          display.setRotation(settings.invertdisplay?2:0); //make it look right
          }
        else if (key == "WiFiSSID")
          {
          strcpy(settings.ssid, doc["value"]);
          saveSettings();
          }
        else if (key == "WiFiPass")
          {
          strcpy(settings.wifiPass, doc["value"]);
          saveSettings();
          }

        }
      break;
      }
    case WStype_DISCONNECTED:
      {
      show("Disc");
      break;
      }
    }
  }



void sendUpdates()
  {
  DynamicJsonDocument doc(128);

  doc["type"] = "update";
  doc["key"] = "accelx";
  doc["value"] = ax;
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  doc["key"] = "accely";
  doc["value"] = ay;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
 
  doc["key"] = "accelz";
  doc["value"] = az;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  doc["key"] = "gyrox";
  doc["value"] = gx;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  doc["key"] = "gyroy";
  doc["value"] = gy;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
 
  doc["key"] = "gyroz";
  doc["value"] = gz;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
  }

  void initDisplay()
  {
  bool displayOk=display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if(!displayOk) 
    {
    Serial.println(F("SSD1306 allocation failed"));
    delay(5000);
    ESP.reset();  //try again
    }
  else  
    {
    display.setRotation(settings.invertdisplay?2:0); //make it look right
    display.clearDisplay();       //no initial logo
    display.setTextSize(3);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    }

  if (settings.debug)
    show("Init");
  else
    show("Display OK");
  }
  



void setup() 
  {
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash

  loadSettings();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  initDisplay();
  accelgyro.initialize();

  show(accelgyro.testConnection() ? "MPU6050 OK" : "MPU6050 Fail",true);

  // set up the on-board brake light simulation LED
  pinMode(BRAKE_LED_PORT,OUTPUT);
  digitalWrite(BRAKE_LED_PORT,BRAKE_OFF);  //turn it off

  // Initialize LittleFS
  if (!LittleFS.begin())
    {
    show("LittleFS\nFailed",true);
    return;
    }
  else   
    show("LittleFS\nOK");

  // Open the WiFi
  show("WiFi Start");
  delay(1000);
  WiFi.softAP(settings.ssid, settings.wifiPass);
  
  char buff[6+SSID_SIZE+6+PASSWORD_SIZE+18]; //"SSID: xxxxxxx\nPASS: xxxxxx\n192.168.4.1"
  sprintf(buff,"SSID: %s\nPass: %s\n%s",
          settings.ssid,
          settings.wifiPass,
          WiFi.softAPIP().toString().c_str());
  show(buff,true);
  delay(5000);

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);

  server.serveStatic("/", LittleFS, "/index.html");
  server.begin();
  show("init done");
  }

void loop() 
  {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 250; // interval in milliseconds

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
    {
    previousMillis = currentMillis;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sprintf(dispbuf,"X%05d Y%05d Z%05d\nX%05d Y%05d Z%05d",ax,ay,az,gx,gy,gz);
    show(String(dispbuf));
    digitalWrite(BRAKE_LED_PORT, ay<(-settings.sensitivity));
    }

  webSocket.loop(); // Keep WebSocket server running
  server.handleClient();

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 500)
    {
    sendUpdates();
    lastUpdate = millis();
    }
  }