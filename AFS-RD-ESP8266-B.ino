/*
  AFS-RD-ESP8266-B
  Anti Freeze System - Relay Device. Based on ESP8266 module and Blynk service.
  Project is made for Summer Houses wich are left without hitting during winter.
  With the help of this system it is possible to control many electric heaters in house
  via Internet. It is possible to set MIN/MAX temp and (eg 2-4 Celsius) to prevent water in
  main heating system to frie and damage it. 
  Additionaly, temperature can be changed to bigger value if someone want to visit house during
  winter season (heating house after arrival can take several hours).
  
  WeMos D1 mini Pinout
  https://github.com/esp8266/Arduino/blob/master/variants/d1_mini/pins_arduino.h> 

  Project Pinout
  | Arduino |ESP GPIO | Some other hardware |
  |---------|---------|---------------------|
  | D3      | GPIO0   | Relay               |
  | D4      | GPIO2   | LED_BUILTIN         |
  | D5      | GPIO14  | IR receiver         |
  | A0      | A0      | LM35 temp sens      |
  | Vcc     | Vcc     | Vcc                 |
  | GND     | GND     | GND                 |
  
  Created 24 May 2018
  By Marcin Postek
  https://github.com/kalvis84/anti-freeze-system-RD-ESP8266-B
  Change log
  * version 0.1
    + initial release
  License
  MIT License
  
  Copyright (c) 2018 kalvis84
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>

//needed for WifiManager library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <BlynkSimpleEsp8266.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

//LED_BLUE is pulled up to VCC. O = ON.
#define LED_BLUE_OFF 1
#define LED_BLUE_ON 0

// ToDo - sleep scenarious have to be cheched. This configuration does not work
// LIGHT_SLEEP_T stops CPU and MODEM_SLEEP_T don't lover current consumption.
// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}

#include "ir_arkas_remote.h"

// An IR detector/demodulator is connected to GPIO pin 14(D5 on a WeMos D1 mini board).
uint16_t RECV_PIN = 14;
IRrecv irrecv(RECV_PIN);

//float adc_factor = 1.0; //uncomment to calibrate. 
//    adc_factor = measured voltage on A0 / value from terminal
float adc_factor = 645.0 / 220.0; //comment for callibraton
float celsius = 0;

uint16_t RELAY_PIN = 0;  //GPIO0
uint16_t LEDBLUE_PIN  = 2; //GPIO2

//define your default values here, if there are different values in config.json, they are overwritten.
//char mqtt_server[40];
//char mqtt_port[6] = "8080";
char blynk_token[34] = "YOUR_BLYNK_TOKEN";

//flag for saving data
bool shouldSaveConfig = false;

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Airbox-12BF";
//char pass[] = "72392900";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
//char auth[] = "33f5bc4798794609bee02c0a0178975e";

WidgetLCD lcd(V0);
BlynkTimer timer; // Create a Timer object called "timer"! 

//flags for timmers
bool doCredentialsReset = false;
bool doTempRun = true;  //first get temperature to prevent sending zeros;
bool doIrRun = false;
bool doBlynkRun = false;

//timmers routines
//functions only for set flags
inline void irTimer(void){ doIrRun = true;}
inline void blynkTimer(void){ doBlynkRun = true;}
inline void tempTimer(void){ doTempRun = true;}

void irRun(void);
void blynkRun(void);
void tempRun(void);

// read config file from SPIFFS. 
void readConfig(void); 
// save config file to SPIFFS. 
void saveConfig(void);
//callback notifying us of the need to save config
void saveConfigCallback (void);
void CredentialsReset(void);
void connectWifi(void);
//blink LED tha way it is defined by: count, ton and toff. 
void blinkLEDBuildin(int count, int ton, int toff);

void setup()
{
  // Debug console
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LEDBLUE_PIN, OUTPUT);
  
  readConfig();

  connectWifi();

  if (shouldSaveConfig) {
    saveConfig();
  }
    
  irrecv.enableIRIn();  // Start the receiver
  Serial.printf("\nIRrecvDemo is now running and waiting for IR message on Pin %d", RECV_PIN);

  Blynk.config(blynk_token);
  if(!Blynk.connect()) {
    Serial.println("Blynk connection timed out.");
  } 
  timer.setInterval(200, irTimer); //  timer for IR routine. 
  timer.setInterval(500, blynkTimer);
  timer.setInterval(1000, tempTimer);
}

void loop()
{
  timer.run(); // BlynkTimer is working...

  /* if something wron, reset credential and go back to WiFiManager HTTPserver.*/
  if(doCredentialsReset) { doCredentialsReset = false; CredentialsReset();}

  if(doIrRun) { doIrRun = false; irRun();}
  if(doTempRun) { doTempRun = false; tempRun();}
  if(doBlynkRun) { doBlynkRun = false; blynkRun();}
  //ToDo - check proper way to reduce power consumption (overheating module).
//  wifi_set_sleep_type(MODEM_SLEEP_T);
}

void irRun(void)
{
  decode_results results;
  uint16_t irLastCommand = 0; // last command received from remote controler
  static int zeroButtonHits = 0; // 10 times in a row cause reset of WiFi Credentials.

  //decode IR code
  if (irrecv.decode(&results)) {
    if(results.command > 0){
      irLastCommand = (uint16_t) results.command;
      //count ZORO button - 10 times -resets credentials.
      if(irLastCommand == ZERO){
        if(++zeroButtonHits == 10)   doCredentialsReset = true;
      }else zeroButtonHits = 0;
    }
    serialPrintUint64(results.value, HEX); Serial.print(", ");
    Serial.print(results.address, DEC); Serial.print(", ");
    Serial.println(results.command, DEC);
    irrecv.resume();  // Receive the next value
  }

  // do whatever needs to be done after pressing right button.
  if(irLastCommand > 0){
    // Check commandsfrom remote
    switch(irLastCommand){
      case PLAYPAUSE:
        Serial.print("PLAYPAUSE");
        if(digitalRead(RELAY_PIN)){
          digitalWrite(RELAY_PIN, 0);
        }
        else{
          digitalWrite(RELAY_PIN, 1);
        }
        break;
      case BACKW:
        Serial.print("BACKW");
        digitalWrite(LEDBLUE_PIN, 1);
        break;
      case FORW:
        Serial.print("FORW");
        digitalWrite(LEDBLUE_PIN, 0);
        break;
      default:
//        if(irLastCommand > 0) blinkLEDBuildin(irLastCommand/20, 300, 300); //ToDo - remove. Only for blinking test.
        break;
    }
    irLastCommand = 0;
  }
}

void tempRun(void){
  int analogValue = analogRead(A0);
  float millivolts = (analogValue/1024.0) * adc_factor * 1000; //3300 is the voltage provided by NodeMCU
  //  ToDo - take avg of 10.
  celsius = millivolts/10;
  Serial.print("in DegreeC=   ");
  Serial.println(celsius);
  //ToDo - add adc_factor to FS routine?
  if(adc_factor == 1.0){
    Serial.print("milivolts=   ");
    Serial.println(millivolts);
  }
}

void blynkRun(void)
{
  Blynk.run();
  //update status LEDs in app
  if(digitalRead(RELAY_PIN)) Blynk.virtualWrite(V0, 1);
  else Blynk.virtualWrite(V0, 0);
  if(digitalRead(LEDBLUE_PIN)) Blynk.virtualWrite(V2, 1);  
  else  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V5, celsius);
}

void readConfig(void){
    //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

//          strcpy(mqtt_server, json["mqtt_server"]);
//          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(blynk_token, json["blynk_token"]);

        } else {
          Serial.println("failed to load json config");
        }
      }else{
        Serial.println("failed to open config file");
      }
    }else{
      Serial.println("/config.json not exist. Must be created via WiFiManager site.");
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}

void connectWifi(){
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
//  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
//  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 34);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
 // wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
//  wifiManager.addParameter(&custom_mqtt_server);
//  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_blynk_token);

  //reset settings - for testing
//  wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    blinkLEDBuildin(3, 300, 300);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
//  strcpy(mqtt_server, custom_mqtt_server.getValue());
//  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());
}

//callback from WiFiManager - notifying us of the need to save config.
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveConfig(void){
  //save the custom parameters to FS
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
//  json["mqtt_server"] = mqtt_server;
//  json["mqtt_port"] = mqtt_port;
  json["blynk_token"] = blynk_token;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  //end save
}

void CredentialsReset(void)
{
    Serial.println("Reset credentials!");
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    blinkLEDBuildin(5, 300, 300);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
}

//blink LED tha way it is defined by: count, ton and toff. 
// time in ms.
void blinkLEDBuildin(int count, int ton, int toff)
{
  Serial.println("Start blinking...");
  digitalWrite(LEDBLUE_PIN, LED_BLUE_OFF);
  delay(1000); //initial delay.
  for(int i = 0; i < count; ++i){
    digitalWrite(LEDBLUE_PIN, LED_BLUE_ON);
    delay(ton);
    digitalWrite(LEDBLUE_PIN, LED_BLUE_OFF);
    delay(toff);
  }
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 0
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  // You can also use:
  // String i = param.asStr();
  // double d = param.asDouble();
  Serial.print("V0 is: ");
  Serial.println(pinValue);
  digitalWrite(RELAY_PIN, pinValue);
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 2
BLYNK_WRITE(V2)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  // You can also use:
  // String i = param.asStr();
  // double d = param.asDouble();
  Serial.print("V2 is: ");
  Serial.println(pinValue);
  digitalWrite(LEDBLUE_PIN, pinValue);
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 127
//CREDENTIAL RESET
BLYNK_WRITE(V127)
{
  doCredentialsReset = true;
}
