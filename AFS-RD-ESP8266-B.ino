/*
  AFS-RD-ESP8266-B
  Anti Freeze System - Relay Device. Based on ESP8266 module and Blynk service.
  Project is made for Summer Houses wich are left without hitting during winter.
  With the help of this system it is possible to control many electric heaters in house
  via Internet. It is possible to set MIN/MAX temp and (eg 2-4 Celsius) to prevent water in
  main heating system to frie and damage it. 
  Additionaly, temperature can be changed to bigger value if someone want to visit house during
  winter season (heating house after arrival can take several hours).
  
  Programing Options
  * Board WEMOS D1 R2 & mini
  * 1M SPIFS
  
  WeMos D1 mini Pinout
  https://github.com/esp8266/Arduino/blob/master/variants/d1_mini/pins_arduino.h> 

  Project Pinout
  | Arduino |ESP GPIO | Some other hardware | BLYNK Virtual PIN |
  |---------|---------|---------------------|-------------------|
  | D3      | GPIO0   | Relay               | V0                |
  | D4      | GPIO2   | LED_BUILTIN         |                   |
  | D5      | GPIO14  | IR receiver         |                   |
  |       D0 configured for DEEP_SLEEP      |                   |
  |               AND HW RESET.             |                   |
  | D0      | GPIO16  | 470 Ohm to HW RESET |                   |
  | D7      | GPIO13  | DS18B20             | V5                |
  | Vcc     | Vcc     | Vcc                 |                   |
  | GND     | GND     | GND                 |                   |
  |         |         | TempMIN             | V3                |
  |         |         | TempMAX             | V4                |
  |---------|---------|---------------------|-------------------|

  Parameters Flow
  |    TRANSMITER    |   DIR    |   SERVER    | DIR |      RECEIVER      |
  |------------------|----------|-------------|-----|--------------------|
  |WeMos virtualWrite|(PUSH) => | V0, V3, V4  | =>  | APP                |
  |               APP|(PUSH) => | V0, V3, V4  | =>  | BLYNK_WRITE  WeMos |
  |WeMos virtualWrite|(PUSH) => |     V5      | =>  | APP                |
  |               APP|(PUSH) => |    V127     | =>  | BLYNK_WRITE  WeMos |
  |------------------|----------|-------------|-----|--------------------|

  Remote controler
  PLAYPAUSE => ON/OFF relay
  
  Created 24 May 2018
  By Marcin Postek
  https://github.com/kalvis84/anti-freeze-system-RD-ESP8266-B
  Change log
  * version 0.2 2018-05-30
    + full functionality.
    + blue led connected in fw with relay pin.
    + min max values for temp.
    - remove blue led control by remote.
  * version 0.1 2018-05-30
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
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//needed for WifiManager library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <BlynkSimpleEsp8266.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// ToDo - sleep scenarious have to be cheched. This configuration does not work
// LIGHT_SLEEP_T stops CPU and MODEM_SLEEP_T don't lover current consumption.
// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}

#include "ir_arkas_remote.h"

/**********************************DS18B20******************/
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 13

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
// DS18B20 water proof = { 0x28, 0xFF, 0x97, 0x59, 0x74, 0x15, 0x3, 0x65 }
//DeviceAddress insideThermometer = { 0x28, 0xFF, 0x97, 0x59, 0x74, 0x15, 0x3, 0x65 };
DeviceAddress insideThermometer;
/**********************************END DS18B20******************/

//LED_BLUE is pulled up to VCC. O = ON.
#define LED_BLUE_OFF 1
#define LED_BLUE_ON 0

uint16_t RELAY_PIN = 0;  //GPIO0
uint16_t LEDBLUE_PIN  = 2; //GPIO2
uint16_t RECV_PIN = 14;

// An IR detector/demodulator is connected to GPIO pin 14(D5 on a WeMos D1 mini board).
IRrecv irrecv(RECV_PIN);

//Blynk widgets
WidgetLCD lcd(V0);
BlynkTimer timer; // Create a Timer object called "timer"! 

//Default values for WifiManager additioanl parameters.
//define your default values here, if there are different values in config.json, they are overwritten.
//char mqtt_server[40];
//char mqtt_port[6] = "8080";
char temp_min[5] = "11.1";
char temp_max[5] = "22.2";
char blynk_token[34] = "YOUR_BLYNK_TOKEN";

/*******************MQTT**********************/
#define MQTT_SERVER      "192.168.0.10"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
#define MQTT_USERNAME    "admin"
#define MQTT_PASSWORD    "admin"

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

// Setup a topic called 'sh/ac' for publishing.
Adafruit_MQTT_Publish aircondition = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "/sh/ac");

// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

/*******************END MQTT************************/

float celsius = 0;
int celsiusOutOfRangeCount = 0; 
float tempMin = 25;
float tempMax = 30;

//flag for MQTT publishing
bool doMQTTPublish = false;

//flag for saving data
bool shouldSaveConfig = false;
bool shouldUpdateVirtualsOnBlynk = false;

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
void mqttRun(void);

//connect to MQTT server.
void MQTT_connect(void);

// read config file from SPIFFS. 
void readConfig(void); 
// save config file to SPIFFS. 
void saveConfig(void);
//callback notifying us of the need to save config
void saveConfigCallback (void);
void CredentialsReset(void);
void HardwareResetESP(void);
void connectWifi(void);
//blink LED tha way it is defined by: count, ton and toff. 
void blinkLEDBuildin(int count, int ton, int toff);
void printAddress(DeviceAddress deviceAddress);

void setup()
{
  // Debug console
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LEDBLUE_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, 0);
  digitalWrite(LEDBLUE_PIN, LED_BLUE_OFF);
    
  readConfig();
  connectWifi();
    
  irrecv.enableIRIn();  // Start the receiver
  Serial.printf("\nIRrecvDemo is now running and waiting for IR message on Pin %d", RECV_PIN);

  Blynk.config(blynk_token);
  if(!Blynk.connect()) {
    Serial.println("Blynk connection timed out.");
  } 
  timer.setInterval(200, irTimer); //  timer for IR routine. 
  timer.setInterval(5000, blynkTimer);
  timer.setInterval(5000, tempTimer);

//*****************DS18B20 Setup******************************************
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  int deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

//  if(insideThermometer == 0){
    if (!sensors.getAddress(insideThermometer, 0)){
      Serial.println("Unable to find address for Device 0");
    }
//  }  

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    // 9 mean 0.5°C resolution 
  sensors.setResolution(insideThermometer, 9);
  Serial.print("Device 0 Resolution: ");
  int resolution = sensors.getResolution(insideThermometer);
  Serial.print(resolution, DEC);
  Serial.println();
//*****************END - DS18B20 Setup******************************************

  /* Setting sleep type to MODEM_SLEEP. This reduce power consumption
   * 3-4 times and temperature off ESP8266 module.
   * Module goes to sleep after yield() (it is in delay();
   */ 
  Serial.println("ModemSleep");
  wifi_set_sleep_type(MODEM_SLEEP_T);
  Serial.println("delay 5s");
  delay(5000);
}

void loop()
{
  timer.run(); // BlynkTimer is working...

  if (shouldSaveConfig) {
    shouldSaveConfig = false;
    saveConfig();
  }
  /* if something wron, reset credential and go back to WiFiManager HTTPserver.*/
  if(doCredentialsReset) { doCredentialsReset = false; CredentialsReset();}

  //Main routines. Frequency configured in setup.
  if(doIrRun) { doIrRun = false; irRun();}
  if(doTempRun) { doTempRun = false; tempRun();}
  if(doBlynkRun) { doBlynkRun = false; blynkRun();}
  if(doMQTTPublish) {doMQTTPublish = false; mqttRun();}
  
//  Serial.println("delay 5s");
  delay(100);
  //ToDo - check proper way to reduce power consumption (overheating module).
  //wifi_set_sleep_type(MODEM_SLEEP_T);
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
          digitalWrite(LEDBLUE_PIN, LED_BLUE_OFF);
        }
        else{
          digitalWrite(RELAY_PIN, 1);
          digitalWrite(LEDBLUE_PIN, LED_BLUE_ON);
        }
        break;
      default:
//        if(irLastCommand > 0) blinkLEDBuildin(irLastCommand/20, 300, 300); //ToDo - remove. Only for blinking test.
        break;
    }
    irLastCommand = 0;
  }
}

/**************************DS18B20 print temp****************************/
void tempRun(void){
  static uint8_t timeToPublishMQTT = 10;
  //  ToDo - take avg of 10.

  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  celsius = sensors.getTempC(insideThermometer);
  // check if value is correct. < -50mean there is no sensor or broken.
  if(celsius < -50) {
    celsius = 0;
    //if 10 broken values than reset.
    if(++celsiusOutOfRangeCount == 10){
      blinkLEDBuildin(4, 300, 300);
      HardwareResetESP();
    }
  }else celsiusOutOfRangeCount = 0;

  Serial.print("Temp C: ");
  Serial.println(celsius);
  
  //do temperature regulation.
  if(celsius > tempMax) {
    digitalWrite(RELAY_PIN, 0);
    digitalWrite(LEDBLUE_PIN, LED_BLUE_OFF);
  }
  else if(celsius < tempMin) {
    digitalWrite(RELAY_PIN, 1);
    digitalWrite(LEDBLUE_PIN, LED_BLUE_ON);
  }

  //Publish new temperature value
  if(timeToPublishMQTT) --timeToPublishMQTT;
  else { timeToPublishMQTT = 10; doMQTTPublish = true; }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
/**************************************************************************/

void blynkRun(void)
{
  Blynk.run();
  //ToDo - only if changed
  //update status LEDs in app
  if(digitalRead(RELAY_PIN)) Blynk.virtualWrite(V0, 1);
  else Blynk.virtualWrite(V0, 0);
  Blynk.virtualWrite(V5, celsius);
  if(shouldUpdateVirtualsOnBlynk){
    shouldUpdateVirtualsOnBlynk = false;
    Blynk.virtualWrite(V3, tempMin);
    Blynk.virtualWrite(V4, tempMax);
  }
}

void mqttRun(){
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  char strBuf [128];

  MQTT_connect();

  // Now we can publish stuff!
  Serial.print(F("\nSending temperature val "));
  Serial.print(celsius);
  Serial.print("...");

  sprintf(strBuf, "[{\"temperature\": %2.2lf},{\"sensor_id\": \"AFS-1\", \"location\": \"bedroom\"}]", celsius);
  if (! aircondition.publish(strBuf)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         blinkLEDBuildin(2, 300, 300);
         //reset and try again, or maybe put it to deep sleep
         HardwareResetESP;
       }
  }
  Serial.println("MQTT Connected!");
}

void readConfig(void){
    //clean FS, for testing
//  SPIFFS.format();

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

          strcpy(temp_min, json["temp_min"]);
          strcpy(temp_max, json["temp_max"]);
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
  WiFiManagerParameter custom_temp_min("tmin", "temp min", temp_min, 5);
  WiFiManagerParameter custom_temp_max("tmax", "temp max", temp_max, 5);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 34);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
 // wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_temp_min);
  wifiManager.addParameter(&custom_temp_max);
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
    HardwareResetESP();
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(temp_min, custom_temp_min.getValue());
  strcpy(temp_max, custom_temp_max.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  tempMin = strtod(temp_min, (char **)0);
  tempMax = strtod(temp_max, (char **)0);
  //ToDo - check if temp_min can have less letters than configured (5).
  // once json was not saved when 14 and 25 values are put on configuration site.
  shouldUpdateVirtualsOnBlynk = true;
}

//callback from WiFiManager - notifying us of the need to save config.
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveConfig(void){
  char buff[5];
  //save the custom parameters to FS
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["temp_min"] = dtostrf(tempMin, 2, 1, buff);
  json["temp_max"] = dtostrf(tempMax, 2, 1, buff);;
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
    // Hardware reset.
    HardwareResetESP();
}

/*
 * HardwareResetESP. Require 470 Ohm from GPIO16 to HW RESET.
 * It works evan after flashing (known issue in ESP).
 */
void HardwareResetESP(void)
{
    // Hardware reset.
    digitalWrite(16, 0);  //GPIO16
    pinMode(16, OUTPUT); //GPIO16  
//    ESP.restart(); //software reset do not work correctly after flashing. #1017
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
  digitalWrite(LEDBLUE_PIN, !pinValue);
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 3
BLYNK_WRITE(V3)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V3 to a variable
  if(pinValue != tempMin){
    tempMin = pinValue;
    shouldSaveConfig = true;
    Serial.println("tempMin updated.");
  }
  Serial.print("V3 is: ");
  Serial.println(pinValue);
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 4
BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V4 to a variable
  if(pinValue != tempMax){
    tempMax = pinValue;
    shouldSaveConfig = true;
    Serial.println("tempMax updated.");
  }
  Serial.print("V4 is: ");
  Serial.println(pinValue);
}

//CREDENTIAL RESET
//hidden button in application. for test purpose.
BLYNK_WRITE(V127)
{
  doCredentialsReset = true;
}
