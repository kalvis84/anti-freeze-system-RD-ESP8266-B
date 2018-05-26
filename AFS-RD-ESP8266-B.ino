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

#include <ESP8266WiFi.h>

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

// An IR detector/demodulator is connected to GPIO pin 14(D5 on a WeMos D1 mini board).
uint16_t RECV_PIN = 14;
IRrecv irrecv(RECV_PIN);
decode_results results;
uint16_t irLastCommand = 0; // last command received from remote controler

//float adc_factor = 1.0; //uncomment to calibrate. 
//    adc_factor = measured voltage on A0 / value from terminal
float adc_factor = 645.0 / 220.0; //comment for callibraton

uint16_t RELAY_PIN = 0;
uint16_t LEDBLUE_PIN  = 2;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Airbox-12BF";
char pass[] = "72392900";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "33f5bc4798794609bee02c0a0178975e";

WidgetLCD lcd(V0);
BlynkTimer timer; // Create a Timer object called "timer"! 

const long timerInterval = 100L; // in [ms]
int getTemperature = 0;
const uint8_t GET_TEMPERATURE_PERIOD = 10000/*ms*/ / timerInterval; // get number of main timer ticks.
int getIrCode = 0;
const uint8_t GET_IR_CODE_PERIOD = 500/*ms*/ / timerInterval; // get number of main timer ticks.
int syncBlynk = 0;
const uint8_t SYNC_BLYNK_PERIOD = 500/*ms*/ / timerInterval; // get number of main timer ticks.

void mainTimer(void);

void setup()
{
  // Debug console
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LEDBLUE_PIN, OUTPUT);

  irrecv.enableIRIn();  // Start the receiver
  Serial.printf("\nIRrecvDemo is now running and waiting for IR message on Pin %d", RECV_PIN);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(timerInterval, mainTimer); //  Here you set interval (1sec) and which function to call 
}

void loop()
{
  timer.run(); // BlynkTimer is working...
  
  //ToDo - hceck proper way to reduce power consumption (overheating module).
//  wifi_set_sleep_type(MODEM_SLEEP_T);
}

void mainTimer(void)
{
  if (irrecv.decode(&results)) {
    if(results.command > 0) irLastCommand = (uint16_t) results.command;
    serialPrintUint64(results.value, HEX); Serial.print(", ");
    Serial.print(results.address, DEC); Serial.print(", ");
    Serial.println(results.command, DEC);
    irrecv.resume();  // Receive the next value
  }
  // Check commandsfrom remote
  switch(irLastCommand){
    case PLAYPAUSE:
      Serial.print("PLAYPAUSE");
      if(digitalRead(RELAY_PIN)){
        digitalWrite(RELAY_PIN, 0);
//        Blynk.virtualWrite(V0, 0);
      }
      else{
        digitalWrite(RELAY_PIN, 1);
//        Blynk.virtualWrite(V0, 1);
      }
      break;
    case BACKW:
      Serial.print("BACKW");
      digitalWrite(LEDBLUE_PIN, 1);
//      Blynk.virtualWrite(V2, 1);
      break;
    case FORW:
      Serial.print("FORW");
      digitalWrite(LEDBLUE_PIN, 0);
//      Blynk.virtualWrite(V2, 0);
      break;
    default:
      break;
  }
  irLastCommand = 0;

//  ToDo - take avg of 10.
  int analogValue = analogRead(A0);
  float millivolts = (analogValue/1024.0) * adc_factor * 1000; //3300 is the voltage provided by NodeMCU
  float celsius = millivolts/10;
//  Serial.print("adc_factor=   ");
//  Serial.println(adc_factor);
//  Serial.print("milivolts=   ");
//  Serial.println(millivolts);
//  Serial.print("in DegreeC=   ");
//  Serial.println(celsius);

  //ToDo - synch only if changed
  if(syncBlynk != 0) syncBlynk--;
  else{
    Blynk.run();
    syncBlynk = SYNC_BLYNK_PERIOD;
    //update status LEDs in app
    if(digitalRead(RELAY_PIN)) Blynk.virtualWrite(V0, 1);
    else Blynk.virtualWrite(V0, 0);
    if(digitalRead(LEDBLUE_PIN)) Blynk.virtualWrite(V2, 1);  
    else  Blynk.virtualWrite(V2, 0);
    Blynk.virtualWrite(V5, celsius);
//    Serial.print("in DegreeC=   ");
//    Serial.println(celsius);
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
