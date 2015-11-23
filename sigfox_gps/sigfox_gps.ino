/*
 * Example code showing of to send any datatype - 12 bytes max - (in this example, a struct of
 *   GPS coordinate) to SigFox network with a SmartEverything prototyping board
 *   (http://www.smarteverything.it/).
 * 
 * - GPS frame decoding is made with TinyGPS library (https://github.com/mikalhart/TinyGPS).
 * - SigFox modem communication is done through direct AT commands. This code is minimalist :
 *     it doesn't manage downlink messages, and it's error management is sort of non existant.
 * 
 * Why don't I use official SmartEverything libraries (https://github.com/ameltech/) ?
 * 
 *  - GPS official library is sometimes unable to parse NMEA frames. It can read altitude,
 *      number of locked satellites, but sometimes lacks latitude and longitude.
 *  - SigFox official library can only send to SigFox network an array of char. If it's OK 
 *      to send "Hello" messages, it's rather limited when you want to send any datatype.
 *      Moreover, this library fails to send message smaller than three caracters
 *      
 * Official repository for this code : https://github.com/aboudou/SmartEverything_SigFox_GPS
 * 
 * Feel free to clone it, modify it, improve it… The following code is licensed under the BSD 2-Clauses License.
 * 
 * Copyright (c) 2015, Arnaud Boudou
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list 
 * of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list 
 * of conditions and the following disclaimer in the documentation and/or other materials 
 * provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Wire.h>
#include <Arduino.h>
#include "TinyGPS.h"

byte led_status = HIGH;
unsigned long previousSendTime = 0;
byte firstCall = 1;

// GPS coordinate structure, 12 bytes size on 32 bits platforms
struct gpscoord {
  float a_latitude;  // 4 bytes
  float a_longitude; // 4 bytes
  int   a_altitude;  // 4 bytes
};

TinyGPS gps;

float latitude  = 0.0f;
float longitude = 0.0f;
int   altitude  = 0;

// Set to 1 to get debug messages to console
// If you set if to 1, you'll have to connect to the serial console to allow the program to run
#define DEBUG 1

void setup() {

  // Init UART devices
  if (DEBUG) {
    SerialUSB.begin(115200);
  }
  GPS.begin(9600);
  SigFox.begin(19200);

  if (DEBUG) {
    // Wait for the user to connect to serial console
    while (!SerialUSB) {}
  }

  // Switch SigFox modem to command mode
  sigfoxCommandMode();
}


void loop() {
  bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS.available())
    {
      char c = GPS.read();
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData) {
    // Got GPS data
    led_status = LOW;
    ledBlueLight(led_status);

    // Check if we need to send data to SigFox network :
    //   - First call of "loop" method
    byte canSendData = 0;
    if (firstCall) {
      canSendData = 1;
      firstCall = 0;
    }
    //   - The last data send is 10+ minutes old
    if(!canSendData) {
      unsigned long diff = millis() - previousSendTime;
      if (diff >= 10*60*1000) { // 10 minutes delay min
        canSendData = 1;
      }
    }
    
    if (canSendData) {
      ledGreenLight(LOW);
      ledRedLight(LOW);

      // Get GPS coordinates
      unsigned long age;
      gps.f_get_position(&latitude, &longitude, &age);
      altitude = gps.altitude() / 100;

      if (DEBUG) {
        SerialUSB.println(latitude, 6);
        SerialUSB.println(longitude, 6);
        SerialUSB.println(altitude);
      }

      // Store coordinates into dedicated structure
      gpscoord coords = {latitude, longitude, altitude};

      // Save time of sending action.
      previousSendTime = millis();

      // Send data
      if (DEBUG) {
        SerialUSB.println("Sending data");
      }
      bool answer = sigfoxSend(&coords, sizeof(gpscoord));

      // Light LED depending on modem answer
      if (answer) {
        ledGreenLight(HIGH);
        ledRedLight(LOW);
      } else {
        ledGreenLight(LOW);
        ledRedLight(HIGH);
      }
    }
  } else {
    // Locking GPS position
    SerialUSB.println("Locking GPS position...");
    led_status = !led_status;
  }

  ledBlueLight(led_status);
}


// Switch SigFox modem to command mode
void sigfoxCommandMode() {
  SigFox.print("+++");

  // Waiting for modem answer
  while (!SigFox.available()) {
    delay(100);
  }
  while (SigFox.available()) {
    char answer = (char)SigFox.read();
    if (DEBUG) {
      SerialUSB.print(answer);
    }
  }

  if (DEBUG) {
    SerialUSB.println("");
  }
}

/* Send any datatype through AT commands. */
bool sigfoxSend(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;

  if (DEBUG) {
    SerialUSB.println("Issuing AT command");
  }
  
  SigFox.print('A');
  SigFox.print('T');
  SigFox.print('$');
  SigFox.print('S');
  SigFox.print('F');
  SigFox.print('=');
  //0-1 == 255 --> (0-1) > len
  for(uint8_t i = len-1; i < len; --i) {
    if (bytes[i] < 16) {SigFox.print("0");}
    SigFox.print(bytes[i], HEX);
  }
  SigFox.print('\r');

  if (DEBUG) {
    SerialUSB.print('A');
    SerialUSB.print('T');
    SerialUSB.print('$');
    SerialUSB.print('S');
    SerialUSB.print('F');
    SerialUSB.print('=');
    for(uint8_t i = len-1; i < len; --i) {
      if (bytes[i] < 16) {SerialUSB.print("0");}
      SerialUSB.print(bytes[i], HEX);
    }
    SerialUSB.print('\r');
    SerialUSB.println("");
  }

  bool error = false;
  // Waiting for modem answer
  while (!SigFox.available()) {
    delay(100);
  }
  bool firstChar = true;
  while (SigFox.available()) {
    char answer = (char)SigFox.read();
    if (DEBUG) {
      SerialUSB.print(answer);
    }
    if (firstChar) {
      firstChar = false;
      if (answer == 'O') { // "OK" message
        error = false; 
      } else { // "ERROR" message 
        error = true;
      }
    }
  }
  if (DEBUG) {
    SerialUSB.println("");
  }

  return !error;
}

