// SPDX-FileCopyrightText: 2019 Brent Rubell for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// Adafruit IO - Analog Devices ADT7410 + ADXL343 Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Brent Rubell for Adafruit Industries
// Copyright (c) 2019 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************ Adafruit IO Config *******************************/
// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "melbaz1"
#define IO_KEY "aio_ddiz88gxucz5ihDNynIGq5QULFGs"

/******************************* WiFi Config ********************************/
//#define WIFI_SSID "SHAW-B231D0"
//#define WIFI_PASS "251167082106"

#define WIFI_SSID "SM-G973W3730"
#define WIFI_PASS "7802983369"


// comment out the following two lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

/************************** Configuration ***********************************/
// time between sending data to adafruit io, in seconds.
#define IO_DELAY 3
/************************ Example Starts Here *******************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_ADT7410.h"
#include <Adafruit_ADXL343.h>
#include <cmath>

#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include "base64.hpp"
#include "CloudIoTCoreDevice.h"
#include "ciotc_config.h" // Update this file with your configuration

#define CLOUD_IOT_CORE_HTTP_HOST "cloudiotdevice.googleapis.com"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
CloudIoTCoreDevice *device;

const int httpsPort = 443;
String apiPath;

float tempC, accelX, accelY, accelZ, accelAVG, baseline, change;

float baseline_readings[3] = {};

bool tap_off = true;

bool startup = true;

int ON_count = 0;

int ON_duration = 0;

bool tap_on = false;

int baseline_index = 0;

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

// Create the ADXL343 accelerometer sensor object
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

// set up the 'temperature' feed
AdafruitIO_Feed *huzzah_temperature = io.feed("temperature");

// set up the 'accelX' feed
AdafruitIO_Feed *huzzah_accel_x = io.feed("accelX");

// set up the 'accelY' feed
AdafruitIO_Feed *huzzah_accel_y = io.feed("accelY");

// set up the 'accelZ' feed
AdafruitIO_Feed *huzzah_accel_z= io.feed("accelZ");


AdafruitIO_Feed *huzzah_accel_avg = io.feed("accelAVG");


AdafruitIO_Feed *water_on_duration = io.feed("water_on_duration");


AdafruitIO_Feed *tap_state = io.feed("tap_state");


AdafruitIO_Feed *predicted_activity = io.feed("predicted_activity");


void setup()
{
  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while (!Serial)
    ;

  Serial.println("Adafruit IO - ADT7410 + ADX343");

  /* Initialise the ADXL343 */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_2_G);

  /* Initialise the ADT7410 */
  if (!tempsensor.begin())
  {
    Serial.println("Couldn't find ADT7410!");
    while (1)
      ;
  }

  // sensor takes 250 ms to get first readings
  delay(250);

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}


void sendToCloud(unsigned char string[]) {
  WiFiClientSecure wifi;
  wifi.setInsecure();//skip verification
  long long int timeNow = timeClient.getEpochTime();
  
  wdt_reset();
  wdt_disable();
  String jwt = device->createJWT(timeNow, jwt_exp_secs);
  wdt_enable(0);
  wdt_reset();
  unsigned char base64[21]; // 20 bytes for output + 1 for null terminator
  // encode_base64() places a null terminator automatically, because the output is a string
  
  int base64_length = encode_base64(string, strlen((char *) string), base64);
  String body = String("{\"binary_data\":\"") + (char *) base64 + "\"}";

  if (!wifi.connect(CLOUD_IOT_CORE_HTTP_HOST, httpsPort)) {
    Serial.println("Connection failed!");
  }
  else {
    Serial.println("Connected to server!");
    wifi.println("POST " + apiPath + " HTTP/1.1");
    wifi.println(String("host: ") + CLOUD_IOT_CORE_HTTP_HOST);
    wifi.println("authorization: " + String("Bearer ") + jwt);
    wifi.println("content-type: application/json");
    wifi.println("Connection: close");
    wifi.println("cache-control: no-cache");
    wifi.print("Content-Length: ");
    wifi.println(body.length());
    wifi.println();
    wifi.println(body);


    while (wifi.connected()) {
      String line = wifi.readStringUntil('\n');
      if (line == "\r") {
        Serial.println("headers received");
        break;
      }
    }

    while (wifi.available()) {
      char c = wifi.read();
      Serial.write(c);
    }
    wifi.stop();
  }

}


void loop()
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

   /* Get a new accel. sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  //accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  //accelZ = event.acceleration.z;
  //accelAVG = (accelX + accelY + accelZ)/3;


  if(tap_off == true && startup == true){
    baseline_readings[baseline_index] = accelY;
    baseline_index++;

    if(baseline_index == 3){
      baseline = (baseline_readings[0]+baseline_readings[1]+baseline_readings[2])/3;
      startup = false;
    }
  }

  if(baseline_index == 3){
    change = (abs(accelY-baseline)/baseline)*100;

    if(change > 2.5){
      tap_on = true;
      tap_off = false;
      ON_count++;
      Serial.println("Tap is ON! Sending tap state to Adafruit IO...");
      tap_state->save(1);
    }
    else{
      tap_on = false;
      tap_off = true;

      Serial.println("Tap is OFF! Sending tap state to Adafruit IO...");
      tap_state->save(0);
      
      if(ON_count > 0){
        ON_duration = ON_count * IO_DELAY;
        Serial.println("Sending duration of tap turned ON to Adafruit IO...");
        water_on_duration->save(ON_duration);
        ON_count = 0;

        Serial.println("Sending predicted activity to Adafruit IO...");
        if(ON_duration >= 3 && ON_duration < 15){
          predicted_activity->save("Filling up water bottle");
        }
        else if(ON_duration >= 15 && ON_duration < 25){
          predicted_activity->save("Washing hands or filling up a cooking pot");
        }
        else if(ON_duration >= 25){
          predicted_activity->save("Washing dishes");
        }
      }
    }
  }


  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("X: "); Serial.print(accelX); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(accelY); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(accelZ); Serial.print("  ");Serial.println("m/s^2 ");
  
  // Read and print out the temperature
//  tempC = tempsensor.readTempC();
//  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("C");

  //Serial.println("Sending to Adafruit IO...");
  //huzzah_temperature->save(tempC, 0, 0, 0, 2);
  //huzzah_accel_x->save(accelX);
  //huzzah_accel_y->save(accelY);
  //huzzah_accel_z->save(accelZ);
  //huzzah_accel_avg->save(accelAVG);
  //Serial.println("Data sent!");

  //Serial.print("Waiting ");Serial.print(IO_DELAY);Serial.println(" seconds...");
  // wait IO_DELAY seconds between sends
  for (int i = 0; i < IO_DELAY; i++)
  {
    delay(1000);
  }
}
